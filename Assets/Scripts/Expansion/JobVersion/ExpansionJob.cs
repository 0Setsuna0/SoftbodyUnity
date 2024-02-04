using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Security.Policy;
using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine.Serialization;

public class ExpansionJob : MonoBehaviour
{
    public MeshFilter meshFilter;
    public Mesh mesh;

    public NativeArray<int> Triangles;
    public NativeArray<int2> Edges;
    public NativeArray<Vector3> Pos;
    public NativeArray<Vector3> PrevPos;
    public NativeArray<Vector3> Correction;
    public NativeArray<Vector3> Vel;

    public NativeArray<float> RestLength;
    public NativeArray<float> DistanceLambda;
    public NativeArray<float> InvMass;
    public NativeArray<float> VolBuffer;

    
    public int outerIterationNum = 10;
    public int inerIterationNum = 1;
    public int triangleNum;
    public int vertexNum;
    public bool drawFlag = true;

    public float invEdgeStiffness = 0.0f;
    private float invVolumeStiffness = 0.0f;
    public float restVolume;
    public float currentVolume;
    public float pressure = 1;
    public float VolumeLambda = 0;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);
    void Awake()
    {
        MeshDataInit();
        
        InitPhysics();
    }

    // Update is called once per frame
    void Update()
    {
        float dt = Time.deltaTime / outerIterationNum;

        for (int i = 0; i < outerIterationNum; i++)
        {
            PreSolve(dt);
            
            for (int j = 0; j < inerIterationNum; j++)
            {
                SolveDistance(dt);
                
                SolveVolume(dt);
            }
            
            PostSolve(dt);
        }
    }

    private void MeshDataInit()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        vertexNum = mesh.vertices.Length;
        Triangles = new NativeArray<int>(mesh.triangles.Length, Allocator.Persistent);
        ConvertToNativeArray(mesh.triangles, Triangles);

        triangleNum = Triangles.Length / 3;
    }

    private void InitPhysics()
    {
        //remove reapting vertex index
        int index = 0;
        Dictionary<Vector3, int> posDictionary = new Dictionary<Vector3, int>();
        for (int i = 0; i < mesh.vertices.Length; i++)
        {
            if (!posDictionary.ContainsKey(mesh.vertices[i]))
            {
                posDictionary.Add(mesh.vertices[i], index++);
            }
        }
        Debug.Log(posDictionary.Count);
        vertexNum = posDictionary.Count;

        for (int i = 0; i < Triangles.Length; i++)
        {
            Triangles[i] = posDictionary[mesh.vertices[Triangles[i]]];
        }

        HashSet<int2> edgeSet = new HashSet<int2>();
        for (int i = 0; i < triangleNum; i++)
        {
            int id0 = Triangles[3 * i];
            int id1 = Triangles[3 * i + 1];
            int id2 = Triangles[3 * i + 2];

            int2 edge1 = new int2(id0, id1);
            int2 edge1_inv = new int2(id1, id0);
            int2 edge2 = new int2(id1, id2);
            int2 edge2_inv = new int2(id2, id1);
            int2 edge3 = new int2(id2, id0);
            int2 edge3_inv = new int2(id0, id2);

            
            if (!edgeSet.Contains(edge1))
            {
                edgeSet.Add(edge1);
            }

            if (!edgeSet.Contains(edge1_inv))
            {
                edgeSet.Add(edge1_inv);
            }
            if (!edgeSet.Contains(edge2))
            {
                edgeSet.Add(edge2);
            }

            if (!edgeSet.Contains(edge2_inv))
            {
                edgeSet.Add(edge2_inv);
            }
            if (!edgeSet.Contains(edge3))
            {
                edgeSet.Add(edge3);
            }

            if (!edgeSet.Contains(edge3_inv))
            {
                edgeSet.Add(edge3_inv);
            }
        }

        Edges = new NativeArray<int2>(edgeSet.Count, Allocator.Persistent);
        ConvertToNativeArray(edgeSet.ToArray(), Edges);
        
        InvMass = new NativeArray<float>(vertexNum, Allocator.Persistent);
        Pos = new NativeArray<Vector3>(vertexNum, Allocator.Persistent);
        PrevPos = new NativeArray<Vector3>(vertexNum, Allocator.Persistent);
        Correction = new NativeArray<Vector3>(vertexNum, Allocator.Persistent);
        Vel = new NativeArray<Vector3>(vertexNum, Allocator.Persistent);
        VolBuffer = new NativeArray<float>(1, Allocator.Persistent);
        
        RestLength = new NativeArray<float>(Edges.Length, Allocator.Persistent);
        DistanceLambda = new NativeArray<float>(Edges.Length, Allocator.Persistent);
        
        for (int i = 0; i < posDictionary.Keys.Count; i++)
        {
            Vector3 originalPos = posDictionary.Keys.ElementAt(i);
            int newIndex = posDictionary[originalPos];
            
            Pos[newIndex] = originalPos;
            PrevPos[newIndex] = originalPos;
            
            InvMass[newIndex] = 1.0f;
        }
        
        for (int i = 0; i < Edges.Length; i++)
        {
            int id0 = Edges[i][0];
            int id1 = Edges[i][1];
            RestLength[i] = (Pos[id0] - Pos[id1]).magnitude;
        }

        for (int i = 0; i < Triangles.Length / 3; i++)
        {
            int id0 = Triangles[3 * i];
            int id1 = Triangles[3 * i + 1];
            int id2 = Triangles[3 * i + 2];
            restVolume += Vector3.Dot(Pos[id0], Vector3.Cross(Pos[id1], Pos[id2])) / 6;
        }
    }

    private void PreSolve(float dt_s)
    {
        ExPreSolvePass preSolveJob = new ExPreSolvePass()
        {
            _Gravity = this.gravity,
            _dt = dt_s,
            _InvMass = this.InvMass,
            _PrevPos = this.PrevPos,
            _Vel = this.Vel,
            _Pos = this.Pos,
        };

        JobHandle preSolveJobHandle = preSolveJob.Schedule(vertexNum, 32);
        
        preSolveJobHandle.Complete();
    }
    
    private void SolveDistance(float dt_s)
    {
        float alpha = invEdgeStiffness / (dt_s * dt_s);
        ExSolveDistancePass solveDistanceJob = new ExSolveDistancePass()
        {
            _RestLength = this.RestLength,
            _EdgeIdx = this.Edges,
            _InvMass = this.InvMass,
            _Pos = this.Pos,
            _alpha = alpha,
            _Correction = this.Correction,
            _Lambda = this.DistanceLambda
        };

        JobHandle solveDistanceJobHandle = solveDistanceJob.Schedule(Edges.Length, 32);
        solveDistanceJobHandle.Complete();

        ExCorrectionPass correctJob = new ExCorrectionPass()
        {
            _Pos = this.Pos,
            _Correction = this.Correction,
        };

        JobHandle correctJobHandle = correctJob.Schedule(vertexNum, 32);
        correctJobHandle.Complete();
    }

    private void SolveVolume(float dt_s)
    {
        float alpha = invVolumeStiffness / (dt_s * dt_s);
        
        currentVolume = 0.0f;
        for (int i = 0; i < triangleNum; i++)
        {
            int id0 = Triangles[3 * i];
            int id1 = Triangles[3 * i + 1];
            int id2 = Triangles[3 * i + 2];
            currentVolume += Vector3.Dot(Pos[id0], Vector3.Cross(Pos[id1], Pos[id2])) / 6;
        }

        currentVolume = Mathf.Abs(currentVolume);
        float C = currentVolume - pressure * restVolume;
        
        //gradient of C
        ExSolveVolumeSubPass1 solveVolumeSubJob1 = new ExSolveVolumeSubPass1()
        {
            _Pos = this.Pos,
            _Correction = this.Correction,
            _Triangles = this.Triangles,
        };

        JobHandle solveVolumeSubJobHandle1 = solveVolumeSubJob1.Schedule(triangleNum, 32);
        solveVolumeSubJobHandle1.Complete();
        
        //K
        float K = alpha;
        for (int i = 0; i < vertexNum; i++)
        {
            K += InvMass[i] * Vector3.Dot(Correction[i], Correction[i]);
            //Correction[i] = Vector3.zero;
        }
       
        //correct
        float deltaLambda = (-C - alpha * VolumeLambda) / K;
        ExSolveVolumeSubPass2 solveVolumeSubJob2 = new ExSolveVolumeSubPass2()
        {
            _Pos = this.Pos,
            _Correction = this.Correction,
            _InvMass = this.InvMass,
            _DeltaLambda = deltaLambda,
        };

        JobHandle solveVolumeSubJobHandle2 = solveVolumeSubJob2.Schedule(vertexNum, 32);
        solveVolumeSubJobHandle2.Complete();

        //update lambda
        VolumeLambda += deltaLambda;
    }
    
    private void PostSolve(float dt_s)
    {
        ExPostSolvePass postSolveJob = new ExPostSolvePass()
        {
            _InvMass = this.InvMass,
            _Pos = this.Pos,
            _PrevPos = this.PrevPos,
            _dt = dt_s,
            _Vel = this.Vel,
        };

        JobHandle postSolveJobHandle = postSolveJob.Schedule(vertexNum, 32);
        postSolveJobHandle.Complete();

        ExReSetXPBDParam reSetXpbdParam = new ExReSetXPBDParam()
        {
            _DistanceLambda = this.DistanceLambda
        };
        JobHandle resetXpbdJobHandle = reSetXpbdParam.Schedule(Edges.Length, 32);
        resetXpbdJobHandle.Complete();

        VolumeLambda = 0.0f;

        mesh.triangles = Triangles.ToArray();
        mesh.vertices = Pos.ToArray();
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }
    public void ConvertToNativeArray<T>(T[] managedArray, NativeArray<T> dst) where T : struct
    {
        for (int i = 0; i < managedArray.Length; i++)
        {
            dst[i] = managedArray[i];
        }
    }

    private void OnDrawGizmos()
    {
        if (Edges.Length == 0 || Pos.Length == 0 || !drawFlag)
        {
            return;
        }

        Gizmos.color = Color.blue;
        for (int i = 0; i < Edges.Length; i++)
        {
            Vector3 a = Pos[Edges[i][0]];
            Vector3 b = Pos[Edges[i][1]];
            
            Gizmos.DrawLine(a, b);
        }
    }

    private void OnDestroy()
    {
        Pos.Dispose();
        PrevPos.Dispose();
        Correction.Dispose();
        Vel.Dispose();
        InvMass.Dispose();

        RestLength.Dispose();
        Edges.Dispose();
        Triangles.Dispose();
        
    }
}
