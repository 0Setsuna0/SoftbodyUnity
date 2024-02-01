using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

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
    public NativeArray<float> InvMass;

    public int iterationNum = 10;
    public bool drawFlag = true;

    public float invEdgeStiffness = 0.0f;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);
    void Awake()
    {
        MeshDataInit();
        
        InitPhysics();
    }

    // Update is called once per frame
    void Update()
    {
        float dt = Time.deltaTime / iterationNum;

        for (int i = 0; i < iterationNum; i++)
        {
            PreSolve(dt);
            
            SolveDistance(dt);
            
            PostSolve(dt);
        }
    }

    private void MeshDataInit()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        Triangles = new NativeArray<int>(mesh.triangles.Length, Allocator.Persistent);
        ConvertToNativeArray(mesh.triangles, Triangles);

        int triangleNum = Triangles.Length / 3;
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

            
            if (!edgeSet.Contains(edge1) && !edgeSet.Contains(edge1_inv))
            {
                edgeSet.Add(edge1);
            }
            if (!edgeSet.Contains(edge2) && !edgeSet.Contains(edge2_inv))
            {
                edgeSet.Add(edge2);
            }
            if (!edgeSet.Contains(edge3) && !edgeSet.Contains(edge3_inv))
            {
                edgeSet.Add(edge3);
            }
        }

        Edges = new NativeArray<int2>(edgeSet.Count, Allocator.Persistent);
        ConvertToNativeArray(edgeSet.ToArray(), Edges);
    }

    private void InitPhysics()
    {
        InvMass = new NativeArray<float>(mesh.vertices.Length, Allocator.Persistent);
        Pos = new NativeArray<Vector3>(mesh.vertices.Length, Allocator.Persistent);
        PrevPos = new NativeArray<Vector3>(mesh.vertices.Length, Allocator.Persistent);
        Vel = new NativeArray<Vector3>(mesh.vertices.Length, Allocator.Persistent);
        RestLength = new NativeArray<float>(Edges.Length, Allocator.Persistent);
        for (int i = 0; i < mesh.vertices.Length; i++)
        {
            Pos[i] = mesh.vertices[i];
            PrevPos[i] = mesh.vertices[i];
            InvMass[i] = 1.0f;
        }

        for (int i = 0; i < Edges.Length; i++)
        {
            int id0 = Edges[i][0];
            int id1 = Edges[i][1];
            RestLength[i] = (Pos[id0] - Pos[id1]).magnitude;
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

        JobHandle preSolveJobHandle = preSolveJob.Schedule(Pos.Length, 32);
        
        preSolveJobHandle.Complete();
    }
    
    private void SolveDistance(float dt_s)
    {
        float alpha = invEdgeStiffness / (dt_s * dt_s);
        ExSolveDistancePass solveDistanceJob = new ExSolveDistancePass()
        {
            _RestLength = RestLength,
            _EdgeIdx = Edges,
            _InvMass = InvMass,
            _Pos = Pos,
            _alpha = alpha,
            _Correction = Correction,
        };
    }

    private void SolveVolume(float dt_s)
    {
        
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

        JobHandle postSolveJobHandle = postSolveJob.Schedule(Pos.Length, 32);
        postSolveJobHandle.Complete();
        
        mesh.vertices = Pos.ToArray();
        mesh.RecalculateNormals();
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
