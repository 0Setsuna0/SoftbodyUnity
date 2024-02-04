using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

public class XPBDTetJob : MonoBehaviour
{
    //geometry
    private NativeArray<Vector3> Vertex;
    private NativeArray<float> VFile;
    private NativeArray<int> TetIdx;
    private NativeArray<int> EdgeIdx;
    public NativeArray<int> SurfaceIdx;
    public HashSet<int> surfacePoints;
    public int numParticles = 1000;
    public int numTets = 100;
    public int numEdges = 1000;
    private int[,] VolIdOrder;

    //particle update 
    private NativeArray<Vector3> Pos;
    private NativeArray<Vector3> PrevPos;
    private NativeArray<Vector3> Correction;
    
    private NativeArray<Vector3> Vel;

    //constraint
    public float invEdgeStiffness;
    private float invVolumeStiffness = 0;

    
    public float pressure = 1;
    public float pressured = 1;
    
    public float velDamping = 0.95f;
    
    //tet
    public NativeArray<Vector3> Temp;
    public NativeArray<Vector3> Grad;
    
    //physics
    public NativeArray<float> InvMass;
    public Vector3 gravity;
    public float floorHeight = 0.0f;
    
    //rendering
    public MeshFilter meshFilter;
    public MeshRenderer meshRenderer;
    
    //rest constraint
    public NativeArray<float> RestVolumes;

    public NativeArray<float> RestEdgeLength;

    public Material mat;

    public int iterationNum = 10;
    // Start is called before the first frame update
    void Start()
    {
        InitializeTetMesh("Assets/bunny.txt", ref numParticles, ref numTets, ref numEdges);
        
        Pos = new NativeArray<Vector3>(numParticles, Allocator.Persistent);
        PrevPos = new NativeArray<Vector3>(numParticles, Allocator.Persistent);
        Correction = new NativeArray<Vector3>(numParticles, Allocator.Persistent);
        NativeArray<Vector3>.Copy(Vertex, PrevPos, numParticles);
        NativeArray<Vector3>.Copy(Vertex, Pos, numParticles);
        Vel = new NativeArray<Vector3>(numParticles, Allocator.Persistent);
        
        RestVolumes = new NativeArray<float>(numTets, Allocator.Persistent);
        RestEdgeLength = new NativeArray<float>(numEdges, Allocator.Persistent);
        Temp = new NativeArray<Vector3>(4, Allocator.Persistent);
        Grad = new NativeArray<Vector3>(4, Allocator.Persistent);
        
        InvMass = new NativeArray<float>(numParticles, Allocator.Persistent);
        
        //build triangle mesh
        meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshFilter.mesh.vertices = Vertex.ToArray();
        meshFilter.mesh.triangles = SurfaceIdx.ToArray();
        meshFilter.mesh.RecalculateNormals();
        meshRenderer.material = mat;
        
        VolIdOrder = new int[4, 3]
        {
            { 1, 3, 2 },
            { 0, 2, 3 },
            { 0, 3, 1 },
            { 0, 1, 2 }
        };

        InitPhysics();
    }

    private void InitializeTetMesh(string filePath, ref int numParticles, ref int numTets, ref int numEdges)
    {
        string[] lines = File.ReadAllLines(filePath);

        List<float> vFileList = new List<float>();
        List<int> tetIdxList = new List<int>();
        List<int> edgeIdxList = new List<int>();
        List<int> surfaceIdxList = new List<int>();

        int flag = 0;

        foreach (string line in lines)
        {
            string[] values = line.Split(new char[] { ',', ' ' }, StringSplitOptions.RemoveEmptyEntries);
            if (values.Length == 1)
            {
                if (values[0] == "tetIds")
                {
                    flag = 1;
                }
                else if (values[0] == "tetEdgeIds")
                {
                    flag = 2;
                }
                else if (values[0] == "tetSurfaceTriIds")
                {
                    flag = 3;
                }
                
                continue;
            }

            for (int j = 0; j < values.Length; j++)
            {
                if (flag == 0)
                {
                    // handle vertex position data
                    vFileList.Add(float.Parse(values[j]));
                }
                else if (flag == 1)
                {
                    // handle tet idx data
                    tetIdxList.Add(int.Parse(values[j]));
                }
                else if (flag == 2)
                {
                    // handle edge idx data
                    edgeIdxList.Add(int.Parse(values[j]));
                }
                else if (flag == 3)
                {
                    // handle surface idx data
                    surfaceIdxList.Add(int.Parse(values[j]));
                }
            }
        }

        numParticles = vFileList.Count / 3;
        numTets = tetIdxList.Count / 4;
        numEdges = edgeIdxList.Count / 2;

        VFile = new NativeArray<float>(vFileList.ToArray(), Allocator.Persistent);
        TetIdx = new NativeArray<int>(tetIdxList.ToArray(), Allocator.Persistent);
        EdgeIdx = new NativeArray<int>(edgeIdxList.ToArray(), Allocator.Persistent);
        SurfaceIdx = new NativeArray<int>(surfaceIdxList.ToArray(), Allocator.Persistent);
        surfacePoints = new HashSet<int>(SurfaceIdx);
        // Create Vector3 array from VFile
        Vertex = new NativeArray<Vector3>(numParticles, Allocator.Persistent);
        for (int i = 0; i < numParticles; i++)
        {
            Vertex[i] = new Vector3(VFile[3 * i], VFile[3 * i + 1], VFile[3 * i + 2]);
        }

        VFile.Dispose();
    }
    
    private float GetTetVolume(int nr)
    {
        int id0 = TetIdx[4 * nr];
        int id1 = TetIdx[4 * nr + 1];
        int id2 = TetIdx[4 * nr + 2];
        int id3 = TetIdx[4 * nr + 3];
        Temp[0] = Pos[id1] - Pos[id0];
        Temp[1] = Pos[id2] - Pos[id0];
        Temp[2] = Pos[id3] - Pos[id0];
        Temp[3] = Vector3.Cross(Temp[0], Temp[1]);
        return Vector3.Dot(Temp[2], Temp[3]) / 6;
    }

    private void InitPhysics()
    {
        for (int i = 0; i < InvMass.Length; i++)
        {
            InvMass[i] = 0.0f;
        }

        for (int i = 0; i < RestVolumes.Length; i++)
        {
            RestVolumes[i] = 0.0f;
        }

        for (int i = 0; i < RestEdgeLength.Length; i++)
        {
            RestEdgeLength[i] = 0.0f;
        }

        for (int i = 0; i < numTets; i++)
        {
            //get i-th tetrahegon's volume
            float vol = GetTetVolume(i);
            RestVolumes[i] = vol;
            float pInvMass = vol > 0.0f ? 1.0f / (vol / 4) : 0.0f;
            InvMass[TetIdx[4 * i]] += pInvMass;
            InvMass[TetIdx[4 * i + 1]] += pInvMass;
            InvMass[TetIdx[4 * i + 2]] += pInvMass;
            InvMass[TetIdx[4 * i + 3]] += pInvMass;
        }

        for (int i = 0; i < numEdges; i++)
        {
            int id0 = EdgeIdx[2 * i];
            int id1 = EdgeIdx[2 * i + 1];
            RestEdgeLength[i] = (Pos[id1] - Pos[id0]).magnitude;
        }
        
    }
    // Update is called once per frame
    void Update()
    {
        float dt_s = Time.deltaTime / iterationNum;
        for (int i = 0; i < iterationNum; i++)
        {
            
            PredictPosition(dt_s);
            
            SolveConstraintXPBD(dt_s);
            
            CalculateVel(dt_s);
        }

        for (int i = 0; i < Vel.Length; i++)
        {
            Vel[i] *= velDamping;
        }
    }

    private void FixedUpdate()
    {
        Interaction();
    }


    //pre solve pass
    private void PredictPosition(float dt)
    {
        PresolvePass presolvePassJob = new PresolvePass()
        {
            _Gravity = this.gravity,
            _dt = dt,
            _InvMass = this.InvMass,
            _PrevPos = this.PrevPos,
            _Vel = this.Vel,
            _Pos = this.Pos,
        };

        JobHandle presolveJobHandle = presolvePassJob.Schedule(Pos.Length, 32);
        
        presolveJobHandle.Complete();
        
    }

    //solve constraint pass
    private void SolveConstraintXPBD(float dt)
    {
        SolveEdgeDistacneConstraintXPBDJob(dt);
        
        SolveTetVolumeConstraintXPBDJob(dt);
    }

    private void SolveEdgeDistacneConstraintXPBDJob(float dt)
    {
        float alpha = invEdgeStiffness / (dt * dt);
        SolveDistanceConstraintPass solveDistanceJob = new SolveDistanceConstraintPass()
        {
            _RestLength = RestEdgeLength,
            _EdgeIdx = EdgeIdx,
            _InvMass = InvMass,
            _Pos = Pos,
            _alpha = alpha,
            _coef = pressured,
            _Correction = Correction,
        };

        JobHandle solveDistanceHandle = solveDistanceJob.Schedule(numEdges, 32);
        solveDistanceHandle.Complete();

        CorrectPosPass correctPosJob = new CorrectPosPass()
        {
            _Correction = Correction,
            _Pos = Pos,
        };
        JobHandle correctPosHandle = correctPosJob.Schedule(numParticles, 32);
        correctPosHandle.Complete();
    }

    private void SolveTetVolumeConstraintXPBDJob(float dt)
    {
        float alpha = invVolumeStiffness / (dt * dt);
        SolveVolumeConstraintPass solveVolumeJob = new SolveVolumeConstraintPass()
        {
            _alpha = alpha,
            _TetIdx = TetIdx,
            _Pos = Pos,
            _InvMass = InvMass,
            _RestVolume = RestVolumes,
            _ExpansionCoef = pressured * pressured * pressured,
            _Correction = Correction
        };

        JobHandle solveVolumeHandle = solveVolumeJob.Schedule(numTets, 32);
        solveVolumeHandle.Complete();

        CorrectPosPass correctPosJob = new CorrectPosPass()
        {
            _Correction = Correction,
            _Pos = Pos,
        };
        JobHandle correctPosHandle = correctPosJob.Schedule(numParticles, 32);
        correctPosHandle.Complete();
    }
    
    
    //post solve pass
    private void CalculateVel(float dt)
    {
        
        PostsolvePass postsolvePassJob = new PostsolvePass()
        {
            _InvMass = this.InvMass,
            _Pos = this.Pos,
            _PrevPos = this.PrevPos,
            _Vel = this.Vel,
            _dt = dt,
        };
        
        JobHandle postsolveJobHandle = postsolvePassJob.Schedule(Vel.Length, 32);
        postsolveJobHandle.Complete();
        meshFilter.mesh.SetVertices(Pos);
        meshFilter.mesh.RecalculateNormals();
    }

    private void Interaction()
    {
        if (Input.GetKey(KeyCode.W))
        {
         
            Pos[200] += new Vector3(0,0.08f,0);
        }
        if (Input.GetKey(KeyCode.S))
        {
            
            Pos[200] -= new Vector3(0,0.08f,0);
        }
        if (Input.GetKey(KeyCode.A))
        {
            
            Pos[200] += new Vector3(0.08f,0,0);
        }
        if (Input.GetKey(KeyCode.D))
        {
            Pos[200] -= new Vector3(0.08f,0,0);
        }
    }

    private void OnApplicationQuit()
    {
        Pos.Dispose();
        PrevPos.Dispose();
        Correction.Dispose();
        Vel.Dispose();
        InvMass.Dispose();

        Vertex.Dispose();
        TetIdx.Dispose();
        EdgeIdx.Dispose();
        SurfaceIdx.Dispose();
        Temp.Dispose();
        Grad.Dispose();
        RestVolumes.Dispose();
        RestEdgeLength.Dispose();
    }
}
