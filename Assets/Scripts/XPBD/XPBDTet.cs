using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.Serialization;

public class XPBDTet : MonoBehaviour
{
    //geometry
    private Vector3[] Vertex;
    private float[] VFile;
    private int[] TetIdx;
    private int[] EdgeIdx;
    public int[] SurfaceIdx;
    public int numParticles = 1000;
    public int numTets = 100;
    public int numEdges = 1000;
    private int[,] VolIdOrder;

    //particle update 
    private Vector3[] Pos;
    private Vector3[] PrevPos;
    private Vector3[] Vel;

    //constraint
    public float edgeStiffness;
    public float volumeStiffness;
    
    public float edgeInsideConstraintCoef;
    public float edgeSurfaceConstraintCoef;
    public float velDamping = 0.95f;
    
    //tet
    public Vector3[] Temp = new Vector3[4];
    public Vector3[] Grad = new Vector3[4];
    
    //physics
    public float[] InvMass;
    public Vector3 gravity;
    
    //rendering
    public MeshFilter meshFilter;
    public MeshRenderer meshRenderer;
    
    //rest constraint
    public float[] RestVolumes;

    public float[] RestEdgeLength;

    public Material mat;

    public int iterationNum = 10;
    // Start is called before the first frame update
    void Start()
    {
        InitializeTetMesh("Assets/bunny.txt", ref numParticles, ref numTets, ref numEdges);
        
        Pos = new Vector3[numParticles];
        PrevPos = new Vector3[numParticles];
        Array.Copy(Vertex, PrevPos, numParticles);
        Array.Copy(Vertex, Pos, numParticles);
        Vel = new Vector3[numParticles];

        RestVolumes = new float[numTets];
        RestEdgeLength = new float[numEdges];
        
        InvMass = new float[numParticles];
        
        //build triangle mesh
        meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = gameObject.AddComponent<MeshRenderer>();
        meshFilter.mesh.vertices = Vertex;
        meshFilter.mesh.triangles = SurfaceIdx;
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

    // Update is called once per frame
    void Update()
    {
        float dt_s = Time.deltaTime / iterationNum;
        for (int i = 0; i < iterationNum; i++)
        {
            PredictPosition(dt_s);
            
            SolveConstraint(dt_s);
            
            CalculateVel(dt_s);
        }

        for (int i = 0; i < Vel.Length; i++)
        {
            Vel[i] *= velDamping;
        }
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

    VFile = vFileList.ToArray();
    TetIdx = tetIdxList.ToArray();
    EdgeIdx = edgeIdxList.ToArray();
    SurfaceIdx = surfaceIdxList.ToArray();

    // Create Vector3 array from VFile
    Vertex = new Vector3[numParticles];
    for (int i = 0; i < numParticles; i++)
    {
        Vertex[i] = new Vector3(VFile[3 * i], VFile[3 * i + 1], VFile[3 * i + 2]);
    }
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
        Array.Fill(InvMass, 0.0f);
        Array.Fill(RestVolumes, 0.0f);

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

    //pre solve pass
    private void PredictPosition(float dt)
    {
        for (int i = 0; i < numParticles; i++)
        {
            if(InvMass[i] == 0.0f)
                continue;

            Vel[i] += dt * gravity;
            PrevPos[i] = Pos[i];
            Pos[i] += dt * Vel[i];

            float y = Pos[i].y;
            if (y < 0.0f)
            {
                Pos[i] = PrevPos[i];
                Pos[i].y = 0.0f;
            }
        }
    }

    //solve constraint pass
    private void SolveConstraint(float dt)
    {
        SolveEdgeDistanceConstraint(dt);
        
        SolveTetVolumeConstraint(dt);
    }
    private void SolveEdgeDistanceConstraint(float dt)
    {
        float alpha = edgeStiffness / (dt * dt);

        for (int i = 0; i < numEdges; i++)
        {
            int id0 = EdgeIdx[2 * i];
            int id1 = EdgeIdx[2 * i + 1];

            float w0 = InvMass[id0];
            float w1 = InvMass[id1];
            float w = w0 + w1;
            if (w == 0.0f)
            {
                continue;
            }

            Grad[0] = Pos[id0] - Pos[id1];
            float len = Grad[0].magnitude;
            if(len == 0.0f)
                continue;
            Grad[0] /= len;
            float restLength = RestEdgeLength[i];
            float C = len - restLength;
            float s = -C / (w + alpha);

            Pos[id0] += Grad[0] * s * w0;
            Pos[id1] -= Grad[0] * s * w1;
        }
    }
    private void SolveTetVolumeConstraint(float dt)
    {
        float alpha = volumeStiffness / (dt * dt);

        for (int i = 0; i < numTets; i++)
        {
            float w = 0.0f;
            for (int j = 0; j < 4; j++)
            {
                //each face idx
                int id0 = TetIdx[4 * i + VolIdOrder[j, 0]];
                int id1 = TetIdx[4 * i + VolIdOrder[j, 1]];
                int id2 = TetIdx[4 * i + VolIdOrder[j, 2]];

                Temp[0] = Pos[id1] - Pos[id0];
                Temp[1] = Pos[id2] - Pos[id0];

                Grad[j] = Vector3.Cross(Temp[0], Temp[1]);
                Grad[j] /= 6.0f;

                w += InvMass[TetIdx[4 * i + j]] * Vector3.SqrMagnitude(Grad[j]);
            }

            if (w == 0.0f)
            {
                continue;
            }

            float vol = GetTetVolume(i);
            float restVol = RestVolumes[i];
            float C = vol - restVol;
            float s = -C / (w + alpha);

            for (int j = 0; j < 4; j++)
            {
                int id = TetIdx[4 * i + j];
                Pos[id] += Grad[j] * s * InvMass[id];
            }

        }
    }
    
    //post solve pass
    private void CalculateVel(float dt)
    {
        for (int i = 0; i < numParticles; i++)
        {
            if (InvMass[i] == 0.0)
            {
                continue;
            }

            Vel[i] = (Pos[i] - PrevPos[i]) / dt;
        }
        UpdateMeshData();
    }
    private void UpdateMeshData()
    {
        meshFilter.mesh.vertices = Pos;
        meshFilter.mesh.RecalculateNormals();
    }
}
