using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Serialization;

public class ExpansionMesh : MonoBehaviour
{
    public MeshFilter meshFilter;
    public Mesh mesh;

    //shaders
    public ComputeShader constraintInitShader;
    public ComputeShader preSolveShader;
    public ComputeShader constraintSolveShader;
    public ComputeShader correctPosShader;
    public ComputeShader postSolveShader;
    
    //shader buffer
    public ComputeBuffer VertexPosBuffer;
    public ComputeBuffer PrevPosBuffer;
    public ComputeBuffer VelBuffer;
    public ComputeBuffer RestDistanceBuffer;
    public ComputeBuffer EdgeIndexBuffer;
    public ComputeBuffer InvMassBuffer;
    public ComputeBuffer CorrectionBuffer;
    
    //cpu data
    public float[] restLength;

    public Vector3[] vertexPos;
    //geometry
    private int[] triangles;
    private int2[] edges;

    //simulation
    public int iterationNum = 10;
    public Vector3 gravity;
    public float invEdgeStiffness = 0.0f;
    void Start()
    {
        MeshDataInit();
        
        ShaderBufferInit();
       
        InitConstraint();
    }
    void Update()
    {
        float dt = Time.deltaTime / iterationNum;
        
        for (int i = 0; i < iterationNum; i++)
        {
            Presolve(dt);
            
            //SolveDistance(dt);
            
            //PostSolve(dt);
            
            VertexPosBuffer.GetData(vertexPos);
            mesh.vertices = vertexPos;
            mesh.RecalculateNormals();
        }
    }
    
    private void MeshDataInit()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        triangles = mesh.triangles;
        int triangleNum = triangles.Length / 3;
        Debug.Log(triangleNum);
        HashSet<int2> edgeSet = new HashSet<int2>();
        for (int i = 0; i < triangleNum; i++)
        {
            int id0 = triangles[3 * i];
            int id1 = triangles[3 * i + 1];
            int id2 = triangles[3 * i + 2];

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

        edges = edgeSet.ToArray();
        restLength = new float[edges.Length];
        vertexPos = new Vector3[mesh.vertices.Length];
    }

    private void ShaderBufferInit()
    {
        VertexPosBuffer = new ComputeBuffer(mesh.vertices.Length, sizeof(float) * 3);
        PrevPosBuffer = new ComputeBuffer(mesh.vertices.Length, sizeof(float) * 3);
        CorrectionBuffer = new ComputeBuffer(mesh.vertices.Length, sizeof(float) * 3);
        VelBuffer = new ComputeBuffer(mesh.vertices.Length, sizeof(float) * 3);
        
        RestDistanceBuffer = new ComputeBuffer(edges.Length, sizeof(float));
        EdgeIndexBuffer = new ComputeBuffer(edges.Length, sizeof(int) * 2);

        InvMassBuffer = new ComputeBuffer(mesh.vertices.Length, sizeof(float));
        
        VertexPosBuffer.SetData(mesh.vertices);
        PrevPosBuffer.SetData(mesh.vertices);
        EdgeIndexBuffer.SetData(edges);
        float[] invMass = new float[mesh.vertices.Length];
        Array.Fill(invMass, 1.0f);
        InvMassBuffer.SetData(invMass);
        
        constraintInitShader.SetBuffer(0, "_vertexPos", VertexPosBuffer);
        constraintInitShader.SetBuffer(0, "_edgeIdx", EdgeIndexBuffer);
        constraintInitShader.SetBuffer(0, "_restDistance", RestDistanceBuffer);

        preSolveShader.SetBuffer(0, "_pos", VertexPosBuffer);
        preSolveShader.SetBuffer(0, "_vel", VelBuffer);
        preSolveShader.SetBuffer(0, "_prevPos", PrevPosBuffer);

        constraintSolveShader.SetBuffer(0, "_restLength", RestDistanceBuffer);
        constraintSolveShader.SetBuffer(0, "_invMass", InvMassBuffer);
        constraintSolveShader.SetBuffer(0, "_correction", CorrectionBuffer);
        constraintSolveShader.SetBuffer(0, "_pos", VertexPosBuffer);
        constraintSolveShader.SetBuffer(0, "_edgeIdx", EdgeIndexBuffer);

        correctPosShader.SetBuffer(0, "_correction", CorrectionBuffer);
        correctPosShader.SetBuffer(0, "_pos", VertexPosBuffer);
        
        postSolveShader.SetBuffer(0, "_pos", VertexPosBuffer);
        postSolveShader.SetBuffer(0, "_prevPos", PrevPosBuffer);
        postSolveShader.SetBuffer(0, "_vel", VelBuffer);

    }

    private void InitConstraint()
    {
        constraintInitShader.Dispatch(0, (edges.Length - 1) / 64 + 1, 1, 1);
        RestDistanceBuffer.GetData(restLength);
    }

    private void Presolve(float dt_s)
    {
        preSolveShader.SetFloat("_dt", dt_s);
        preSolveShader.SetVector("_gravity", gravity);
        preSolveShader.Dispatch(0, (mesh.vertices.Length - 1) / 64 + 1, 1, 1);
    }

    private void SolveDistance(float dt_s)
    {
        for (int i = 0; i < 10; i++)
        {
            float alpha = invEdgeStiffness / (dt_s * dt_s);
            constraintSolveShader.SetFloat("_alpha", alpha);
        
            constraintSolveShader.Dispatch(0, (edges.Length - 1) / 64 + 1, 1, 1);

            correctPosShader.SetBuffer(0, "_correction", CorrectionBuffer);
            correctPosShader.Dispatch(0, (mesh.vertices.Length - 1) / 64 + 1, 1, 1);
        }

    }

    private void PostSolve(float dt_s)
    {
        postSolveShader.SetFloat("_dt", dt_s);
        
        postSolveShader.Dispatch(0, (mesh.vertices.Length - 1) / 64 + 1, 1, 1);
    }
}
