using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

struct VertexData
{
    public float3 position;
}
public class XPBDTetGPU : MonoBehaviour
{
    public MeshFilter meshFilter;
    public ComputeShader computeShader;
    public Material material;

    private ComputeBuffer vertexBuffer;
    private ComputeBuffer normalBuffer;
    
    float currentTime = 0f;
    public float speed = 0.001f;

    void Start()
    {
        meshFilter = GetComponent<MeshFilter>();
        Mesh mesh = meshFilter.mesh;
        
        VertexData[] verticesData = new VertexData[mesh.vertices.Length];
        for (int i = 0; i < mesh.vertices.Length; i++)
        {
            verticesData[i] = new VertexData { position = mesh.vertices[i] };
        }

        vertexBuffer = new ComputeBuffer(verticesData.Length, sizeof(float) * 3);
        vertexBuffer.SetData(verticesData);
        
        computeShader.SetBuffer(0, "vertices", vertexBuffer);

        Vector3[] normals = mesh.normals;
        ComputeBuffer normalBuffer = new ComputeBuffer(normals.Length, sizeof(float) * 3);
        normalBuffer.SetData(normals);
    
        computeShader.SetBuffer(0, "normals", normalBuffer);
    }

    void Update()
    {
        currentTime += Time.deltaTime;
        
        computeShader.SetFloat("deltaTime", Time.deltaTime);
        computeShader.SetFloat("currentTime", currentTime);
        computeShader.SetFloat("speed", speed);
        
        computeShader.Dispatch(0, (meshFilter.mesh.vertices.Length - 1) / 32 + 1, 1, 1);
        
        Mesh mesh = meshFilter.mesh;
        
        VertexData[] verticesData = new VertexData[mesh.vertices.Length];
        vertexBuffer.GetData(verticesData);
        
        Vector3[] newVertices = new Vector3[verticesData.Length];
        for (int i = 0; i < verticesData.Length; i++)
        {
            newVertices[i] = verticesData[i].position;
        }

        mesh.vertices = newVertices;
        
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    void OnDestroy()
    {
        vertexBuffer.Release();
    }
}
