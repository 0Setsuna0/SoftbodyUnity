using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.Mathematics;
using UnityEngine;

public class ExpansionMesh : MonoBehaviour
{
    public MeshFilter meshFilter;
    public Mesh mesh;

    public ComputeShader constraintInitShader;

    public ComputeBuffer VertexPosBuffer;
    public ComputeBuffer RestDistanceBuffer;

    public int[] triangles;
    public int2[] edges;

    void Start()
    {
        MeshDataInit();
        
        VertexPosBuffer = new ComputeBuffer(mesh.vertices.Length, sizeof(float) * 3);
        RestDistanceBuffer = new ComputeBuffer(mesh.triangles.Length * 3 / 2, sizeof(int) * 2);
        
        VertexPosBuffer.SetData(mesh.vertices);
        constraintInitShader.SetBuffer(0, "vertexPos", VertexPosBuffer);
        
    }

    private void MeshDataInit()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;

        triangles = mesh.triangles;
        int triangleNum = triangles.Length / 3;
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
    }
    
    void Update()
    {
        constraintInitShader.Dispatch(0, (mesh.vertices.Length - 1) / 64 + 1, 1, 1);
        Vector3[] vertexPos = new Vector3[mesh.vertices.Length];
        VertexPosBuffer.GetData(vertexPos);

        mesh.vertices = vertexPos;
        mesh.RecalculateNormals();
    }
}
