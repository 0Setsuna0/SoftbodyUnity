using JetBrains.Annotations;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BoundingBoxSoftbody : MonoBehaviour
{
    public BoxCollider boxCollider;
    [Header("细分程度")]
    public int voxelResolution = 3;
    private int vertexNum;
    Vector3 voxelSize;

    [Header("顶点组")]
    public Vector3[] pointsPosition;
    public Vector3[] prevPointsPosition;

    private Vector3[] pointsVelocity;
    private float[] pointsMass;

    private int[,] facePointIdx;
    private int[,] tetIdx;
    private int[,] edgeIdx;

    private float[] edgeResLength;
    private float[] tetRestVolume;

    [SerializeField]
    //child mesh
    private MeshFilter childMeshFilter = null;
    private Bounds childMeshBounds = new Bounds();
    //father mesh
    private MeshFilter fatherMeshFilter = null;

    public int iterationStep = 0;
    public int subStepNum = 5;
    Transform transform;

    //simulation param
    float restVolume;
    // Start is called before the first frame update
    void Start()
    {
        //get component
        boxCollider = GetComponent<BoxCollider>();
        transform = GetComponent<Transform>();

        childMeshFilter = GetComponentInChildren<MeshFilter>();
        childMeshBounds = childMeshFilter.mesh.bounds;
        fatherMeshFilter = GetComponent<MeshFilter>();

        vertexNum = voxelResolution * voxelResolution * voxelResolution;
        pointsPosition = new Vector3[vertexNum];
        prevPointsPosition = new Vector3[vertexNum];
        pointsMass = new float[vertexNum];
        pointsVelocity = new Vector3[vertexNum];

        boxCollider.center = childMeshBounds.center;
        boxCollider.size = childMeshBounds.size;
        voxelSize = childMeshBounds.size / (voxelResolution - 1);
        restVolume = transform.localScale.x * transform.localScale.y * transform.localScale.z;

        edgeIdx = new int[voxelResolution * voxelResolution * voxelResolution, 18];
        
    }

    // Update is called once per frame
    void Update()
    {
        //XPBD simulation pass
        for(int i = 0; i < subStepNum; i++)
        {
            PredictPointPos();

            SolveConstrain();

            PointProjection();
        }

        //rebuild geometry point
        UpdateSoftVertexSystem();
    }

    void PredictPointPos()
    {
        pointsPosition.CopyTo(prevPointsPosition, 0);
        for (int i = 0; i < vertexNum; i++)
        {
            pointsPosition[i] += Time.deltaTime * pointsVelocity[i];
        }
    }

    void SolveConstrain()
    {

    }

    void PointProjection()
    {

    }

    void UpdateSoftVertexSystem()
    {
        //init points
        if (!boxCollider.Equals(null))
        {
            Vector3 boxColliderCenter = boxCollider.center;
            Vector3 boxColliderSize = boxCollider.size;

            Vector3 boxColliderHalfSize = boxColliderSize / 2;
            Vector3 boxColliderOrigin = boxColliderCenter - boxColliderHalfSize;
            for (int i = 0; i < voxelResolution; i++)
            {
                for (int j = 0; j < voxelResolution; j++)
                {
                    for (int k = 0; k < voxelResolution; k++)
                    {
                        pointsPosition[i * voxelResolution * voxelResolution + j * voxelResolution + k] =
                            transform.position + boxColliderOrigin + new Vector3(i * voxelSize.x, j * voxelSize.y, k * voxelSize.z);

                    }
                }
            }

            for (int i = 0; i < voxelResolution - 1; i++)
            {
                for (int j = 0; j < voxelResolution - 1; j++)
                {
                    for (int k = 0; k < voxelResolution - 1; k++)
                    {
                        // 获取当前体素的八个顶点索引
                        int p0 = i * voxelResolution * voxelResolution + j * voxelResolution + k;
                        int p1 = p0 + 1;
                        int p2 = p0 + voxelResolution;
                        int p3 = p2 + 1;
                        int p4 = p0 + voxelResolution * voxelResolution;
                        int p5 = p4 + 1;
                        int p6 = p4 + voxelResolution;
                        int p7 = p6 + 1;

                        // 添加当前体素的边索引
                        edgeIdx[p0, 0] = p0;
                        edgeIdx[p0, 1] = p1;
                        edgeIdx[p0, 2] = p1;
                        edgeIdx[p0, 3] = p3;
                        edgeIdx[p0, 4] = p3;
                        edgeIdx[p0, 5] = p2;
                        edgeIdx[p0, 6] = p2;
                        edgeIdx[p0, 7] = p0;
                        edgeIdx[p0, 8] = p0;
                        edgeIdx[p0, 9] = p4;
                        edgeIdx[p0, 10] = p1;
                        edgeIdx[p0, 11] = p5;
                        edgeIdx[p0, 12] = p3;
                        edgeIdx[p0, 13] = p7;
                        edgeIdx[p0, 14] = p2;
                        edgeIdx[p0, 15] = p6;
                        edgeIdx[p0, 16] = p4;
                        edgeIdx[p0, 17] = p5;

                    }
                }

            }
        }
        else
        {
            Debug.Log("require box collider component");
        }
    }



    //void InitialBoundingBoxPoint()
    //{
    //    pointMassPosition[0] = childMeshFilter.transform.TransformPoint(new Vector3(childMeshBounds.size.x, childMeshBounds.size.y, childMeshBounds.size.z));
    //    pointMassPosition[1] = childMeshFilter.transform.TransformPoint(new Vector3(-childMeshBounds.size.x, childMeshBounds.size.y, childMeshBounds.size.z));
    //    pointMassPosition[2] = childMeshFilter.transform.TransformPoint(new Vector3(childMeshBounds.size.x, -childMeshBounds.size.y, childMeshBounds.size.z));
    //    pointMassPosition[3] = childMeshFilter.transform.TransformPoint(new Vector3(-childMeshBounds.size.x, -childMeshBounds.size.y, childMeshBounds.size.z));
    //    pointMassPosition[4] = childMeshFilter.transform.TransformPoint(new Vector3(childMeshBounds.size.x, childMeshBounds.size.y, -childMeshBounds.size.z));
    //    pointMassPosition[5] = childMeshFilter.transform.TransformPoint(new Vector3(-childMeshBounds.size.x, childMeshBounds.size.y, -childMeshBounds.size.z));
    //    pointMassPosition[6] = childMeshFilter.transform.TransformPoint(new Vector3(childMeshBounds.size.x, -childMeshBounds.size.y, -childMeshBounds.size.z));
    //    pointMassPosition[7] = childMeshFilter.transform.TransformPoint(new Vector3(-childMeshBounds.size.x, -childMeshBounds.size.y, -childMeshBounds.size.z));
    //}

    void OnDrawGizmos()
    {
        // 在 Scene 视图中绘制顶点
        Gizmos.color = Color.blue;

        foreach (Vector3 point in pointsPosition)
        {
            Gizmos.DrawSphere(point, 0.01f);
        }

        for (int i = 0; i < voxelResolution * voxelResolution * voxelResolution; i++)
        {
            for (int j = 0; j < 18; j += 2)
            {
                int pointIndex1 = edgeIdx[i, j];
                int pointIndex2 = edgeIdx[i, j + 1];

                Vector3 point1 = pointsPosition[pointIndex1];
                Vector3 point2 = pointsPosition[pointIndex2];

                Gizmos.DrawLine(point1, point2);
            }
        }

    }
}
