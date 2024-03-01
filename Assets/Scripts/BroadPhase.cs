using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BroadPhase : MonoBehaviour
{
    #region soft body simulation inspector coefficients
    [Range(0.0f, 100.0f)]
    [Tooltip("Strength of forces to maintain constant volume")]
    public float PressureMultiplier = 50.0f;

    [Range(20.0f, 100.0f)]
    [Tooltip("Resistance against deformation")]
    public float SpringStiffness = 50.0f;

    [Range(1.5f, 10.0f)]
    [Tooltip("Resistance against jiggle")]
    public float SpringDamping = 1.5f;

    [Range(0.0f, 1.0f)]
    [Tooltip("Percentage of impact velocity reflected")]
    public float BounceCoefficient = .05f;

    [Range(0.0f, 1.0f)]
    [Tooltip("Percentage of impact velocity maintained to slide along surface")]
    public float SlideCoefficient = .99f;

    [Range(0.02f, 1.2f)] [Tooltip("User defined compression coef")]
    public float compressionCoef;
    #endregion

    #region jumping: inspector properties and private state
    [Range(-20.0f, 20.0f)]
    [Tooltip("Negative values create a procedural build up animation before the body rebounds")]
    public float JumpSpeed = -8.5f;

    [Range(0.0f, 10.0f)]
    [Tooltip("Min. seconds between land and jump")]
    public float JumpWaitMin = 3.0f;

    [Range(0.0f, 10.0f)]
    [Tooltip("Max. seconds between land and jump")]
    public float JumpWaitMax = 4.0f;

    public bool dynamic = true;
    #endregion

    #region point mass and hull face private state
    private Vector3[] PointMassAccelerations;
    private Vector3[] PointMassVelocities;
    private Vector3[] PointMassPositions;

    private Vector3[] RestOffset;
    
    private Vector3[] FaceNormals;
    private float[] FaceAreas;
    #endregion

    #region private state, read only after Awake
    private int[,] FacePointMassIndexes;
    private int[,] DiagonalSpringPointMassIndexes;
    private float[] SpringRestLengths;
    private float HullRestVolume;
    private Vector3 TransformStartPosition;
    private Vector3 TransformStartScale;

    private const int INVALID_LAYER = -1;
    private static int PlayAreaTriggerLayer = INVALID_LAYER;
    #endregion

    // components to cache for use during updates
    // TODO write an ECS/Job System version and see how the perf. is for naive custom collision testing
    private BoxCollider TriggerCollider;
    
    public SkinnedMeshRenderer ChildMeshRenderer;
    public Mesh childMesh;
    public Bounds childMeshBounds;

    public Vector3 MappingCoef;
    public Transform childTransform;
    [SerializeField]
    private Vector3 initialOffset;
    public float initialHeight;
    
    public bool debugModel = true;
    
    private void Awake()
    {
        TriggerCollider = GetComponent<BoxCollider>();
        ChildMeshRenderer = GetComponentInChildren<SkinnedMeshRenderer>();

        childMesh = ChildMeshRenderer.sharedMesh;
        childMeshBounds = childMesh.bounds;
        
        // 获取子物体的局部包围盒中心
        Vector3 localBoundsCenter = childMeshBounds.center;

        // 将局部包围盒中心转换为世界坐标
        Vector3 worldBoundsCenter = childTransform.TransformPoint(localBoundsCenter);

        // 使用子物体网格的包围盒的世界中心来设置父物体的 BoxCollider 中心
        TriggerCollider.center = transform.InverseTransformPoint(worldBoundsCenter);

        // 考虑子物体的缩放，使用子物体网格的包围盒的大小和子物体的缩放来设置父物体的 BoxCollider 大小
        Vector3 scaledBoundsSize = new Vector3(
            childMeshBounds.size.x * childTransform.localScale.x,
            childMeshBounds.size.y * childTransform.localScale.y,
            childMeshBounds.size.z * childTransform.localScale.z
        );

        TriggerCollider.size = scaledBoundsSize;
        
        // this soft body simulation uses a bounding box shaped (for simplicity) mass spring lattice 
        // to enclose the mesh to be deformed
        PointMassAccelerations = new Vector3[8];
        PointMassVelocities = new Vector3[8];
        PointMassPositions = new Vector3[8];
        RestOffset = new Vector3[8];

        InitializePointMassPositionsToBoundingBox();
        InitializePointMassIndexesForBoundingBox();
        SaveSpringRestLengths();

        int numFaces = FacePointMassIndexes.GetLength(0);
        FaceNormals = new Vector3[numFaces];
        FaceAreas = new float[numFaces];

        MappingCoef = new Vector3(TriggerCollider.size.x / transform.localScale.x,
            TriggerCollider.size.y / transform.localScale.y, TriggerCollider.size.z / transform.localScale.z);
        
        HullRestVolume = TriggerCollider.size.x * TriggerCollider.size.y * TriggerCollider.size.z;
        
        TransformStartPosition = transform.position;
        TransformStartScale = transform.localScale;

        initialOffset = transform.position - worldBoundsCenter;
        if (PlayAreaTriggerLayer == INVALID_LAYER)
        {
            // static, just initialize once
            PlayAreaTriggerLayer = LayerMask.NameToLayer("Play Area"); 
        }

        initialHeight = PointMassPositions[0].y - PointMassPositions[4].y;
    }

    private void InitializePointMassPositionsToBoundingBox()
    {
        // 使用子物体网格的包围盒
        Bounds childMeshBounds = childMesh.bounds;
        Vector3 center = childTransform.TransformPoint(childMeshBounds.center);
        
        // 获取子物体网格的包围盒的8个角的位置
        Vector3[] corners = new Vector3[8];
        corners[0] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        corners[1] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[2] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[3] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        corners[4] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, -childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        corners[5] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, -childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[6] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, -childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[7] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, -childMeshBounds.size.y, childMeshBounds.size.z) / 2;

        // 转换角的位置到世界坐标系
        for (int i = 0; i < 8; i++)
        {
            corners[i] = childTransform.TransformPoint(corners[i]);
            RestOffset[i] = corners[i] - childTransform.TransformPoint(childMeshBounds.center);
        }

        // 设置点的位置
        PointMassPositions = corners;
        
    }

    private void InitializePointMassIndexesForBoundingBox()
    {
        // Initialize the arrays that hold point mass indexes

        // The first index array has sets of 4 indexes for each of the 6 squares
        // that form the faces of the point mass hull (as a bounding box).
        FacePointMassIndexes =
                new int[6, 4]
                    { {3, 2, 1, 0}, {1, 5, 4, 0}, {2, 6, 5, 1}, {3, 7, 6, 2},
                    {0, 4, 7, 3}, {4, 5, 6, 7} };

        // The second index array enumerates the remaining springs, which are diagonal, since
        // they are not on the edges of the faces, and each spring is defined as a pair of indexes.
        DiagonalSpringPointMassIndexes =
            new int[16, 2]
                {
                    // crossing inside hull
                    {0, 6}, {1, 7}, {2, 4}, {3, 5}, 

                    // diagonals on hull
                    {3, 1}, {2, 0},
                    {0, 5}, {4, 1},
                    {1, 6}, {5, 2},
                    {2, 7}, {6, 3},
                    {3, 4}, {7, 0},
                    {4, 6}, {5, 7},
                };
    }

    private void SaveSpringRestLengths()
    {
        // calculate spring rest lengths
        int springIndex = 0;
        int numFaceEdgeSprings = FacePointMassIndexes.GetLength(0) * FacePointMassIndexes.GetLength(1);
        int numDiagonalSprings = DiagonalSpringPointMassIndexes.GetLength(0);
        SpringRestLengths = new float[numFaceEdgeSprings + numDiagonalSprings];

        // first the springs on the face edges of the hull
        int numFaces = FacePointMassIndexes.GetLength(0);
        int numPtsPerFace = FacePointMassIndexes.GetLength(1);
        for (int i = 0; i < numFaces; ++i)
        {
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                int pt0Index = FacePointMassIndexes[i, j];
                int pt1Index = FacePointMassIndexes[i, (j + 1) % numPtsPerFace];
                SpringRestLengths[springIndex] =
                    (PointMassPositions[pt0Index] - PointMassPositions[pt1Index]).magnitude;
                ++springIndex;
            }
        }

        // now calculate the rest lengths of the springs that aren't face edges on the hull
        for (int i = 0; i < numDiagonalSprings; ++i)
        {
            int pt0Index = DiagonalSpringPointMassIndexes[i, 0];
            int pt1Index = DiagonalSpringPointMassIndexes[i, 1];
            SpringRestLengths[springIndex] =
                (PointMassPositions[pt0Index] - PointMassPositions[pt1Index]).magnitude;
            ++springIndex;
        }
    }

    // Places the soft body back where it started, and resets the simulation state
    private void Respawn()
    {
        transform.position = TransformStartPosition;
        transform.localScale = TransformStartScale;
        InitializePointMassPositionsToBoundingBox();
        for (int i = 0; i < PointMassVelocities.Length; ++i)
        {
            PointMassVelocities[i] = new Vector3(0.0f, 0.0f, 0.0f); // (I tend to avoid the method calls from properties such as Vector3.zero)
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        // track landing on a surface, for the jump behavior
        if (other.gameObject.layer != PlayAreaTriggerLayer && 
            other.transform.position.y < transform.position.y)
        {
        }
    }

    private void OnTriggerStay(Collider other)
    {
        // overlapping with another collider, so make sure the point masses don't interpenetrate,
        // and resolve their velocities for the collision
        Vector3 depenetrationDir;
        float depenetrationDist;
        if (other.gameObject.layer != PlayAreaTriggerLayer &&
            Physics.ComputePenetration(TriggerCollider, transform.position, transform.rotation,
                                       other, other.transform.position, other.transform.rotation,
                                       out depenetrationDir, out depenetrationDist))
        {
            //Debug.LogFormat("Collision normal {0}", depenetrationDir);
            for (int i = 0; i < PointMassPositions.Length; ++i)
            {
                Vector3 p = PointMassPositions[i];
                if (other.bounds.Contains(p))
                {
                    // clamp interpenetrating point mass to surface of other collider
                    PointMassPositions[i] = 
                        other.ClosestPoint(p + depenetrationDir * (depenetrationDist + 1.0f));

                    // reflect component of velocity along other collider normal
                    // while maintaining the remainder of velocity, but reduce by
                    // energy loss coefficient
                    // (approximate average contact normals as depenetration direction)
                    float speedAlongNormalSigned = Vector3.Dot(PointMassVelocities[i], depenetrationDir);
                    float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
                    Vector3 velocityAlongNormal = speedAlongNormalSigned * depenetrationDir;
                    Vector3 slideVelocity = PointMassVelocities[i] - velocityAlongNormal;
                    velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing

                    // reduce velocityAlongNormal by bounce coefficient if reflecting
                    float bounceCoefficient = (speedAlongNormalSign >= 0.0f ? 1.0f : BounceCoefficient);
                    PointMassVelocities[i] = 
                        bounceCoefficient * velocityAlongNormal + 
                        slideVelocity * SlideCoefficient;                        
                }
            }
        }
    }

    private void OnTriggerExit(Collider other)
    {
        if (other.gameObject.layer == PlayAreaTriggerLayer)
        {
            Debug.LogWarningFormat("{0}>{1} left the play area, and will be respawned", 
                                    transform.parent == null ? "(no parent)" : transform.parent.gameObject.name,                    
                                    gameObject.name);
            //Respawn();
        }
    }

    // I've chosen fixed update to remove the variable of step size when investigating numeric
    // stability of the simulation.
    void FixedUpdate()
    {
        // accumulate forces for this update, start with gravity
        for (int i = 0; i < PointMassAccelerations.Length; ++i)
        {
            // simplify force to acceleration, as mass == 1
            PointMassAccelerations[i] = Physics.gravity / 2;
        }
        
        ApplyCoef();
        
        AccumulateSpringForces();
        AccumulatePressureForces();
       
        SolveForVelocitiesAndPositions(Time.fixedDeltaTime);
    }

    // Calculates the force on each spring and adds it to Accelerations 
    private void AccumulateSpringForces()
    {
        int springIndex = 0;
        // first the springs on the face edges of the hull
        int numFaces = FacePointMassIndexes.GetLength(0);
        int numPtsPerFace = FacePointMassIndexes.GetLength(1);
        for (int i = 0; i < numFaces; ++i)
        {
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                int pt0Index = FacePointMassIndexes[i, j];
                int pt1Index = FacePointMassIndexes[i, (j + 1) % numPtsPerFace];
                Vector3 springForce =
                    CalcSpringForce(pt0Index, pt1Index, SpringRestLengths[springIndex],
                                    SpringStiffness, SpringDamping);

                if (HasNaN(springForce))
                {
                    // This bug was due to division by float.PositiveInfinity in the faceNormal
                    // calculation, after the positions became too far apart due to too low
                    // damping value and too high velocities. It seems to be fixed by limiting
                    // the minimum damping value.
                    //Debug.LogWarningFormat("{0}>{1} calculated NaN for face edge spring force",
                    //    transform.parent == null ? "(no parent)" : transform.parent.gameObject.name,
                    //    gameObject.name);
                }
                else
                {
                    // mass == 1, so a = F/m = F
                    PointMassAccelerations[pt0Index] += springForce;
                    PointMassAccelerations[pt1Index] -= springForce;
                }
                ++springIndex;
            }
        }

        // now accumulate the springs inside the hull
        int numDiagonalSprings = DiagonalSpringPointMassIndexes.GetLength(0);
        for (int i = 0; i < numDiagonalSprings; ++i)
        {
            int pt0Index = DiagonalSpringPointMassIndexes[i, 0];
            int pt1Index = DiagonalSpringPointMassIndexes[i, 1];
            Vector3 springForce =
                CalcSpringForce(pt0Index, pt1Index, SpringRestLengths[springIndex],
                                SpringStiffness, SpringDamping);
            if (HasNaN(springForce))
            {
                // This bug was due to division by float.PositiveInfinity in the faceNormal
                // calculation, after the positions became too far apart due to too low
                // damping value and too high velocities. It seems to be fixed by limiting
                // the minimum damping value.
                //Debug.LogWarningFormat("{0}>{1} calculated NaN for diagonal spring force",
                //    transform.parent == null ? "(no parent)" : transform.parent.gameObject.name,
                //    gameObject.name);
            }
            else
            {
                PointMassAccelerations[pt0Index] += springForce;
                PointMassAccelerations[pt1Index] -= springForce;
            }
            ++springIndex;
        }
    }

    // Calculates the force on each set of point masses forming a hull face, and adds it to Accelerations 
    //
    // estimate volume and pressure forces, similar to pseudocode: https://arxiv.org/ftp/physics/papers/0407/0407003.pdf
    // but replace the Ideal Gas Law approximation with a formula that is easier to tune for
    // an approximately fixed volume fluid (like water)
    private void AccumulatePressureForces()
    {
        float volume = transform.localScale.x * transform.localScale.y * transform.localScale.z;

        //calculate ratio
        float volumeRatio = volume / HullRestVolume;
        float surfaceArea = 0.0f;
        int numFaces = FacePointMassIndexes.GetLength(0);
        int numPtsPerFace = FacePointMassIndexes.GetLength(1);
        //Debug.LogFormat("volume {0}, ratio {1}", volume, volumeRatio);
        Debug.Assert(numPtsPerFace == 4); // assume rectangles

        // cache normals and area while calculating total surface area
        for (int i = 0; i < numFaces; ++i)
        {
            Vector3 a = PointMassPositions[FacePointMassIndexes[i, 0]];
            Vector3 b = PointMassPositions[FacePointMassIndexes[i, 1]];
            Vector3 c = PointMassPositions[FacePointMassIndexes[i, 2]];

            Vector3 faceNormal = CalcCross(a, b, c);
            float faceArea = faceNormal.magnitude; // magnitude of cross is area of parallelogram
            if (float.IsInfinity(faceArea))
            {
                // This bug was due to division by float.PositiveInfinity in the faceNormal
                // calculation, after the positions became too far apart due to too low
                // damping value and too high velocities. It seems to be fixed by limiting
                // the minimum damping value.
                //Debug.LogWarningFormat("{0}>{1} calculated infinity for face area, positions are too far apart",
                //                        transform.parent == null ? "(no parent)" : transform.parent.gameObject.name,
                //                        gameObject.name);                 

                // provide some finite values by using the starting top face
                faceNormal = new Vector3(0.0f, 1.0f, 0.0f);
                faceArea = TransformStartScale.x * TransformStartScale.z;
            }
            else if (faceArea > 0.0f)
            {
                // normalize the cross product
                faceNormal /= faceArea;
            }

            FaceNormals[i] = faceNormal;
            FaceAreas[i] = faceArea;

            surfaceArea += faceArea;
        }

        for (int i = 0; i < numFaces; ++i)
        {
            Vector3 faceNormal = FaceNormals[i];
            float faceArea = FaceAreas[i];

            // approximate force distribution through fluid by using more force on the 
            // faces of the hull that have a smaller area (assuming the rest area for each face
            // is equal)
            //
            // TODO refine the pressure algorithm so that the volume doesn't compress as much
            //
            float pressureForceMult = 1.0f - faceArea / surfaceArea;
            pressureForceMult *= pressureForceMult;
            for (int j = 0; j < numPtsPerFace; ++j)
            {
                Vector3 pressureForce =
                    faceNormal * (PressureMultiplier * pressureForceMult * (1.0f - volumeRatio));
                if (HasNaN(pressureForce))
                {
                    // This bug was due to division by float.PositiveInfinity in the faceNormal
                    // calculation, after the positions became too far apart due to too low
                    // damping value and too high velocities. It seems to be fixed by limiting
                    // the minimum damping value.
                    //Debug.LogWarningFormat("{0}>{1} calculated NaN for pressure force",
                    //    transform.parent == null ? "(no parent)" : transform.parent.gameObject.name,
                    //    gameObject.name);                     
                }
                else
                {
                    PointMassAccelerations[FacePointMassIndexes[i, j]] += pressureForce;
                }
            }
        }
    }

    private void CorrectVerticalPos(Vector3[] pos, int idx1, int idx2)
    {
        pos[idx1].x = pos[idx2].x;
        pos[idx1].z = pos[idx2].z;
    }

    private void ApplyCoef()
    {
        float topHeight = PointMassPositions[4].y + initialHeight * compressionCoef;
        Vector3 depenetrationDir = new Vector3(0, -1, 0);
        if (PointMassPositions[0].y > topHeight)
        {
            for (int i = 0; i < 4; i++)
            {
                PointMassPositions[i].y = topHeight;
                
                float speedAlongNormalSigned = Vector3.Dot(PointMassVelocities[i], depenetrationDir);
                float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
                Vector3 velocityAlongNormal = speedAlongNormalSigned * depenetrationDir;
                Vector3 slideVelocity = PointMassVelocities[i] - velocityAlongNormal;
                velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing

                float bounceCoefficient = (speedAlongNormalSign >= 0.0f ? 1.0f : BounceCoefficient);
                PointMassVelocities[i] = bounceCoefficient * velocityAlongNormal + slideVelocity * SlideCoefficient;
            }
        }
    }
    
    private void SolveForVelocitiesAndPositions(float deltaTime)
    {
        if (dynamic)
        {
            // solve for velocities and positions of point masses, and recalculate the bounding box
            Bounds ptBounds = new Bounds();
            for (int i = 0; i < PointMassPositions.Length; ++i)
            {
      
                PointMassVelocities[i] += PointMassAccelerations[i] * deltaTime;
                PointMassPositions[i] += PointMassVelocities[i] * deltaTime;
                
                if (i == 0)
                {
                    ptBounds = new Bounds(PointMassPositions[i], new Vector3(0, 0, 0));
                }
                else
                {
                    ptBounds.Encapsulate(PointMassPositions[i]);
                }
            }

            PointMassPositions[0].y = PointMassPositions[1].y = PointMassPositions[2].y = PointMassPositions[3].y;
            CorrectVerticalPos(PointMassPositions, 0, 4);
            CorrectVerticalPos(PointMassPositions, 1, 5);
            CorrectVerticalPos(PointMassPositions, 2, 6);
            CorrectVerticalPos(PointMassPositions, 3, 7);
            
            // The position and scale are set to fit the TriggerCollider (BoxCollider) to be
            // a conservative bounds for the point masses. Also, the child mesh will inherit this 
            // transform, and go squish, etc.
            if (HasNaN(ptBounds.center) || HasNaN(ptBounds.size))
            {
                Debug.LogWarningFormat("{0}>{1} simulation destabilized, NaN detected, and will be respawned",
                                        transform.parent == null ? "(no parent)" : transform.parent.gameObject.name,
                                        gameObject.name);

                Respawn();
            }
            else
            {
                Vector3 currentEdgeX = PointMassPositions[0] - PointMassPositions[3];
                Vector3 currentEdgeY = PointMassPositions[0] - PointMassPositions[4];
                Vector3 currentEdgeZ = PointMassPositions[0] - PointMassPositions[1];

                currentEdgeX /= currentEdgeX.magnitude;
                currentEdgeY /= currentEdgeY.magnitude;
                currentEdgeZ /= currentEdgeZ.magnitude;
                
                Vector3 standardEdgeX = new Vector3(1, 0, 0);
                Vector3 standardEdgeY = new Vector3(0, 1, 0);
                Vector3 standardEdgeZ = new Vector3(0, 0, 1);
                
                
                //transform.rotation = totalRotation;
                
                transform.localScale = new Vector3(
                    (PointMassPositions[0] - PointMassPositions[3]).magnitude / MappingCoef.x,
                    (PointMassPositions[0] - PointMassPositions[4]).magnitude / MappingCoef.y,
                    (PointMassPositions[0] - PointMassPositions[1]).magnitude / MappingCoef.z);
                

                Vector3 center = PointMassPositions[0] - currentEdgeX / 2 - currentEdgeY / 2 - currentEdgeZ / 2;
                transform.position = ptBounds.center + new Vector3(initialOffset.x * transform.localScale.x,
                    initialOffset.y * transform.localScale.y, initialOffset.z * transform.localScale.z);
            }
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                PointMassPositions[i] = transform.TransformPoint(RestOffset[i] + childMeshBounds.center);
                transform.localScale = new Vector3(1, 1, 1);
            }
        }
        
    }

    
    private void MapVectorMul(Vector3 vec3, Vector3 mappingCoef)
    {
        vec3 = new Vector3(vec3.x * mappingCoef.x, vec3.y * mappingCoef.y, vec3.z * mappingCoef.z);
    }

    private void MapVectorDiv(Vector3 vec3, Vector3 mappingCoef)
    {
        vec3 = new Vector3(vec3.x / mappingCoef.x, vec3.y / mappingCoef.y, vec3.z / mappingCoef.z);
    }
    // Returns the cross product: (a - b) X (c - b)
    static Vector3 CalcCross(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 bToA = a - b;
        Vector3 btoC = c - b;
        return Vector3.Cross(bToA, btoC);
    }

    // Got NaN? Then there's an error in the math.
    static bool HasNaN(Vector3 v)
    {
        return float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z);
    }

    // Calculates the force from the spring to apply to point mass 0, 
    // and the inverse for point mass 1
    //
    // Adapted from http://joeyfladderak.com/lets-talk-physics-soft-body-dynamics/
    //
    Vector3 CalcSpringForce(int pt0Index, int pt1Index, float restLength, float stiffness, 
                            float damping)
    {
        Vector3 pt0ToPt1 = PointMassPositions[pt1Index] - PointMassPositions[pt0Index];
        float springLength = pt0ToPt1.magnitude;
        if (springLength == 0.0f)
        {
            // if fully compressed, avoid a zero force in the final multiplication
            pt0ToPt1 = Vector3.up;
        }
        else
        {
            // normalize
            pt0ToPt1 /= springLength;
        }
        float stretch = springLength - restLength;
        Vector3 relativePtVelocity = PointMassVelocities[pt1Index] - PointMassVelocities[pt0Index];

        // spring force magnitude =
        //      delta from rest length * stiffness coefficient + 
        //      relative pt velocity along spring * damping coefficient
        float springForceMagnitude =
            (stretch * stiffness) + Vector3.Dot(relativePtVelocity, pt0ToPt1) * damping;

        return springForceMagnitude * pt0ToPt1;
    }

    private void OnDrawGizmos()
    {
        if (PointMassPositions == null || PointMassPositions.Length < 4 || !debugModel)
            return;

        // draw the top point masses
        Gizmos.color = Color.red;
        for (int i = 0; i < 4; ++i)
        {
            Gizmos.DrawSphere(PointMassPositions[i], .1f);
        }

        // draw the bottom point masses
        Gizmos.color = Color.blue;
        for (int i = 4; i < PointMassPositions.Length; ++i)
        {
            Gizmos.DrawSphere(PointMassPositions[i], .1f);
        }

        // draw the springs used to form the faces of the point mass hull and draw
        // their normals
        for (int i = 0; i < FacePointMassIndexes.GetLength(0); ++i)
        {
            Vector3 a = PointMassPositions[FacePointMassIndexes[i, 0]];
            Vector3 b = PointMassPositions[FacePointMassIndexes[i, 1]];
            Vector3 c = PointMassPositions[FacePointMassIndexes[i, 2]];
            Vector3 d = PointMassPositions[FacePointMassIndexes[i, 3]];

            // face edges
            Gizmos.color = Color.white;
            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, d);
            Gizmos.DrawLine(d, a);
            
            // face normals
            Gizmos.color = Color.blue;        
            Vector3 faceNormal = CalcCross(a, b, c).normalized;
            Vector3 faceMidpoint = Vector3.Lerp(a, c, .5f);
            Gizmos.DrawLine(faceMidpoint, faceMidpoint + faceNormal);
        }

        // draw the springs that cross inside the hull and hull faces
        Gizmos.color = Color.yellow;
        for (int i = 0; i < DiagonalSpringPointMassIndexes.GetLength(0); ++i)
        {
            int pt0Index = DiagonalSpringPointMassIndexes[i, 0];
            int pt1Index = DiagonalSpringPointMassIndexes[i, 1];
            Gizmos.DrawLine(PointMassPositions[pt0Index], PointMassPositions[pt1Index]);
        }
    }
}
