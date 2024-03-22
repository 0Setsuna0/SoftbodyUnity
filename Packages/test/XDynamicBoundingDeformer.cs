using System;
using System.Collections;
using System.Collections.Generic;
using Sirenix.OdinInspector;
using UnityEngine;
namespace XRender.Scripting
{
    public class XDynamicBoundingDeformer : MonoBehaviour
    {
        public enum ImpactType
        {
            Top,
            Bottom,
            Left,
            Right,
            Front,
            Back,
            None
        }
        
        #region SimParam
        [ShowInInspector]
        [Range(0.0f, 100.0f)] 
        private float _pressureCoef = 50.0f;
        
        [ShowInInspector]
        [Range(20.0f, 100.0f)]
        private float _springStiffness = 50.0f;
        
        [ShowInInspector]
        [Range(1.5f, 10.0f)]
        private float _springDamping = 1.5f;

        [ShowInInspector]
        [Range(0.0f, 1.0f)]
        private float _bounceCoefficient = .05f;

        [ShowInInspector]
        [Range(0.0f, 1.0f)]
        private float _slideCoefficient = .99f;

        [ShowInInspector]
        [Range(0.02f, 0.8f)] 
        private float _compressThreshold = 0.02f;

        [ShowInInspector]
        [Range(0.02f, 2.2f)]
        private float _compressionCoef = 1.0f;
        
        [ShowInInspector]
        [Range(0.5f, 4.0f)]
        private float _compressSpeed = 1.0f;

        [ShowInInspector]
        [Range(0.0f, 2.0f)] 
        private float _compressKeepingTime = 0.5f;

        [ShowInInspector] 
        [Range(0.1f, 4.0f)]
        private float _postCompressWaitingTime = 1.75f;

        
        private Vector3 _gravity = new Vector3(0, -9.8f, 0);
        
        private float appliedForce = 200;
        
        public bool dynamic = false;
        public bool needCompressing = false;
        public bool fixBottom = false;
        private bool _frontHitTest = false;
        private bool _beingCompressed = false;
        private bool _afterCompressing = false;
        #endregion

        #region Data
        private Vector3[] _accelerations;
        private Vector3[] _velocities;
        private Vector3[] _positions;
        private Vector3[] _localPositions;
        private Vector3[] _restOffset;
        private Vector3[] _faceNormals;
        private Vector3 _transformStartPosition;
        private Vector3 _transformStartScale;
        private Vector3[] _currentShapeBound;
        
        private float[] _faceAreas;
        private float[] _springRestLengths;
        
        private int[,] _facePointMassIndexes;
        private int[,] _diagonalSpringPointMassIndexes;
        private Surface[] _surfaces;
        private Surface _tempSurface;
        
        private float _hullRestVolume;

        private Bounds _colliderBounds;
        
        #endregion

        #region Component related
        public Transform childTransform;
        
        private BoxCollider _triggerCollider;
        private SkinnedMeshRenderer _childSkinnedMeshRenderer;
        private MeshFilter _meshFilter;
        private Mesh _childMesh;
        private Bounds _childMeshBounds;
        private Vector3 _mappingCoef;
        private Vector3 _initialOffset;
        private Vector3 _debugCenter;
        private Vector3 _debugContactPos;
        private float _initialHeight;
        private Vector3 _initialScale;
        private Vector3 _initialEuler;
        private Vector3 _initialCenter;
        
        public bool debugModel = true;
        private ImpactType _hitType = ImpactType.None;
        private ImpactType applyHitType = ImpactType.Front;
        #endregion
        
        private readonly int[] _topSurface = { 0, 1, 2, 3 };
        private readonly int[] _bottomSurface = { 4, 5, 6, 7 };
        private readonly int[] _leftSurface = { 2, 3, 6, 7 };
        private readonly int[] _rightSurface = { 0, 1, 4, 5 };
        private readonly int[] _frontSurface = { 0, 4, 3, 7 };
        private readonly int[] _backSurface = { 1, 5, 6, 2 };
        
        //Semi-Rigid Param
        private Rigidbody _rb;
        private XDynamicVirtualBox xvb;
        public Vector3 dynamicCenterOffset = new Vector3();
        public Vector3 dynamicBoundingScale = new Vector3();
        void Awake()
        {
            InitComponents();
            
            InitPositions();

            InitIndexes();

            InitPhysics();
            
            SynchronizePhysicsState();
        }
        //PT
        void FixedUpdate()
        {
            // if (dynamic)
            // {
            //     for (int i = 0; i < _positions.Length; i++)
            //     {
            //         _accelerations[i] = new Vector3(0,-9,0);
            //     }
            //     
            //     ApplyVerticalCompressionCoef();
            //     
            //     AccumulateSpringForce();
            //     
            //     AccumulatePressureForce();
            //     
            //     AdvectParticles(Time.fixedDeltaTime);
            // }
            xvb._springStiffness = _springStiffness;
            xvb._pressureCoef = _pressureCoef;
            
        }

        private IEnumerator RunLateFixedUpdate()
        {
            while (true)
            {
                yield return new WaitForFixedUpdate();
                if(Application.isPlaying)
                    LateFixedUpdate();
            }
        }

        private void OnEnable()
        {
            StartCoroutine(RunLateFixedUpdate());
        }

        private void OnDisable()
        {
            StopCoroutine(RunLateFixedUpdate());
        }

        private void LateFixedUpdate()
        {
            xvb.Step(ref dynamicCenterOffset, ref dynamicBoundingScale);
            
            transform.position += transform.rotation * dynamicCenterOffset;
            transform.localScale = new Vector3(_initialScale.x * dynamicBoundingScale.x,
                _initialScale.y * dynamicBoundingScale.y, _initialScale.z * dynamicBoundingScale.z);
        }

        //GT
        private void Update()
        {
            if(!dynamic)
            {
                SynchronizePhysicsState();
            }
        }

        private void AddToSolver()
        {
            
        }
        
        //----initialize----//
        private void InitComponents()
        {  
             _initialEuler = transform.rotation.eulerAngles;
             transform.rotation = Quaternion.Euler(0,0,0);
            
             _triggerCollider = GetComponent<BoxCollider>();
             _childSkinnedMeshRenderer = GetComponentInChildren<SkinnedMeshRenderer>();
             _meshFilter = GetComponentInChildren<MeshFilter>();
             _rb = GetComponent<Rigidbody>();

             if (_childSkinnedMeshRenderer != null)
             {
                 _childMesh = _childSkinnedMeshRenderer.sharedMesh;
                 _childMeshBounds = _childMesh.bounds;
             }
             else
             {
                 _childMesh = _meshFilter.mesh;
                 _childMeshBounds = _childMesh.bounds;
             }

             //get local child bounds center
             Vector3 localBoundsCenter = _childMeshBounds.center;
             //translate to world location
             Vector3 worldBoundsCenter = childTransform.TransformPoint(localBoundsCenter);
             //get loacl father bounds center
             _triggerCollider.center = transform.InverseTransformPoint(worldBoundsCenter);

             var localScale = childTransform.localScale;
             Vector3 scaledBoundsSize = new Vector3(
                 _childMeshBounds.size.x * localScale.x,
                 _childMeshBounds.size.y * localScale.y,
                 _childMeshBounds.size.z * localScale.z
             );
             _triggerCollider.size = scaledBoundsSize;

             _positions = new Vector3[8];
             _velocities = new Vector3[8];
             _accelerations = new Vector3[8];
             _restOffset = new Vector3[8];
             _currentShapeBound = new Vector3[6];
             _initialOffset = (transform.position - transform.TransformPoint(_triggerCollider.center));
             _surfaces = new Surface[6];
             _localPositions = new Vector3[8];
             _initialScale = new Vector3();

             _initialScale = transform.localScale; 
        }

        private void InitPositions()
        {
            Vector3[] corners = new Vector3[8];
            
            corners[0] = _triggerCollider.center + new Vector3(_triggerCollider.size.x, _triggerCollider.size.y, _triggerCollider.size.z) / (2);
            corners[1] = _triggerCollider.center + new Vector3(_triggerCollider.size.x, _triggerCollider.size.y, -_triggerCollider.size.z) / (2);
            corners[2] = _triggerCollider.center + new Vector3(-_triggerCollider.size.x, _triggerCollider.size.y, -_triggerCollider.size.z) / (2);
            corners[3] = _triggerCollider.center + new Vector3(-_triggerCollider.size.x, _triggerCollider.size.y, _triggerCollider.size.z) / (2);
            corners[4] = _triggerCollider.center + new Vector3(_triggerCollider.size.x, -_triggerCollider.size.y, _triggerCollider.size.z) / (2);
            corners[5] = _triggerCollider.center + new Vector3(_triggerCollider.size.x, -_triggerCollider.size.y, -_triggerCollider.size.z) / (2);
            corners[6] = _triggerCollider.center + new Vector3(-_triggerCollider.size.x, -_triggerCollider.size.y, -_triggerCollider.size.z) / (2);
            corners[7] = _triggerCollider.center + new Vector3(-_triggerCollider.size.x, -_triggerCollider.size.y, _triggerCollider.size.z) / (2);
            
            for (int i = 0; i < 8; i++)
            {
                _restOffset[i] = corners[i] - _triggerCollider.center;
                //corners[i] =  transform.TransformPoint(corners[i]);
            }
            
            _positions = corners;
            xvb = new XDynamicVirtualBox(_positions, _triggerCollider.center);
            _mappingCoef = new Vector3(
                (_positions[0] - _positions[3]).magnitude ,
                (_positions[0] - _positions[4]).magnitude ,
                (_positions[0] - _positions[1]).magnitude) ;
        }

        private int SurfaceTypeToIdx(SurfaceType surfaceType)
        {
            switch (surfaceType)
            {
                case SurfaceType.Top:
                    return 0;
                case SurfaceType.Bottom:
                    return 5;
                case SurfaceType.Front:
                    return 4;
                case SurfaceType.Back:
                    return 2;
                case SurfaceType.Left:
                    return 3;
                case SurfaceType.Right:
                    return 1;
                default:
                    return 0;
            }
        }
        private void InitIndexes()
        {
            _facePointMassIndexes = new int[6, 4]
            {
                { 3, 2, 1, 0 }, { 1, 5, 4, 0 }, { 2, 6, 5, 1 }, { 3, 7, 6, 2 },
                { 0, 4, 7, 3 }, { 4, 5, 6, 7 }
            };
            
            
            _diagonalSpringPointMassIndexes = new int[16, 2]
            {
                { 0, 6 }, { 1, 7 }, { 2, 4 }, { 3, 5 }, 
             
                { 3, 1 }, { 2, 0 },
                { 0, 5 }, { 4, 1 },
                { 1, 6 }, { 5, 2 },
                { 2, 7 }, { 6, 3 },
                { 3, 4 }, { 7, 0 },
                { 4, 6 }, { 5, 7 },
            };
        }

        private void InitPhysics()
        {
            int springIndex = 0;
            int numFaceEdgeSprings = _facePointMassIndexes.GetLength(0) * _facePointMassIndexes.GetLength(1);
            int numDiagonalSprings = _diagonalSpringPointMassIndexes.GetLength(0);
            _springRestLengths = new float[numFaceEdgeSprings + numDiagonalSprings];

            //save face edge springs' initial length
            int numFaces = _facePointMassIndexes.GetLength(0);
            int numParticlePerFace = _facePointMassIndexes.GetLength(1);
            for (int i = 0; i < numFaces; i++)
            {
                for (int j = 0; j < numParticlePerFace; j++)
                {
                    int pIndex0 = _facePointMassIndexes[i, j];
                    int pIndex1 = _facePointMassIndexes[i, (j + 1) % numParticlePerFace];
                    _springRestLengths[springIndex] = (_positions[pIndex0] - _positions[pIndex1]).magnitude;
                    springIndex++;
                }
            }
            //save diagonal edge springs' initial length
            for (int i = 0; i < numDiagonalSprings; ++i)
            {
                int pt0Index = _diagonalSpringPointMassIndexes[i, 0];
                int pt1Index = _diagonalSpringPointMassIndexes[i, 1];
                _springRestLengths[springIndex] =
                    (_positions[pt0Index] - _positions[pt1Index]).magnitude;
                springIndex++;
            }
            
            _faceNormals = new Vector3[numFaces];
            _faceAreas = new float[numFaces];

            var size = _triggerCollider.size;
            var transform1 = transform;
            var localScale = transform1.localScale;
            

            _hullRestVolume =  (transform1.localScale.x * transform1.localScale.y * transform1.localScale.z);

            _initialHeight = _positions[0].y - _positions[4].y;
            _debugContactPos = new Vector3(0, 0, 0);
            transform.rotation = Quaternion.Euler(_initialEuler);
            
            
        }
        
        //----physics solver----//

        #region Force Solver
        private Vector3 ComputeSpringForce(int idx0, int idx1, float restLength, float stiffness, float airDamping)
        {
            Vector3 pos0 = _positions[idx0];
            Vector3 pos1 = _positions[idx1];

            //from particle 0 to particle 1
            Vector3 dif = pos1 - pos0;
            float d = dif.magnitude;
            if (d == 0.0f)
            {
                //for stability
                dif = new Vector3(0, 1, 0);
            }
            else
            {
                //normalize
                dif /= d;
            }

            float c = d - restLength;
            //air damping
            Vector3 relativeVel = _velocities[idx1] - _velocities[idx0];

            float forceSize = (c * stiffness) + Vector3.Dot(relativeVel, dif) * airDamping;

            return forceSize * dif;
        }
        private void AccumulateSpringForce()
        {
            int springIndex = 0;
            int numFaces = _facePointMassIndexes.GetLength(0);
            int numParticlePerFace = _facePointMassIndexes.GetLength(1);
            for (int i = 0; i < numFaces; i++)
            {
                for (int j = 0; j < numParticlePerFace; j++)
                {
                    int pIndex0 = _facePointMassIndexes[i, j];
                    int pIndex1 = _facePointMassIndexes[i, (j + 1) % numParticlePerFace];

                    Vector3 force = ComputeSpringForce(pIndex0, pIndex1, _springRestLengths[springIndex], _springStiffness,
                        _springDamping);

                    _accelerations[pIndex0] += force;
                    _accelerations[pIndex1] -= force;

                    springIndex++;
                }
            }
            
            int numDiagonalSprings = _diagonalSpringPointMassIndexes.GetLength(0);
            for (int i = 0; i < numDiagonalSprings; i++)
            {
                int pIndex0 = _diagonalSpringPointMassIndexes[i, 0];
                int pIndex1 = _diagonalSpringPointMassIndexes[i, 1];

                Vector3 force = ComputeSpringForce(pIndex0, pIndex1, _springRestLengths[springIndex], _springStiffness,
                    _springDamping);

                _accelerations[pIndex0] += force;
                _accelerations[pIndex1] -= force;

                springIndex++;
            }
        }

        private void AccumulatePressureForce()
        {
            float currentVolume = transform.localScale.x * transform.localScale.y * transform.localScale.z;

            float volumeRatio = currentVolume / _hullRestVolume;
            float surfaceArea = 0.0f;
            int numFaces = _facePointMassIndexes.GetLength(0);
            int numPtsPerFace = _facePointMassIndexes.GetLength(1);

            //caculate face normal and area
            for (int i = 0; i < numFaces; i++)
            {
                Vector3 pos0 = _positions[_facePointMassIndexes[i, 0]];
                Vector3 pos1 = _positions[_facePointMassIndexes[i, 1]];
                Vector3 pos2 = _positions[_facePointMassIndexes[i, 2]];

                Vector3 faceNormal = Vector3.Cross(pos0 - pos1, pos2 - pos1);
                float faceArea = faceNormal.magnitude;
                if (float.IsInfinity(faceArea))
                {
                    faceNormal = new Vector3(0, 1.0f, 0);
                    faceArea = _transformStartScale.x * _transformStartScale.y;
                }
                else if(faceArea > 0.0f)
                {
                    faceNormal /= faceArea;
                }

                _faceNormals[i] = faceNormal;
                _faceAreas[i] = faceArea;

                surfaceArea += faceArea;
            }
            
            for (int i = 0; i < numFaces; ++i)
            {
                Vector3 faceNormal = _faceNormals[i];
                float faceArea = _faceAreas[i];
                
                float pressureForceMult = 1.0f - faceArea / surfaceArea;
                pressureForceMult *= pressureForceMult;
                for (int j = 0; j < numPtsPerFace; ++j)
                {
                    Vector3 pressureForce =
                        faceNormal * (_pressureCoef * pressureForceMult * (1.0f - volumeRatio));
                    if (IsInvalidVector(pressureForce))
                    {
                        Debug.Log("invalid pressure force");
                    }
                    else
                    {
                        _accelerations[_facePointMassIndexes[i, j]] += pressureForce;
                    }
                }
            }
        }

        private void AdvectParticles(float dt)
        {
            if(_afterCompressing)
                return;
            Vector3 boundsCenter = new Vector3();
            for (int i = 0; i < _positions.Length; ++i)
            {
                _velocities[i] += _accelerations[i] * dt;
                if (fixBottom && i >= 4)
                    _velocities[i].y = 0.0f;
                _positions[i] += _velocities[i] * dt;

                boundsCenter += _positions[i];
            }

            boundsCenter /= _positions.Length;
            
            KeepRelativePosition();

            var transform1 = transform;
            Vector3 localScale;
            localScale = new Vector3(
                (_positions[0] - _positions[3]).magnitude * _initialScale.x / _mappingCoef.x,
                (_positions[0] - _positions[4]).magnitude * _initialScale.y / _mappingCoef.y,
                (_positions[0] - _positions[1]).magnitude * _initialScale.z / _mappingCoef.z);
            transform1.localScale = localScale;

            _debugCenter = boundsCenter;
            
            Vector3 initialOffset = new Vector3(_initialOffset.x * localScale.x / _initialScale.x,
                _initialOffset.y * localScale.y / _initialScale.y, _initialOffset.z * localScale.z / _initialScale.z);
            Vector3 rotatedOffset = transform1.rotation * initialOffset;
            
            // transform1.position = boundsCenter + new Vector3(rotatedOffset.x,
            //     rotatedOffset.y, rotatedOffset.z);
        }

        #endregion
        
        private void KeepRelativePosition()
        {
            for (int i = 0; i < _positions.Length; i++)
            {
                _localPositions[i] = transform.InverseTransformPoint(_positions[i]);
            }

            _localPositions[0].y = _localPositions[1].y = _localPositions[2].y = _localPositions[3].y;
            CorrectVerticalPos(_localPositions, 0, 4);
            CorrectVerticalPos(_localPositions, 1, 5);
            CorrectVerticalPos(_localPositions, 2, 6);
            CorrectVerticalPos(_localPositions, 3, 7);
            
            for (int i = 0; i < _positions.Length; i++)
            {
                _positions[i] = transform.TransformPoint(_localPositions[i]);
            }
        }
        
        private void CorrectVerticalPos(Vector3[] pos, int idx1, int idx2)
        {
            pos[idx1].x = pos[idx2].x;
            pos[idx1].z = pos[idx2].z;
            if (pos[idx1].y < pos[idx2].y)
                pos[idx1].y = pos[idx2].y + 0.02f;
        }
        
        
        //----externel interaction controller----//
        private void SynchronizePhysicsState()
        {
            for (int i = 0; i < 8; i++)
            {
                _positions[i] = transform.TransformPoint(_restOffset[i] + _triggerCollider.center);
            }

            for (int i = 0; i < 6; i++)
            {
                CalculateSurfaceCenter(i, ref _currentShapeBound[i]);
            }
            
            _surfaces[0] = new Surface(_currentShapeBound[0], transform.rotation * (new Vector3(0, -1, 0)));
            _surfaces[1] = new Surface(_currentShapeBound[1], transform.rotation * (new Vector3(-1, 0, 0)));
            _surfaces[2] = new Surface(_currentShapeBound[2], transform.rotation * (new Vector3(0, 0, 1)));
            _surfaces[3] = new Surface(_currentShapeBound[3], transform.rotation * (new Vector3(1, 0, 0)));
            _surfaces[4] = new Surface(_currentShapeBound[4], transform.rotation * (new Vector3(0, 0, -1)));
            _surfaces[5] = new Surface(_currentShapeBound[5], transform.rotation * (new Vector3(0, 1, 0)));
            
            Array.Fill(_velocities, new Vector3(0,0,0));
            Array.Fill(_accelerations, new Vector3(0, 0, 0));
        }
        
        private void ApplyVerticalCompressionCoef()
        {
            Vector3 topPoint = _positions[4] + _surfaces[5].Normal * (_initialHeight * _compressionCoef);
            _surfaces[0].Point = topPoint;
            if (_surfaces[0].IsParticleInVirtualSurface(_positions[0]))
            {
                HandleContact(ImpactType.Top, _surfaces[0], false);
            }
        }

        //对指定的面施加瞬态冲量
        private void ApplyImpact(ImpactType impactType, float force)
        {
            int[] strainedParticleIdx = new int[4];
            
            _hitType = impactType;
            Surface tempSurface = new Surface();
            GetImpactPhysicsInfo(impactType, ref strainedParticleIdx, ref tempSurface);
            Vector3 depenetrationDir = tempSurface.Normal.normalized;
            Debug.Log(depenetrationDir * force);
            for (int i = 0; i < 4; i++)
            {
                int pIndex = strainedParticleIdx[i];
                _accelerations[pIndex] += depenetrationDir * force;
            }
        }
        
        //对指定的面施加持续性虚拟平面挤压
        private void HandleContact(ImpactType impactType, Surface surface, bool lockOpposite)
        {
            Vector3 depenetrationDir = surface.Normal;
            int[] strainedParticleIdx = new int[4];
            
            Surface tempSurface = new Surface();
            GetImpactPhysicsInfo(impactType, ref strainedParticleIdx, ref tempSurface);
            
            
            for (int i = 0; i < 4; i++)
            {
                int pIndex = strainedParticleIdx[i];
                HandleVirtualSurfaceContact(ref _positions[pIndex], surface);

                float normalSpeed = Vector3.Dot(_velocities[pIndex], depenetrationDir);
                float normalSpeedSign = Mathf.Sign(normalSpeed);
                Vector3 normalVel = normalSpeed * depenetrationDir;
                Vector3 tangentialVel = _velocities[pIndex] - normalVel;
                
                //reflect normal velocity, keep the tangential velocity
                normalVel *= normalSpeedSign;

                float bounceCoef = (normalSpeedSign >= 0.0f ? 1.0f : _bounceCoefficient);
                _velocities[pIndex] = bounceCoef * normalVel + tangentialVel * _slideCoefficient;
            }
        }
        
        //响应瞬态冲量，锁定对立面
        private void ConstraintSurface()
        {
            if(applyHitType == ImpactType.None)
                return;
            bool trigger = false;
            int[] oppositeParticleIdx = new int[4];
            Vector3 depenetrationDir = new Vector3();
            Surface surface = new Surface();
            switch (_hitType)
            {
                case ImpactType.Top:
                    oppositeParticleIdx = _bottomSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Bottom)];
                    trigger = surface.IsParticleInVirtualSurface(_positions[4]);
                    break;
                case ImpactType.Front:
                    oppositeParticleIdx = _backSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Back)];
                    trigger = surface.IsParticleInVirtualSurface(_positions[1]);
                    break;
                case ImpactType.Back:
                    oppositeParticleIdx = _frontSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Front)];
                    trigger = surface.IsParticleInVirtualSurface(_positions[0]);
                    break;
                case ImpactType.Left:
                    oppositeParticleIdx = _rightSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Right)];
                    trigger = surface.IsParticleInVirtualSurface(_positions[0]);
                    break;
                case ImpactType.Right:
                    oppositeParticleIdx = _leftSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Left)];
                    trigger = surface.IsParticleInVirtualSurface(_positions[2]);
                    break;
            }
            
            if(!trigger)
                return;
            
            for (int i = 0; i < 4; i++)
            {
                int pIndex = oppositeParticleIdx[i];
                HandleVirtualSurfaceContact(ref _positions[pIndex], surface);

                float normalSpeed = Vector3.Dot(_velocities[pIndex], depenetrationDir);
                float normalSpeedSign = Mathf.Sign(normalSpeed);
                Vector3 normalVel = normalSpeed * depenetrationDir;
                Vector3 tangentialVel = _velocities[pIndex] - normalVel;
                
                //reflect normal velocity, keep the tangential velocity
                normalVel *= normalSpeedSign;

                float bounceCoef = (normalSpeedSign >= 0.0f ? 1.0f : _bounceCoefficient);
                _velocities[pIndex] = bounceCoef * normalVel + tangentialVel * _slideCoefficient;
            }
            
        }
        
        //根据受击面，得到惩罚方向，响应点索引等信息
        private void GetImpactPhysicsInfo(ImpactType impactType, ref int[] strainedParticleIdx, ref Surface surface)
        {
            switch (impactType)
            {
                case ImpactType.Top:
                    strainedParticleIdx = _topSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Top)];
                    break;
                case ImpactType.Bottom:
                    strainedParticleIdx = _bottomSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Bottom)];
                    break;
                case ImpactType.Back:
                    strainedParticleIdx = _backSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Back)];
                    break;
                case ImpactType.Front:
                    strainedParticleIdx = _frontSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Front)];
                    break;
                case ImpactType.Left:
                    strainedParticleIdx = _leftSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Left)];
                    break;
                case ImpactType.Right:
                    strainedParticleIdx = _rightSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Right)];
                    break;
                default:
                    strainedParticleIdx = _topSurface;
                    surface = _surfaces[SurfaceTypeToIdx(SurfaceType.Top)];
                    break;
            }
        }
        private IEnumerator ChangeCoefOverTime()
        {
            _beingCompressed = true;
            _afterCompressing = false;
            
            float t = 0.0f;
            float coef = 0.0f;
            while (t < 1)
            {
                _compressionCoef = Mathf.LerpUnclamped(1.0f, _compressThreshold, coef);
                t += Time.fixedDeltaTime * _compressSpeed;
                yield return null;
            }

            yield return new WaitForSeconds(_compressKeepingTime);
            
            _compressionCoef = 2.0f;
            yield return new WaitForSeconds(_postCompressWaitingTime);
            _afterCompressing = true;
            t = 0.0f;
            float initialScaleX = transform.localScale.x;
            float initialScaleY = transform.localScale.y;
            float initialScaleZ = transform.localScale.z;
            float[] lerpSclae = new float[3];
            //return to scale(1,1,1)
            while (t < 1)
            {
                t += Time.fixedDeltaTime * 4;
                if (t > 1)
                    t = 1;
                lerpSclae[0] = Mathf.Lerp(initialScaleX, _initialScale.x, t);
                lerpSclae[1] = Mathf.Lerp(initialScaleY, _initialScale.y, t);
                lerpSclae[2] = Mathf.Lerp(initialScaleZ, _initialScale.z, t);
                transform.localScale = new Vector3(lerpSclae[0], lerpSclae[1], lerpSclae[2]);
                yield return null;
            }
            _beingCompressed = false;
            needCompressing = false; 
            dynamic = false;
            _rb.useGravity = true;
        }

        
        //----collision detection----//
        

        private ImpactType GetImpactSurface(Vector3 contactPos)
        {
            float d = 100;
            int sid = 0;
            
            for (int i = 0; i < _surfaces.Length; i++)
            {
                Vector3 dif = (contactPos - _surfaces[i].Point).normalized;
                float dt = Mathf.Abs(Vector3.Dot(dif, _surfaces[i].Normal.normalized));
                if (dt < d)
                {
                    sid = i;
                    d = dt;
                }
            }

            switch (sid)
            {
                case 0:
                    return ImpactType.Right;
                case 1:
                    return ImpactType.Right;
                case 2:
                    return ImpactType.Back;
                case 3:
                    return ImpactType.Left;
                case 4:
                    return ImpactType.Front;
                case 5:
                    return ImpactType.Bottom;
                default:
                    return ImpactType.Right;
            }
        }
        private void OnTriggerEnter(Collider other)
        {
            
        }

        private void OnCollisionEnter(Collision collision)
        {
            Debug.Log("Collision Detected");
            var contactPoints = collision.contacts;
            for (int i = 0; i < contactPoints.Length; i++)
            {
                Debug.Log(contactPoints[i].point);
            }
            if(Mathf.Abs(_rb.velocity.magnitude) > 5)
                xvb.RegistImpactEvent(new XDynamicBoundingImpact(_bottomSurface, 100,
                new Vector3(0, 1, 0)));
            
        }

        // private void OnCollisionStay(Collision collision)
        // {
        //     Vector3 depenetrationDir;
        //     float depenetrationDist;
        //     //Debug.Log(transform.rotation);
        //     var other = collision.collider;
        //     if (
        //         Physics.ComputePenetration(_triggerCollider, transform.position, transform.rotation,
        //                                    other, other.transform.position, other.transform.rotation,
        //                                    out depenetrationDir, out depenetrationDist))
        //     {
        //         _colliderBounds = other.bounds;
        //         //Debug.LogFormat("Collision normal {0}", depenetrationDir);
        //         for (int i = 0; i < _positions.Length; ++i)
        //         {
        //             Vector3 p = _positions[i];
        //             if (other.bounds.Contains(p))
        //             {
        //                 Debug.Log(i);
        //                 // clamp interpenetrating point mass to surface of other collider
        //                 _positions[i] = 
        //                     other.ClosestPoint(p + depenetrationDir * (depenetrationDist + 1.0f));
        //     
        //                 // reflect component of velocity along other collider normal
        //                 // while maintaining the remainder of velocity, but reduce by
        //                 // energy loss coefficient
        //                 // (approximate average contact normals as depenetration direction)
        //                 float speedAlongNormalSigned = Vector3.Dot(_velocities[i], depenetrationDir);
        //                 float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
        //                 Vector3 velocityAlongNormal = speedAlongNormalSigned * depenetrationDir;
        //                 Vector3 slideVelocity = _velocities[i] - velocityAlongNormal;
        //                 velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing
        //     
        //                 // reduce velocityAlongNormal by bounce coefficient if reflecting
        //                 float bounceCoefficient = (speedAlongNormalSign >= 0.0f ? 1.0f : _bounceCoefficient);
        //                 _velocities[i] = 
        //                     bounceCoefficient * velocityAlongNormal + 
        //                     slideVelocity * _slideCoefficient;                        
        //             }
        //         }
        //     }
        // 

        private void OnCollisionStay(Collision other)
        {

        }

        private void OnCollisionExit(Collision other)
        {
            Debug.Log("Exit collision");
        }
        private void OnTriggerStay(Collider other)
        {
            // overlapping with another collider, so make sure the point masses don't interpenetrate,
            // and resolve their velocities for the collision
            Vector3 depenetrationDir;
            float depenetrationDist;
            Debug.Log(transform.rotation);
            
            if (
                Physics.ComputePenetration(_triggerCollider, transform.position, transform.rotation,
                                           other, other.transform.position, other.transform.rotation,
                                           out depenetrationDir, out depenetrationDist))
            {
                _colliderBounds = other.bounds;
                //Debug.LogFormat("Collision normal {0}", depenetrationDir);
                for (int i = 0; i < _positions.Length; ++i)
                {
                    Vector3 p = _positions[i];
                    if (other.bounds.Contains(p))
                    {
                        Debug.Log(i);
                        // clamp interpenetrating point mass to surface of other collider
                        _positions[i] = 
                            other.ClosestPoint(p + depenetrationDir * (depenetrationDist + 1.0f));
            
                        // reflect component of velocity along other collider normal
                        // while maintaining the remainder of velocity, but reduce by
                        // energy loss coefficient
                        // (approximate average contact normals as depenetration direction)
                        float speedAlongNormalSigned = Vector3.Dot(_velocities[i], depenetrationDir);
                        float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
                        Vector3 velocityAlongNormal = speedAlongNormalSigned * depenetrationDir;
                        Vector3 slideVelocity = _velocities[i] - velocityAlongNormal;
                        velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing
            
                        // reduce velocityAlongNormal by bounce coefficient if reflecting
                        float bounceCoefficient = (speedAlongNormalSign >= 0.0f ? 1.0f : _bounceCoefficient);
                        _velocities[i] = 
                            bounceCoefficient * velocityAlongNormal + 
                            slideVelocity * _slideCoefficient;                        
                    }
                }
            }
        }

        private void OnTriggerExit(Collider other)
        {
        }
        
        
        //----helper function and debug function----//
        private bool IsInvalidVector(Vector3 vec)
        {
            return float.IsNaN(vec.x) || float.IsNaN(vec.y) || float.IsNaN(vec.z);
        }

        private void CalculateSurfaceCenter(int faceIndex, ref Vector3 surfaceCenter)
        {
            surfaceCenter = new Vector3(0, 0, 0);
            for (int i = 0; i < 4; i++)
            {
                surfaceCenter += _positions[_facePointMassIndexes[faceIndex,i]];
            }

            surfaceCenter /= 4;
        }

        private void HandleVirtualSurfaceContact(ref Vector3 pos, Surface surface)
        {
            Vector3 s1 = pos - surface.Point;
            float d = Vector3.Dot(s1, surface.Normal);
            pos += -d * surface.Normal;
        }
    #if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (_positions == null || _positions.Length < 4 || !debugModel)
                return;

            // draw the top point masses
            Gizmos.color = Color.red;
            for (int i = 0; i < 4; ++i)
            {
                Gizmos.DrawSphere(_positions[i], .1f);
            }

            // draw the bottom point masses
            Gizmos.color = Color.blue;
            for (int i = 4; i < _positions.Length; ++i)
            {
                Gizmos.DrawSphere(_positions[i], .1f);
            }
            
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(_debugCenter, .1f);

            Gizmos.color = Color.magenta;
            Gizmos.DrawSphere(_debugContactPos,.05f);
            // draw the springs used to form the faces of the point mass hull and draw
            // their normals
            for (int i = 0; i < _facePointMassIndexes.GetLength(0); ++i)
            {
                Vector3 a = _positions[_facePointMassIndexes[i, 0]];
                Vector3 b = _positions[_facePointMassIndexes[i, 1]];
                Vector3 c = _positions[_facePointMassIndexes[i, 2]];
                Vector3 d = _positions[_facePointMassIndexes[i, 3]];

                // face edges
                Gizmos.color = Color.white;
                Gizmos.DrawLine(a, b);
                Gizmos.DrawLine(b, c);
                Gizmos.DrawLine(c, d);
                Gizmos.DrawLine(d, a);
                
            }
            
            for (int i = 0; i < 8; i++)
            {
                
            }
            // draw the springs that cross inside the hull and hull faces
            Gizmos.color = Color.yellow;
            for (int i = 0; i < _diagonalSpringPointMassIndexes.GetLength(0); ++i)
            {
                int pt0Index = _diagonalSpringPointMassIndexes[i, 0];
                int pt1Index = _diagonalSpringPointMassIndexes[i, 1];
                Gizmos.DrawLine(_positions[pt0Index], _positions[pt1Index]);
            }
        }
        #endif
    }
}
