using System.Collections.Generic;
using System.Windows.Forms.VisualStyles;
using UnityEngine;

namespace XRender.Scripting
{
    public class XDynamicVirtualBox
    {
        #region Coef
        public float _pressureCoef = 50.0f;
        
        public float _springStiffness = 10.0f;
        
        private float _springDamping = 1.5f;
        
        private float _bounceCoefficient = .05f;

        private float _slideCoefficient = .99f;
        #endregion
        
        private Vector3 _gravity = new Vector3(0, -9.8f, 0);
        
        private float appliedForce = 200;
        
        private Vector3[] _restOffset;
        private Vector3[] _faceNormals;

        private Vector3[] _currentShapeBound;
        
        private float[] _faceAreas;
        private float[] _springRestLengths;
        
        private int[,] _facePointMassIndexes;
        private int[,] _diagonalSpringPointMassIndexes;
        private XDynamicSurface[] _surfaces;
        
        private List<XDynamicBoundingImpact> _deferredImpactList = new List<XDynamicBoundingImpact>();
        
        private float _hullRestVolume;

        #region InitialBoundingInfo
        private float _floorHeight;
        private Vector3[] _initialPoints;
        private Vector3 _initialCenter;
        private Vector3 _invInitialScale;
        #endregion

        #region RuntimeInfo
        private Vector3[] _virtualPositions;
        private Vector3[] _virtualVelocities;
        private Vector3[] _virtualAccelerations;

        private Vector3 _lastCenter = new Vector3();
        #endregion
        public XDynamicVirtualBox(Vector3[] _positions, Vector3 center)
        {
            _virtualAccelerations = new Vector3[_positions.Length];
            _virtualVelocities = new Vector3[_positions.Length];
            _virtualPositions = new Vector3[_positions.Length];
            _initialPoints = new Vector3[_positions.Length];

            for (int i = 0; i < _positions.Length; i++)
            {
                _virtualPositions[i] = _positions[i];
            }
            
            Vector3 offset = center - Vector3.zero;
            for (int i = 0; i < _virtualPositions.Length; i++)
            {
                _virtualPositions[i] -= offset;
            }
            
            InitIndexes();
            
            InitPhysics();
        }

        //----initialize----//
        
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
                    _springRestLengths[springIndex] = (_virtualPositions[pIndex0] - _virtualPositions[pIndex1]).magnitude;
                    springIndex++;
                }
            }
            //save diagonal edge springs' initial length
            for (int i = 0; i < numDiagonalSprings; ++i)
            {
                int pt0Index = _diagonalSpringPointMassIndexes[i, 0];
                int pt1Index = _diagonalSpringPointMassIndexes[i, 1];
                _springRestLengths[springIndex] =
                    (_virtualPositions[pt0Index] - _virtualPositions[pt1Index]).magnitude;
                springIndex++;
            }
            
            _faceNormals = new Vector3[numFaces];
            _faceAreas = new float[numFaces];
            
            _hullRestVolume = CalculateVolume(_virtualPositions);
            _initialCenter = Vector3.zero;
            _lastCenter = _initialCenter;
            _floorHeight = _virtualPositions[4].y;
            _invInitialScale = new Vector3(
                1 / (_virtualPositions[0] - _virtualPositions[3]).magnitude,
                1 / (_virtualPositions[0] - _virtualPositions[4]).magnitude,
                1 / (_virtualPositions[0] - _virtualPositions[1]).magnitude);
        }

        private float CalculateVolume(Vector3[] pos)
        {
            return (pos[0] - pos[4]).magnitude *
                   (pos[0] - pos[1]).magnitude *
                   (pos[0] - pos[3]).magnitude;
        }

        private Vector3 CalculateCenter(Vector3[] pos)
        {
            Vector3 center = new Vector3(0, 0, 0);
            for (int i = 0; i < pos.Length; i++)
            {
                center += pos[i];
            }

            return center / pos.Length;
        }
        
        public void Step(ref Vector3 offset, ref Vector3 scale)
        {
            for (int i = 0; i < 8; i++)
            {
                _virtualAccelerations[i] = new Vector3(0, -9.8f, 0);
            }
            
            ApplyImpact();
            
            AccumulateSpringForce();
            
            AccumulatePressureForce();
            
            AdvectParticles(Time.fixedDeltaTime, ref offset, ref scale);

            if (_virtualPositions[4].y < _floorHeight)
            {
                HandleBottomCollision();
            }
        }
        
        private Vector3 ComputeSpringForce(int idx0, int idx1, float restLength, float stiffness, float airDamping)
        {
            Vector3 pos0 = _virtualPositions[idx0];
            Vector3 pos1 = _virtualPositions[idx1];

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
            Vector3 relativeVel = _virtualVelocities[idx1] - _virtualVelocities[idx0];

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

                    _virtualAccelerations[pIndex0] += force;
                    _virtualAccelerations[pIndex1] -= force;

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

                _virtualAccelerations[pIndex0] += force;
                _virtualAccelerations[pIndex1] -= force;

                springIndex++;
            }
        }

        private void AccumulatePressureForce()
        {
            float currentVolume = CalculateVolume(_virtualPositions);

            float volumeRatio = currentVolume / _hullRestVolume;
            float surfaceArea = 0.0f;
            int numFaces = _facePointMassIndexes.GetLength(0);
            int numPtsPerFace = _facePointMassIndexes.GetLength(1);

            //caculate face normal and area
            for (int i = 0; i < numFaces; i++)
            {
                Vector3 pos0 = _virtualPositions[_facePointMassIndexes[i, 0]];
                Vector3 pos1 = _virtualPositions[_facePointMassIndexes[i, 1]];
                Vector3 pos2 = _virtualPositions[_facePointMassIndexes[i, 2]];

                Vector3 faceNormal = Vector3.Cross(pos0 - pos1, pos2 - pos1);
                float faceArea = faceNormal.magnitude;
                if (float.IsInfinity(faceArea))
                {
                    faceNormal = new Vector3(0, 1.0f, 0);
                    //faceArea = _transformStartScale.x * _transformStartScale.y;
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
                    
                    _virtualAccelerations[_facePointMassIndexes[i, j]] += pressureForce;
                    
                }
            }
        }
        
        private void AdvectParticles(float dt, ref Vector3 offset, ref Vector3 scale)
        {
            Vector3 boundsCenter = new Vector3();
            for (int i = 0; i < _virtualPositions.Length; ++i)
            {
                _virtualVelocities[i] += _virtualAccelerations[i] * dt;
    
                _virtualPositions[i] += _virtualVelocities[i] * dt;

                boundsCenter += _virtualPositions[i];
            }

            boundsCenter /= _virtualPositions.Length;

            _virtualPositions[0].y = _virtualPositions[1].y = _virtualPositions[2].y = _virtualPositions[3].y;

            CorrectVerticalPos(_virtualPositions, 0, 4);
            CorrectVerticalPos(_virtualPositions, 1, 5);
            CorrectVerticalPos(_virtualPositions, 2, 6);
            CorrectVerticalPos(_virtualPositions, 3, 7);
            
            Vector3 scaleCoef = new Vector3(
                (_virtualPositions[0] - _virtualPositions[3]).magnitude * _invInitialScale.x,
                (_virtualPositions[0] - _virtualPositions[4]).magnitude * _invInitialScale.y,
                (_virtualPositions[0] - _virtualPositions[1]).magnitude * _invInitialScale.z);

            scale = scaleCoef;

            //every loop's offset
            offset = boundsCenter - _lastCenter;
            _lastCenter = boundsCenter;
        }
        
        private void CorrectVerticalPos(Vector3[] pos, int idx1, int idx2)
        {
            pos[idx1].x = pos[idx2].x;
            pos[idx1].z = pos[idx2].z;
            if (pos[idx1].y < pos[idx2].y)
                pos[idx1].y = pos[idx2].y + 0.02f;
        }

        private void HandleBottomCollision()
        {
            for (int i = 4; i < 8; i++)
            {
                Vector3 p = _virtualPositions[i];
                _virtualPositions[i] = new Vector3(_virtualPositions[i].x, _floorHeight, _virtualPositions[i].z);
                
                float speedAlongNormalSigned = Vector3.Dot(_virtualVelocities[i], new Vector3(0, 1, 0));
                float speedAlongNormalSign = Mathf.Sign(speedAlongNormalSigned);
                Vector3 velocityAlongNormal = speedAlongNormalSigned * new Vector3(0,1,0);
                Vector3 slideVelocity = _virtualVelocities[i] - velocityAlongNormal;
                velocityAlongNormal *= speedAlongNormalSign; // reflect if opposing

                // reduce velocityAlongNormal by bounce coefficient if reflecting
                float bounceCoefficient = (speedAlongNormalSign >= 0.0f ? 1.0f : _bounceCoefficient);
                _virtualVelocities[i] = 
                    bounceCoefficient * velocityAlongNormal + 
                    slideVelocity * _slideCoefficient;                        
                
            }
        }

        public void RegistImpactEvent(XDynamicBoundingImpact xDynamicBoundingImpact)
        {
            _deferredImpactList.Add(xDynamicBoundingImpact);
        }

        private void ApplyImpact()
        {
            foreach (var impact in _deferredImpactList)
            {
                int[] pidx = impact._strainedParticleIdx;
                for (int i = 0; i < pidx.Length; i++)
                {
                    _virtualAccelerations[i] += impact._impactDir * impact._impulse;
                }
            }
            _deferredImpactList.Clear();
        }
        
    }
}