using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace XRender.Scripting
{
    public struct Surface
    {
        public Vector3 Point;
        public Vector3 Normal;
        public bool beingCompressed;
        private float impulse;
        public Surface(Vector3 vec3, Vector3 normal)
        {
            Point = vec3;
            Normal = normal;
            beingCompressed = false;
            impulse = 0.0f;
        }

        public void CalculateImpulse(Vector3[] strainedVelocities)
        {
            impulse = 0.0f;
            for (int i = 0; i < strainedVelocities.Length; i++)
            {
                impulse += Vector3.Dot(-Normal, strainedVelocities[i]);
            }
        }
        
        public void HandleContact(ref Vector3[] strainedPositions, ref Vector3[] strainedVelocities)
        {
            if(!beingCompressed)
                return;

            for (int i = 0; i < strainedPositions.Length; i++)
            {
                
            }
        }
        
        public bool IsParticleInVirtualSurface(Vector3 pos)
        {
            Vector3 s1 = pos - Point;
            float d = Vector3.Dot(s1, Normal);
            if (d < 0)
                return true;
            else
                return false;
        }
    }

    public enum  SurfaceType
    {
        Top,
        Bottom,
        Left,
        Right,
        Front,
        Back,
    }
}
