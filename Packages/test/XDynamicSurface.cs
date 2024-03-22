using UnityEngine;
using UnityEngine.Assertions.Must;

namespace XRender.Scripting
{
    public enum XDynamicSurfaceType
    {
        Top = 0,
        Bottom = 5,
        Left = 3,
        Right = 1,
        Front = 4,
        Back = 2
    };
    public class XDynamicSurface : MonoBehaviour
    {
        public struct Surface
        {
            public Vector3 Point;
            public Vector3 Normal;
            public Vector3[] Points;
            
            private float impulse;
            public Surface(Vector3 vec3, Vector3 normal)
            {
                Point = vec3;
                Normal = normal;
                impulse = 0.0f;
                Points = new Vector3[8];
            }
    
            public void CalculateImpulse(Vector3[] strainedVelocities)
            {
                impulse = 0.0f;
                for (int i = 0; i < strainedVelocities.Length; i++)
                {
                    impulse += Vector3.Dot(-Normal, strainedVelocities[i]);
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
}