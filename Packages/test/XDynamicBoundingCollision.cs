using System;
using System.Collections;
using System.Collections.Generic;
using Sirenix.OdinInspector;
using UnityEngine;

namespace XRender.Scripting
{
    public class XDynamicBoundingContact : MonoBehaviour
    {
        public XDynamicSurface strainedSurfaceA;
        public XDynamicSurface strainedSurfaceB;
        
        XDynamicBoundingContact(XDynamicSurface surface1, XDynamicSurface surface2)
        {
            strainedSurfaceA = surface1;
            strainedSurfaceB = surface2;
        }
    }

    public class XDynamicBoundingImpact
    {
        public int[] _strainedParticleIdx = new int[8];
        public float _impulse;
        public Vector3 _impactDir;
        public XDynamicBoundingImpact(int[] pids, float impulse, Vector3 impactDir)
        {
            _strainedParticleIdx = pids;
            _impulse = impulse;
            _impactDir = impactDir;
        }
    }
    
}