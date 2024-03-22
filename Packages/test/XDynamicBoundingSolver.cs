using System.Collections.Generic;
using UnityEngine;

namespace XRender.Scripting
{
    public class XDynamicBoundingSolver : MonoBehaviour
    {
        private List<XDynamicBoundingDeformer> m_BoundingBoxes;

        private List<XDynamicBoundingContact> m_CollisionContacts;
        
        
        
        public void Step(ref XDynamicBoundingDeformer xDeformer)
        {
            
        }
    }
}