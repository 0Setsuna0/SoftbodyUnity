using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
[BurstCompile]
public struct ExSolveBendingPass : IJobParallelFor
{
    [ReadOnly] public Vector3 _Gravity;
    [ReadOnly] public float _dt;
    [ReadOnly] public NativeArray<float> _InvMass;
    
    public NativeArray<Vector3> _Vel;
    public NativeArray<Vector3> _Pos;
    
    public void Execute(int index)
    {
        if (_InvMass[index] == 0)
            return;
        
    }
}