using System.Collections;
using System.Collections.Generic;
using System.ComponentModel.Composition.Hosting;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;

[BurstCompile]
public struct SolveDistanceConstraintPass : IJobParallelFor
{
    [ReadOnly] public NativeArray<float> _RestLength;
    [ReadOnly] public NativeArray<int> _EdgeIdx;
    [ReadOnly] public NativeArray<float> _InvMass;
    [ReadOnly] public NativeArray<Vector3> _Pos;
    [ReadOnly] public float _alpha;

    [NativeDisableParallelForRestriction]
    public NativeArray<Vector3> _Correction;
    
    public void Execute(int index)
    {
        int id0 = _EdgeIdx[2 * index];
        int id1 = _EdgeIdx[2 * index + 1];

        float invMass0 = _InvMass[id0];
        float invMass1 = _InvMass[id1];
        Vector3 p0 = _Pos[id0];
        Vector3 p1 = _Pos[id1];

        float K = invMass1 + invMass0;
        if(K == 0.0f)
            return;
        Vector3 n = p0 - p1;
        float d = n.magnitude;
        if(d == 0.0f)
            return;
        
        float C = d - _RestLength[index];
        //Debug.Log(_RestLength[index]);
        K += _alpha;

        float Kinv = 1 / K;

        float lambda = -Kinv * (C);
        Vector3 pt = n * lambda;
        
        if (id0 != 200)
            _Correction[id0] += invMass0 * pt;
        if (id1 != 200)
            _Correction[id1] -= invMass1 * pt;
        
    }
    
}
