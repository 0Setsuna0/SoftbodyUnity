using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

[BurstCompile]
public struct PresolvePass : IJobParallelFor
{
    [ReadOnly] public Vector3 Gravity;
    [ReadOnly] public float dt;
    [ReadOnly] public NativeArray<float> InvMass;

    [WriteOnly] public NativeArray<Vector3> PrevPos;

    public NativeArray<Vector3> Vel;
    public NativeArray<Vector3> Pos;
    
    public void Execute(int index)
    {
        if (InvMass[index] == 0 || index == 200)
            return;

        Vel[index] += dt * Gravity;
        PrevPos[index] = Pos[index];
        Pos[index] += dt * Vel[index];
    }
}
