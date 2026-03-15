using System;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    internal static partial class FlowFieldInternal
    {
        [BurstCompile]
        internal struct CalculateAgentsDirectionsJob : IJobChunk
        {
            [ReadOnly] internal Flow Flow;
            [ReadOnly] internal Field Field;
            internal FlowFieldAgentsTypeHandles TypeHandles;
            internal float DeltaTime;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var chunkTransforms = TypeHandles.WorldTransform.Resolve(chunk);
                var controls = chunk.GetNativeArray(ref TypeHandles.AgentDirection);
                var prevPositions = chunk.GetNativeArray(ref TypeHandles.PrevPosition);
                var velocities = chunk.GetNativeArray(ref TypeHandles.Velocity);
                var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                while (enumerator.NextEntityIndex(out var i))
                {
                    var position = chunkTransforms[i].position;
                    var prevPosition = prevPositions[i].Value;
                    var newVelocity = (position.xz - prevPosition) / DeltaTime;
                    velocities[i] = new FlowField.Velocity { Value = newVelocity };
                    prevPositions[i] = new FlowField.PrevPosition { Value = position.xz };

                    var direction = SampleFlowBilinear(position, in Field, in Flow);

                    var flowStrength = math.length(direction);
                    if (flowStrength < 0.01f)
                    {
                        controls[i] = new FlowField.AgentDirection { Value = float2.zero };
                        continue;
                    }

                    var density = SampleDensityBilinear(position, in Field);
                    var densityRatio = math.saturate(density / FlowSettings.MaxDensity);
                    direction *= 1f - densityRatio * densityRatio;

                    var prevDir = controls[i].Value;
                    direction = math.lerp(prevDir, direction, math.saturate(8f * DeltaTime));
                    controls[i] = new FlowField.AgentDirection { Value = direction };
                }
            }
        }

        static float2 SampleFlowBilinear(float3 worldPos, in Field field, in Flow flow)
        {
            WorldToGridFrac(worldPos, in field, out var cellMin, out var frac);

            var d00 = SampleCell(cellMin, in field, in flow);
            var d10 = SampleCell(cellMin + new int2(1, 0), in field, in flow);
            var d01 = SampleCell(cellMin + new int2(0, 1), in field, in flow);
            var d11 = SampleCell(cellMin + new int2(1, 1), in field, in flow);

            var dx0 = math.lerp(d00, d10, frac.x);
            var dx1 = math.lerp(d01, d11, frac.x);
            var result = math.lerp(dx0, dx1, frac.y);

            return math.normalizesafe(result) * math.length(result);
        }

        static float2 SampleCell(int2 cell, in Field field, in Flow flow)
        {
            cell = math.clamp(cell, 0, new int2(field.Width - 1, field.Height - 1));
            var index = Grid.CellToIndex(field.Width, cell);
            var direction = flow.GetDirection(index);
            var speedFactor = field.GetSpeedFactor(index);
            return direction * speedFactor;
        }

        static float SampleDensityBilinear(float3 worldPos, in Field field)
        {
            WorldToGridFrac(worldPos, in field, out var cellMin, out var frac);

            var d00 = SampleDensityCell(cellMin, in field);
            var d10 = SampleDensityCell(cellMin + new int2(1, 0), in field);
            var d01 = SampleDensityCell(cellMin + new int2(0, 1), in field);
            var d11 = SampleDensityCell(cellMin + new int2(1, 1), in field);

            return math.lerp(
                math.lerp(d00, d10, frac.x),
                math.lerp(d01, d11, frac.x),
                frac.y);
        }

        static void WorldToGridFrac(float3 worldPos, in Field field, out int2 cellMin, out float2 frac)
        {
            var localPos = worldPos - field.Transform.Value.position;
            var invRotation = math.inverse(field.Transform.Value.rotation);
            var unrotatedPos = math.rotate(invRotation, localPos);
            var adjustedPos = unrotatedPos - field.GetGridOffset();

            var gridPos = new float2(
                adjustedPos.x / field.CellSize.x,
                adjustedPos.z / field.CellSize.y
            );

            var samplePos = gridPos - 0.5f;
            cellMin = (int2)math.floor(samplePos);
            frac = samplePos - math.float2(cellMin);
        }

        static float SampleDensityCell(int2 cell, in Field field)
        {
            cell = math.clamp(cell, 0, new int2(field.Width - 1, field.Height - 1));
            return field.GetDensity(Grid.CellToIndex(field.Width, cell));
        }
    }
}