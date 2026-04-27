using Latios.Kinemation;
using Latios.Transforms;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Latios.Mecanim
{
#if !LATIOS_TRANSFORMS_UNITY
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [RequireMatchingQueriesForUpdate]
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct UpdateMecanimSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var job = new UpdateMecanimJob
            {
                socketLookup    = SystemAPI.GetComponentLookup<Socket>(true),
                transformHandle = new TransformAspectParallelChunkHandle(SystemAPI.GetComponentLookup<WorldTransform>(false),
                                                                         SystemAPI.GetComponentTypeHandle<RootReference>(true),
                                                                         SystemAPI.GetBufferLookup<EntityInHierarchy>(true),
                                                                         SystemAPI.GetBufferLookup<EntityInHierarchyCleanup>(true),
                                                                         SystemAPI.GetEntityStorageInfoLookup(),
                                                                         ref state),
                ElapsedTime = SystemAPI.Time.ElapsedTime,
                DeltaTime   = SystemAPI.Time.DeltaTime,
            };
            job.ScheduleByRef();
            state.Dependency = job.transformHandle.ScheduleChunkGrouping(state.Dependency);
            state.Dependency = job.GetTransformsScheduler().ScheduleParallel(state.Dependency);
        }

        [WithAll(typeof(WorldTransform))]
        [BurstCompile]
        public partial struct UpdateMecanimJob : IJobEntity, IJobEntityChunkBeginEnd, IJobChunkParallelTransform
        {
            [ReadOnly] public ComponentLookup<Socket> socketLookup;
            public TransformAspectParallelChunkHandle transformHandle;
            public double                             ElapsedTime;
            public float                              DeltaTime;

            public ref TransformAspectParallelChunkHandle transformAspectHandleAccess => ref transformHandle.RefAccess();

            public void Execute([EntityIndexInChunk] int indexInChunk,
                                MecanimAspect mecanimAspect,
                                RefRO<OptimizedSkeletonHierarchyBlobReference>     skeletonBlob,
                                RefRW<OptimizedSkeletonState>                      skeletonState,
                                ref DynamicBuffer<OptimizedBoneTransform>          boneTransforms,
                                ref DynamicBuffer<OptimizedBoneInertialBlendState> inertialBlendStates,
                                in DynamicBuffer<DependentSkinnedMesh>             dependentSkinnedMeshes)
            {
                var transform               = transformHandle[indexInChunk];
                var optimizedSkeletonAspect = new OptimizedSkeletonAspect(transform,
                                                                          ref socketLookup,
                                                                          skeletonBlob,
                                                                          skeletonState,
                                                                          ref boneTransforms,
                                                                          ref inertialBlendStates,
                                                                          dependentSkinnedMeshes);
                mecanimAspect.Update(optimizedSkeletonAspect, ElapsedTime, DeltaTime);

                if (!mecanimAspect.applyRootMotion)
                    return;

                var rootBone = optimizedSkeletonAspect.rawRootTransforms[0];
                // Reapply delta time scaling to root motion
                rootBone.position *= DeltaTime;
                // We scale rotation by an additional 100f to revert our 0.01f scale that prevents angle overflow
                rootBone.rotation = MathUtil.ScaleQuaternion(rootBone.rotation, 100f * DeltaTime);

                var result                   = RootMotionTools.ConcatenateDeltas(transform.localTransformQvvs, in rootBone);
                result.rotation              = math.normalize(result.rotation);
                transform.localTransformQvvs = result;
            }

            public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                return transformHandle.OnChunkBegin(in chunk, unfilteredChunkIndex, useEnabledMask, in chunkEnabledMask);
            }

            public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted)
            {
            }
        }
    }
#else
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(TransformSystemGroup))]
    [RequireMatchingQueriesForUpdate]
    [DisableAutoCreation]
    [BurstCompile]
    public partial struct UpdateMecanimSystem : ISystem
    {
        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var job = new UpdateMecanimJob
            {
                ElapsedTime = SystemAPI.Time.ElapsedTime,
                DeltaTime   = SystemAPI.Time.DeltaTime,
            };
            job.ScheduleParallelByRef();
        }

        [BurstCompile]
        public partial struct UpdateMecanimJob : IJobEntity
        {
            public double ElapsedTime;
            public float DeltaTime;

            public void Execute([EntityIndexInChunk] int indexInChunk,
                                MecanimAspect mecanimAspect,
                                RefRO<OptimizedSkeletonHierarchyBlobReference>     skeletonBlob,
                                RefRW<OptimizedSkeletonState>                      skeletonState,
                                ref DynamicBuffer<OptimizedBoneTransform>          boneTransforms,
                                ref DynamicBuffer<OptimizedBoneInertialBlendState> inertialBlendStates,
                                in DynamicBuffer<DependentSkinnedMesh>             dependentSkinnedMeshes,
                                ref LocalTransform localTransform,
                                in LocalToWorld localToWorld)
            {
                var optimizedSkeletonAspect = new OptimizedSkeletonAspect(in localToWorld,
                                                                          skeletonBlob,
                                                                          skeletonState,
                                                                          ref boneTransforms,
                                                                          ref inertialBlendStates,
                                                                          dependentSkinnedMeshes);
                mecanimAspect.Update(optimizedSkeletonAspect, ElapsedTime, DeltaTime);

                if (!mecanimAspect.applyRootMotion)
                    return;

                var rootBone = optimizedSkeletonAspect.rawRootTransforms[0];
                // Reapply delta time scaling to root motion
                rootBone.position *= DeltaTime;
                // We scale rotation by an additional 100f to revert our 0.01f scale that prevents angle overflow
                rootBone.rotation = MathUtil.ScaleQuaternion(rootBone.rotation, 100f * DeltaTime);

                var transform = new TransformQvvs(localTransform.Position, localTransform.Rotation, 1f, localTransform.Scale);
                var result    = RootMotionTools.ConcatenateDeltas(transform, in rootBone);
                result.rotation         = math.normalize(result.rotation);
                localTransform.Rotation = result.rotation;
                localTransform.Position = result.position;
                localTransform.Scale    = result.stretch.x;
            }
        }
    }
#endif
}

