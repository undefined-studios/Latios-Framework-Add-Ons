using System.Collections.Generic;
using Latios.Navigator.Components;
using Latios.Navigator.Utils;
using Latios.Transforms;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Navigator.Systems
{
    [RequireMatchingQueriesForUpdate]
    internal partial struct AgentEdgePathSystem : ISystem
    {
        EntityQuery          m_query;
        LatiosWorldUnmanaged m_latiosWorld;

        [BurstCompile]
        public void OnCreate(ref SystemState state)
        {
            m_latiosWorld = state.GetLatiosWorldUnmanaged();
            m_query       = state.Fluent()
                            .With<WorldTransform>()
                            .With<NavmeshAgentTag>()
                            .WithEnabled<NavMeshAgent>()
                            .With<AgentDestination>()
                            .With<AgentPath>()
                            .With<AgentPathEdge>()
                            .WithEnabled<AgenPathRequestedTag>()
                            .Build();

            state.RequireForUpdate<NavMeshSurfaceBlobReference>();
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            var navMeshSurfaceBlob =
                m_latiosWorld.GetNavMeshSurfaceBlob();

            var job = new PathJob
            {
                AgentHasEdgePathTagLookup          = SystemAPI.GetComponentLookup<AgentHasEdgePathTag>(),
                AgenPathRequestedTagLookup         = SystemAPI.GetComponentLookup<AgenPathRequestedTag>(),
                NavMeshSurfaceBlob                 = navMeshSurfaceBlob,
                TransformAspectParallelChunkHandle = new TransformAspectParallelChunkHandle(SystemAPI.GetComponentLookup<WorldTransform>(false),
                                                                                            SystemAPI.GetComponentTypeHandle<RootReference>(true),
                                                                                            SystemAPI.GetBufferLookup<EntityInHierarchy>(true),
                                                                                            SystemAPI.GetBufferLookup<EntityInHierarchyCleanup>(true),
                                                                                            SystemAPI.GetEntityStorageInfoLookup(),
                                                                                            ref state)
            };
            state.Dependency = job.ScheduleByRef(state.Dependency);
            state.Dependency = job.TransformAspectParallelChunkHandle.ScheduleChunkGrouping(state.Dependency);
            state.Dependency = job.GetTransformsScheduler().ScheduleParallel(state.Dependency);
        }

        [BurstCompile]
        partial struct PathJob : IJobEntity, IJobChunkParallelTransform, IJobEntityChunkBeginEnd
        {
            [BurstCompile]
            struct QueueElement
            {
                public int    Index;
                public int    Cost;
                public float3 MidPoint;  // Midpoint of the portal for better pathfinding
            }

            [BurstCompile]
            struct CostComparer : IComparer<QueueElement>
            {
                public int Compare(QueueElement x,
                                   QueueElement y) => x.Cost.CompareTo(y.Cost);
            }

            [ReadOnly]                            public NavMeshSurfaceBlobReference           NavMeshSurfaceBlob;
            [NativeDisableParallelForRestriction] public ComponentLookup<AgentHasEdgePathTag>  AgentHasEdgePathTagLookup;
            [NativeDisableParallelForRestriction] public ComponentLookup<AgenPathRequestedTag> AgenPathRequestedTagLookup;
            public TransformAspectParallelChunkHandle                                          TransformAspectParallelChunkHandle;

            public ref TransformAspectParallelChunkHandle transformAspectHandleAccess => ref TransformAspectParallelChunkHandle.RefAccess();

            void Execute(Entity entity, [EntityIndexInChunk] int indexInChunk,
                         in NavMeshAgent navmeshAgent,
                         in AgentDestination destination,
                         ref AgentPath agentPath,
                         ref DynamicBuffer<AgentPathEdge> buffer)
            {
                var transform = TransformAspectParallelChunkHandle[indexInChunk];

                ref var blobAsset           = ref NavMeshSurfaceBlob.NavMeshSurfaceBlob.Value;
                var     destinationPosition = destination.Position;  // Target position for the funnel algorithm

                // Determine start and goal triangles for A*
                if (!NavUtils.TryFindTriangleContainingPoint(transform.worldPosition, ref blobAsset,
                                                             out var startTriangleIndex))
                    // Agent is not on the navmesh, find the closest triangle to the agent's position
                    if (!NavUtils.FindClosestTriangleToPoint(transform.worldPosition, ref blobAsset,
                                                             out startTriangleIndex))
                    {
                        // No triangles found near the agent, cannot proceed with pathfinding
                        buffer.Clear();
                        return;
                    }

                // Check if the destination is on the navmesh
                if (!NavUtils.TryFindTriangleContainingPoint(destinationPosition, ref blobAsset,
                                                             out var goalTriangleIndex))
                    // Destination is not on the navmesh, find the closest triangle to the destination position
                    if (!NavUtils.FindClosestTriangleToPoint(destinationPosition, ref blobAsset,
                                                             out goalTriangleIndex))
                    {
                        // No triangles found near the destination, cannot proceed with pathfinding
                        buffer.Clear();
                        return;
                    }

                if (startTriangleIndex == goalTriangleIndex)
                {
                    // Agent and destination are in the same triangle.
                    // Path is just start to end point. Funnel algorithm might still be used with agent pos and target pos.
                    // Clear buffer and add a single segment if your funnel expects it.
                    buffer.Clear();
                    // Add a "degenerate" portal or handle in funnel algorithm
                    buffer.Add(new AgentPathEdge
                    {
                        PortalVertex1 = transform.worldPosition, PortalVertex2 = destinationPosition
                    });

                    AgenPathRequestedTagLookup.SetComponentEnabled(entity, false);
                    AgentHasEdgePathTagLookup.SetComponentEnabled(entity, true);
                    return;
                }

                var priorityQueue = new NativeMinHeap<QueueElement, CostComparer>(
                    blobAsset.Triangles.Length, Allocator.Temp);

                priorityQueue.Enqueue(new QueueElement
                {
                    Index    = startTriangleIndex,  // Start A* from the agent's current triangle
                    Cost     = 0,
                    MidPoint = transform.worldPosition  // Use agent's position as the initial midpoint
                });

                var cameFrom  = new NativeParallelHashMap<int, int>(blobAsset.Triangles.Length, Allocator.Temp);
                var costSoFar = new NativeParallelHashMap<int, int>(blobAsset.Triangles.Length, Allocator.Temp);
                costSoFar.TryAdd(startTriangleIndex, 0);

                var foundGoal = false;
                while (priorityQueue.TryDequeue(out var element))
                {
                    if (element.Index == goalTriangleIndex)
                    {
                        foundGoal = true;
                        break;  // Found the goal triangle
                    }

                    // Process the current triangle's neighbors
                    var offsetData             = blobAsset.AdjacencyOffsets[element.Index];
                    var currentTriangleForCost = NavUtils.GetTriangleByIndex(element.Index, ref blobAsset);

                    for (var i = offsetData.x; i < offsetData.x + offsetData.y; i++)
                    {
                        var neighborIndex           = blobAsset.AdjacencyIndices[i];
                        var neighborTriangleForCost = NavUtils.GetTriangleByIndex(neighborIndex, ref blobAsset);

                        if (!NavUtils.TryGetSharedPortalVertices(in currentTriangleForCost, in neighborTriangleForCost,
                                                                 out var p1, out var p2))
                            continue;

                        var targetMidPoint = (p1 + p2) * 0.5f;

                        // Calculate the cost to reach this neighbor triangle
                        var newCost = costSoFar[element.Index] + (int)(math.distance(
                                                                           element.MidPoint, targetMidPoint) * 10);

                        // Check if the neighbor triangle is already in the cost map or if the new cost is lower
                        if (!costSoFar.TryGetValue(neighborIndex, out var existingCost) || newCost < existingCost)
                        {
                            costSoFar[neighborIndex] = newCost;
                            // Add or update the neighbor in the priority queue
                            priorityQueue.Enqueue(new QueueElement
                            {
                                Index    = neighborIndex,
                                Cost     = newCost,
                                MidPoint = targetMidPoint
                            });

                            cameFrom[neighborIndex] = element.Index;
                        }
                    }
                }

                buffer.Clear();  // Clear previous path
                if (!foundGoal)
                {
                    buffer.Add(new AgentPathEdge
                    {
                        PortalVertex1 = transform.worldPosition, PortalVertex2 = transform.worldPosition
                    });

                    AgenPathRequestedTagLookup.SetComponentEnabled(entity, false);
                    AgentHasEdgePathTagLookup.SetComponentEnabled(entity, true);

                    cameFrom.Dispose();
                    costSoFar.Dispose();
                    priorityQueue.Dispose();
                    return;
                }

                // Reconstruct the path of portals
                var tempPathPortals =
                    new NativeList<AgentPathEdge>(Allocator.Temp);

                var currentPathIndex = goalTriangleIndex;
                while (currentPathIndex != startTriangleIndex)
                {
                    if (!cameFrom.TryGetValue(currentPathIndex, out var previousPathIndex))
                        break;

                    var triCurrent  = NavUtils.GetTriangleByIndex(currentPathIndex, ref blobAsset);
                    var triPrevious = NavUtils.GetTriangleByIndex(previousPathIndex, ref blobAsset);

                    // Check if the triangles share a portal
                    if (NavUtils.TryGetSharedPortalVertices(triPrevious, triCurrent, out var p1, out var p2))
                        // Add portal between triPrevious and triCurrent
                        tempPathPortals.Add(new AgentPathEdge
                        {
                            PortalVertex1 = p1,
                            PortalVertex2 = p2
                        });
                    else
                        break;

                    currentPathIndex = previousPathIndex;
                }

                buffer.Add(new AgentPathEdge
                {
                    PortalVertex1 = transform.worldPosition,
                    PortalVertex2 = transform.worldPosition
                });  // Add start position as first portal vertex

                // Reverse the order of portals to start from the agent's position
                for (var i = tempPathPortals.Length - 1; i >= 0; i--)
                {
                    var portal = tempPathPortals[i];
                    buffer.Add(portal);
                }

                buffer.Add(new AgentPathEdge
                {
                    PortalVertex1 = destinationPosition,
                    PortalVertex2 = destinationPosition
                });  // Add destination position as last portal vertex

                tempPathPortals.Dispose();
                cameFrom.Dispose();
                costSoFar.Dispose();
                priorityQueue.Dispose();

                AgenPathRequestedTagLookup.SetComponentEnabled(entity, false);
                AgentHasEdgePathTagLookup.SetComponentEnabled(entity, true);
            }

            public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                return TransformAspectParallelChunkHandle.OnChunkBegin(in chunk, unfilteredChunkIndex, useEnabledMask, in chunkEnabledMask);
            }

            public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask, bool chunkWasExecuted)
            {
            }
        }
    }
}

