using Unity.Entities;

namespace Latios.Navigator.Systems
{
    [DisableAutoCreation]
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial class NavRootSystem : RootSuperSystem
    {
        protected override void CreateSystems()
        {
            EnableSystemSorting = true;
            GetOrCreateAndAddUnmanagedSystem<AgentEdgePathSystem>();
            GetOrCreateAndAddUnmanagedSystem<AgentPathFunnelingSystem>();
        }
    }
}

