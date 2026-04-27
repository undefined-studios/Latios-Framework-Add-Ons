# Mecanim V2

This is a rewrite of the Mecanim runtime attempting to resolve deep-rooted
issues in the original implementation. With the Mecanim runtime, you can use the
familiar Game Object-based animation tooling in your Latios Framework project.

## Getting Started

**Scripting Define:** LATIOS_ADDON_MECANIM_V2

**Requirements:**

-   Requires Latios Framework 0.15.0 or newer

**Main Author(s):** Dreaming I’m Latios & Alejandro Nunez

**Additional Contributors:**

**Support:** Feel free to report bugs (preferably with repro projects) via
normal Latios Framework support channels. Contributions and collaborations for
new features are welcome.

### Installing

Add the following installer lines to your bootstrap after Kinemation:

```csharp
// In LatiosBakingBootstrap
Latios.Mecanim.Authoring.MecanimBakingBootstrap.InstallMecanimAddon(ref context);

// In LatiosBootstrap
Latios.Mecanim.MecanimBootstrap.InstallMecanimAddon(world);
```

The Mecanim runtime will only take effect when an Animator is baked with a valid
*Animator Controller*. If this field is left null, then it is assumed the
character is a static pose or is being driven by usage of the low-level
Kinemation APIs.

The Mecanim runtime supports both QVVS and Unity Transforms. However, it only
supports optimized skeletons at this time. It is strongly recommended you review
the [Kinemation Getting
Started](https://github.com/Dreaming381/Latios-Framework-Documentation/blob/main/Kinemation%20Animation%20and%20Rendering/Getting%20Started%20-%20Part%201.md)
series before using this add-on.

## MecanimAspect

The `MecanimAspect` is an `IAspect` which allows you to modify the runtime
parameters to drive the state machine. The parameter manipulation API mimics the
classical API, but with a few twists.

The easiest to use API is the one where you pass in a string:

```csharp
mecanimAspect.SetFloatParameter(“strafe”, strafeValue);
```

While the easiest, it is also the slowest. It may be sufficient in many cases,
but you may prefer instead to use one of the hash APIs.

```csharp
FixedString64Bytes parameterName  = "strafe";
var                parameterHash  = parameterName.GetHashCode();
var                parameterIndex = mecanimAspect.GetParameterIndex(parameterHash);
mecanimAspect.SetFloatParameter(parameterIndex, strafeValue);
```

This method is slightly faster, but for the best performance, you want to bake
the parameter index and use that directly.

### Baking the Parameter Index

Inside a Baker, when using the `Latios.Kinemation.Authoring` namespace, use the
`IBaker` extension method `GetBaseControllerOf()` and pass in the Animator
component’s `RuntimeAnimatorController`.

That method will return a handle object from which you can retrieve the
`parameters`.

Another extension method is provided for the `parameters` array called
`TryGetParameter()` which will search through the parameters for the specified
name, and if found, return the parameter index as an `out` argument. You can
save this index inside an `IComponentData` or blob asset for use at runtime.

### Discrete Parameter Smoothing

Sometimes, game logic may dictate sudden changes in parameters that can result
in large frame-to-frame differences in the resulting animation. While
traditionally, the approach would be to smooth out parameters for smoother
blending, Mecanim V2 offers an alternative. At any time, you can start an
inertial blend from the previous frame via `StartInertialBlend()`. This smooths
out sudden drastic changes to blend trees or layer weights.

## Layers

Similar to parameters, you can retrieve layer indices at runtime through
`MecanimAspect` or at bake time via the `GetLayerIndex()` method on the base
controller. You can then set the layer weights at runtime either through
`MecanimAspect.SetLayerWeight()`, or directly via `DynamicBuffer<LayerWeights>`.
Index 0 should always have a weight of 1f, and the impact of the weight of each
layer after depends on the layer blend mode. For additive blending, the weight
dictates how much of additive animation is added on top, where a weight of `0f`
omits the addition, and a weight of `1f` applies the full strength of the
animation. For override blending, the weight acts as an interpolation factor
between all the combined layers prior (layer indices less than the current) and
the current layer’s animation. If you had a base layer of weight `1f`, and two
override layers of `0.5f` each, the overall influences of each layer would be
0.25, 0.25, and 0.5 respectively.

## Events

The Mecanim runtime cannot handle clip events at this time. However, it is able
to provide state transition events. You can retrieve the state machine index and
state index either through `MecanimAspect` APIs or from the
`MecanimControllerBlob` in an `ISmartPostProcessItem` during baking. A state
machine represents the full state machines for an animation layer, including all
sub-state machines which are flattened. Sync layers may share the same state
machine index. States within a state machine are named based on their authoring
full path name, including all sub-state machines with `.` delineation. State
transition events are added to the `DynamicBuffer<MecanimStateTransitionEvent>`.

## Manual Updates

You can enable or disable animation via the enabled property of `MecanimAspect`,
or by setting the enabled state of `MecanimController`. When the controller is
disabled, you can still update it manually using `MecanimAspect.Update()`. The
`elaspedTime` argument is solely used for event generation.

## Other Mecanim Runtime Quirks

The Mecanim runtime does not aim to be a perfect recreation of Unity’s classical
implementation. In some cases, there may be frame or blending differences. For
example, when classical Unity interrupts a transition, it saves the pose at the
point of interruption and blends between that static pose and the target
interrupting state. In contrast, this add-on elects to immediately transition to
the target interrupting state and trigger a new inertial blend to smooth out the
motion.
