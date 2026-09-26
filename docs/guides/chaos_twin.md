# UE Physics Twin (Chaos)

`UMjTwinComponent` builds a **native Unreal Engine replica of a MJCF model** and runs
it on Chaos, Unreal's own physics engine. The regular URLab pipeline keeps MuJoCo as
the integrator and Unreal as the renderer; the twin inverts that: the model is
translated into Unreal rigid bodies, collision shapes, constraints, and force
generators, and Chaos simulates it. MuJoCo does not have to be running at all.

The use cases are the ones where a MuJoCo-owned simulation is awkward:

- Physics that has to interact with normal Unreal gameplay (character controllers,
  vehicles, destructibles, networked actors) without a bridge round-trip.
- Deterministic replay and packaged builds where the MuJoCo library is present but
  the bridge is not.
- A/B comparison of MuJoCo and Chaos on the same model, with the divergence
  measured instead of guessed.

!!! warning "Fidelity is bounded by Chaos, and stated"
    The twin is a *best-effort* replica: MuJoCo's contact model, soft equality
    constraints, tendons, deformables, and muscle actuators have no Chaos
    equivalent. Every translated element is therefore recorded in an
    `FMjTwinReport` with `Exact` / `Approximate` / `Unsupported` verdicts, and
    `ComputeDivergence()` measures the twin against the running MuJoCo model.
    Nothing is dropped silently.

## What the twin mirrors

| MJCF element | Twin representation | Fidelity |
| --- | --- | --- |
| Body (jointed) | One Chaos rigid body, physics frame at the principal/COM frame | Exact |
| Body (no joint, welded to parent) | Folded into the parent rigid body, mass properties combined with the parallel-axis theorem | Exact |
| Body (welded to world) | Static collision body | Exact |
| Mocap body | Non-simulating (kinematic) body | Approximate |
| `sphere`, `box`, `capsule` | Chaos sphere, box, capsule | Exact |
| `cylinder`, `ellipsoid` | Runtime convex hulls (tessellated) | Approximate |
| `mesh` | Convex hull of the imported Unreal mesh asset; bounding box when no asset exists | Approximate / Exact with an imported hull |
| `plane` | Finite slab (10 m × 10 m × 1 m) whose surface lies on the plane | Approximate |
| `hfield`, `sdf` | Not mirrored (reported) | Unsupported |
| `hinge`, `slide` | Chaos constraint with the same axis and range | Exact |
| `ball` | Chaos swing/twist constraint | Approximate |
| `free` | Unconstrained body | Exact |
| `motor`, `position`, `velocity`, `general`/affine actuators | Force generator evaluating MuJoCo's `gain * act + bias` law every physics step or sub-step | Exact |
| `damper` actuators | Velocity-proportional force | Exact |
| Actuator activation dynamics (`integrator`) | Integrated by the twin | Approximate |
| Actuator on a ball/free joint, muscle gain, user callbacks, plugins, tendons | Not mirrored (reported) | Unsupported |
| `connect`, `weld` equalities | Chaos constraints (rigid instead of MuJoCo's soft equality) | Approximate |
| Joint `stiffness`, dof `damping`, `frictionloss`, `armature` | Applied as forces on the joint axis; armature folded into the inertia | Exact / Approximate |
| `gravity`, `gravcomp` | Per-body acceleration correction, applied with the actuator forces | Exact for vertical gravity |
| `sensors`, `cameras`, tendons, deformables, heightfields, plugins | Not mirrored — read them from the URLab components | Unsupported |

## Quick start (C++)

```cpp
// On any actor that owns a UMjTwinComponent:
UMjTwinComponent* Twin = NewObject<UMjTwinComponent>(this);
Twin->SetupAttachment(GetRootComponent());
Twin->RegisterComponent();

// Builds from the MuJoCo model the manager has already compiled.
if (Twin->BuildFromArticulation(RobotActor))
{
    // The report lists every element with its fidelity verdict.
    UE_LOG(LogTemp, Log, TEXT("%s"), *Twin->GetReport().SummaryLine());

    // Drive it like any Unreal physics actor: controls are MuJoCo ctrl values.
    Twin->SetControlByName(TEXT("motor1"), 0.6);
}
```

The component is `BlueprintSpawnableComponent`, so the same flow works from
Blueprint: add the component, call **Build From Articulation** on Begin Play, then
**Set Control By Name**.

### Building from a descriptor

`FMjTwinDescriptorBuilder::BuildFromModel()` translates an `mjModel*` without any
Unreal world involvement, which is what the automation tests use:

```cpp
FMjTwinDescriptor Descriptor;
FString Error;
if (FMjTwinDescriptorBuilder::BuildFromModel(Engine->GetModel(), Descriptor, Error))
{
    Twin->BuildFromDescriptor(Descriptor);
}
```

## How the translation works

**State is read from the compiled `mjModel`, never from the XML.** Masses, inertia
tensors, geom sizes, joint frames, and actuator gains all come from the same data
MuJoCo integrates, so the twin cannot drift from the source model through a parser
difference.

**Units.** MuJoCo is metres/kilograms; Unreal is centimetres/kilograms. Positions
and sizes are multiplied by 100, inertias by 10⁴. Torques are converted to Unreal's
`kg·cm²/s²` (`×10⁴`) and forces to `kg·cm/s²` (`×100`).

**Frames.** URLab maps MuJoCo's right-handed frame to Unreal's left-handed frame by
mirroring Y (`p_ue = M p_mj`, `M = diag(1, -1, 1)`, `R_ue = M R_mj M⁻¹`). The twin
distinguishes polar vectors (`(x, -y, z)`), axial vectors (`(-x, y, -z)`), and
quaternions, so **every joint coordinate keeps the same sign** as its MuJoCo
counterpart — a positive `qpos` is a positive Unreal rotation about the joint axis,
and a positive control produces the same direction of motion.

**Bodies.** A MuJoCo body without a joint is welded to its parent. Chaos has no
equivalent, so each welded chain collapses into one rigid body whose mass, centre of
mass, and inertia tensor are the combined values. Chaos stores a diagonal inertia in
the body frame, so the body frame is chosen to be the **principal inertia frame**
with the COM at the origin — then the diagonal copy is exact rather than an
approximation.

**Joint ranges.** Chaos limits are symmetric about the constraint frame, so the
constraint frame is rotated to the midpoint of the MuJoCo range (`LimitOffset`) and
the limit is set to the half range. A hinge with `range="-30 90"` becomes a frame at
+30° with a ±60° twist limit, which spans exactly `[-30, 90]`.

**Actuators.** The twin evaluates MuJoCo's own law — `gain * act + bias` with
`gainprm` / `biasprm` / `dynprm`, the gear, and the force and control ranges — and
applies the result along the joint axis, with the reaction on the parent body. The
law can run once per frame or inside the physics sub-step (see below).

## Timestep and sub-stepping

Chaos advances rigid bodies once per sub-step; MuJoCo advances the whole model once
per `mj_step`. Matching them is what keeps the twin's trajectories close.

`FMjTwinBuildSettings.bMatchTimestep` (default on) enables Unreal's sub-stepping and
sets the sub-step from `mjOption.timestep`, so a 2 ms MuJoCo model runs with a 2 ms
Chaos sub-step. The default MuJoCo model steps at 500 Hz, which is well inside the
engine's sub-step budget; for very stiff models, keep an eye on the physics thread
cost.

`FMjTwinBuildSettings.bSubstepActuation` evaluates the actuator law inside the
sub-step instead of once per tick. Use it for high-gain position servos and for
models whose stability depends on the actuator seeing the state it reacts to.

## Divergence and playback

`ComputeDivergence()` compares the twin's bodies against the running MuJoCo
simulation (the latest `FMjRenderSnapshot`) and returns per-body position and
orientation errors plus the worst case:

```cpp
FMjTwinDivergence Divergence;
if (Twin->ComputeDivergence(Divergence))
{
    UE_LOG(LogTemp, Log, TEXT("%s"), *Divergence.ToText());
}
```

Set `bLogDivergence` to log it every tick while tuning.

`FMjTwinBuildSettings.bDriveFromMujoco` turns the twin into a kinematic playback of
the MuJoCo state instead: the bodies stop simulating and copy the snapshot every
tick. That is useful for visual parity checks and for comparing a render against a
Chaos result, but it is not a simulation.

## Reproducing MuJoCo state exactly

The twin is also the way to start a Chaos run from a MuJoCo state: build the twin,
let MuJoCo warm up or run a policy, then read back the twin's joint state with
`GetJointState()` (degrees or centimetres) and key the corresponding Unreal physics
bodies to it. Because the twin's joint coordinates carry MuJoCo's signs, no sign
juggling is needed when crossing between the two.

## Limitations

- **Contacts.** Chaos uses its own contact model: combine modes, restitution, and
  softness differ from MuJoCo's `solref` / `solimp` cone model. Geom friction is
  copied (sliding component) and combined with `max`, matching MuJoCo's default
  priority-0 rule, but torsional and rolling friction are not reproduced.
- **Equality constraints** are rigid in Chaos and soft in MuJoCo, so a
  model that leans on `solref` softness will behave more stiffly in the twin.
- **Ball joints** use a different parameterisation (swing/twist vs a quaternion) and
  are reported as approximate even when their limits are mirrored.
- **Parallel-axis limits.** Armature can only be folded into the inertia tensor when
  the joint axis is a principal axis of the body; otherwise it is applied as a force
  and reported as approximate.
- **Static geometry** welded to the world is mirrored, but `hfield` and `sdf` geoms
  are not, so a heightfield-only model has no ground in the twin. Use
  `AMjHeightfieldActor` or a mesh asset for those.

## API reference

- `UMjTwinComponent` — `Source/URLab/Public/MuJoCo/Twin/MjTwinComponent.h`
- `FMjTwinDescriptor`, `FMjTwinDescriptorBuilder` — `Source/URLab/Public/MuJoCo/Twin/MjTwinDescriptor.h`
- `FMjTwinReport`, `FMjTwinOptions`, `FMjTwinBuildSettings`, `FMjTwinDivergence` —
  `Source/URLab/Public/MuJoCo/Twin/MjTwinTypes.h`

Automation tests live in `Source/URLabEditor/Private/Tests/MjTwinTests.cpp` and cover
the translation itself (`URLab.Twin.*`): weld grouping, primitive mapping, joint range
midpoints, actuator classification, and the inertia diagonalisation.
