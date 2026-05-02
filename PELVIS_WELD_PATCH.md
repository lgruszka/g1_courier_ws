# Pelvis weld pin patch for `unitree_mujoco` G1 scene

## Why

In Phase 1 (arm tasks: pick, place, dock LIDAR_LINE), the robot does not
need to walk. It only needs to **stand still** while arms move. The G1 in
MuJoCo has no built-in walking controller — without one, gravity drops it
on the first step. We tried (a) per-joint PD on legs (oscillates + falls),
and (b) a pretrained RL policy from `unitree_rl_gym` (also oscillates
because the policy was not trained against arm-force disturbances).

The reference project `~/AI/g1_logistics_demo` solved the same problem by
**pinning the pelvis to a static `mocap` body via a MuJoCo `<equality>
<weld>` constraint**. The robot stands as if rigidly anchored. Arms move
freely with the real torque physics. Walking proper (`/cmd_vel` → leg
motion) is intentionally deferred — when we get there (Phase 1.3 with a
proper policy) we'll remove the weld.

This is a **sim-only** modification. The real robot has firmware sport
mode, so no equivalent is needed there.

## What to add (mac side)

Edit `~/code/unitree_mujoco/unitree_robots/g1/scene.xml` (the file the
running `simulate_python/unitree_mujoco.py` loads — `config.py` has
`ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"`).

Two insertions inside `<mujoco>...</mujoco>`:

### 1. Add anchor body to `<worldbody>`

Inside the existing `<worldbody>` (after the `<light>` and `<geom name="floor">`
elements, before `</worldbody>`):

```xml
<!-- Anchor for pelvis weld - keeps the robot upright so arm tasks
     (pick/place) can run without a walking controller. Sim-only.
     Phase 1.3 with a real walking policy will remove this body and the
     <equality><weld> below. -->
<body name="pelvis_anchor" mocap="true" pos="0 0 0.793"/>
```

The `pos="0 0 0.793"` is the standing height of the G1 pelvis (taken
verbatim from the `g1_logistics_demo` reference scene). Adjust if your
G1 model has a different default standing height.

### 2. Add `<equality>` block before `</mujoco>`

Just before the final `</mujoco>` tag (and after `</worldbody>`):

```xml
<equality>
  <weld name="pin_pelvis" body1="pelvis" body2="pelvis_anchor"
        solref="0.001 1" solimp="0.99 0.99 0.001"/>
</equality>
```

`solref="0.001 1"` makes the weld near-rigid (timestep 1 ms, damping 1).
`solimp="0.99 0.99 0.001"` clamps both impedance bounds at 0.99 so the
constraint is essentially infinite stiffness.

## After patching

1. Save `scene.xml`.
2. Restart `unitree_mujoco`:
   ```
   cd ~/code/unitree_mujoco/simulate_python
   python3 unitree_mujoco.py
   ```
3. The robot should appear standing upright in the viewer, motionless,
   and stay that way regardless of what `/lowcmd` sends to the legs.
   Arms still respond to `/lowcmd` motor_cmd values normally.
4. From the Linux Parallels side run `phase1_smoke.launch.py` —
   pick/place sequences now play back against a stable robot.

## To remove later (when we add real walking)

Delete the `<body name="pelvis_anchor" .../>` and the entire `<equality>`
block. Restart. Robot will fall again unless a walking controller is
publishing leg torques.
