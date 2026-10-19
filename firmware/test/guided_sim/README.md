# Guided clean host simulation

Runs the real `GuidedCleanManager` + `FloorMap` sources on the host against a
small physics model, so the controller can be exercised without a robot.

```bash
./firmware/test/guided_sim/run.sh       # build + run all scenarios
./firmware/test/guided_sim/run.sh -v    # with firmware LOG() output
```

`run.sh` copies the firmware sources under test into a temp dir next to the
fakes in `fake/` (Arduino `String`, SPIFFS on the host filesystem, and a
`NeatoSerial` stand-in), so the fakes shadow the hardware-facing headers.

The fake `NeatoSerial` simulates:

- a differential-drive robot with systematic and per-move random wheel slip,
  a carpet (extra slip, heading kicks at its edge),
- serial latency (60 ms, LIDAR scans 700 ms), failed answers (2%) and,
  per scenario, lost answers and a serial outage (watchdog tests),
- walls, mapped furniture (cabinet, table legs), unmapped chairs, a cliff region,
- the dock (contacts only when the robot sits within ~2 cm / 6° of the docked pose),
- a ray-cast LIDAR with noise and spurious short returns (CCW or CW, optional mounting offset),
- battery drain.

Before each scenario a likelihood field is rasterized from the world (walls
smeared sideways like a real mapping run) and uploaded.

```bash
./run.sh x basic|robust|flat     # one scenario group
SIM_SEED=7 ./run.sh              # other random seed
SIM_TRAJ_DIR=/tmp/t GUIDED_SIM_TRACE_DIR=/tmp/t ./run.sh   # true path CSV + run traces
SIM_DEBUG=1 ./run.sh             # print every contact with walls/furniture
```

Groups:

- **basic** (two rooms, 1 m door, no-go zone): coverage, unmapped chair, low battery,
  refusals, cliff, CW LIDAR, heavy drift, LIDAR offset, map origin off the dock,
  LIDAR that never delivers.
- **robust**: serial outage (must abort and leave TestMode), lost answers, "home" and
  "stop" mid-run, robot shoved 40 cm / 25° (re-localization), very smeared map.
- **flat** (hallway with the dock next to a cabinet, 80 cm door, living room with
  table legs, sofa, carpet, chairs): both rooms, then 10 runs in a row on the same
  map without recalibration.

Passing it means the control logic is consistent and robust against these
effects; it does not replace testing on a robot.
