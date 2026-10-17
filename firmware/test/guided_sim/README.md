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

- a differential-drive robot with a 0.4% right-wheel slip (heading drift),
- serial latency (every callback deferred by 60 ms),
- walls, unrecorded obstacles (bumper + wheel stall), a cliff region (drop sensors),
- the dock (contacts detected when the robot sits on the origin facing +x),
- a ray-cast LIDAR (CCW or CW angle convention, optional mounting offset),
- battery drain.

Before each scenario a likelihood field is rasterized from the world walls and
uploaded, as the frontend does after a mapping run (unless the scenario disables it).

Scenarios (two rooms joined by a door, one no-go zone in room 2):

| Scenario                  | Checks                                                  |
|---------------------------|---------------------------------------------------------|
| room 2 with no-go         | completes, docks, never enters the no-go, coverage >80% |
| chair in the way          | detours an unrecorded obstacle, docks                   |
| low battery               | returns early, docks                                    |
| battery too low / not docked | refuses to start                                     |
| cliff in room 2           | backs off drops, completes                              |
| clockwise LIDAR           | compass direction auto-calibration                      |
| no compass, low drift     | odometry-only fallback                                  |
| lidar offset + furniture  | compass + dock ICP with an off-centre turret            |
| room 1 only               | recording detours via room 2 — robot must not follow it |
| heavy drift with map      | 2% wheel slip + wrong wheel base, corrected by map matching |
| no map field              | compass fallback without a localization field           |
| map origin off the dock   | recording origin ≠ dock: docking uses the measured dock pose |

The model is deliberately simple (no wheel inertia, perfect encoders apart from
slip, no carpet). Passing it means the control logic is consistent; it does not
replace testing on a robot.
