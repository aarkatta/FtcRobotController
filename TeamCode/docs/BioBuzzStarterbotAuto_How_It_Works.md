# BioBuzzStarterbotAuto: How It Works and What It Needs

Nov 7, 2026 (telemetry updated Nov 12, 2026)

BioBuzzStarterbotAuto is the team's competition autonomous; on the Driver Station it is "BioBuzz StarterBot Auto", next to its teleop partner "BioBuzz StarterBot Teleop". It was called UltronAuto until Nov 12, 2026. Don't confuse it with goBILDA's sample `StarterBotAuto` (on the Driver Station: "goBILDA Sample Auto (reference)"), which is only for reference.

The auto starts facing the hive, shoots the 4 preloaded balls, drives 7 ft straight under the hive, turns left and drives 3 ft on to the parking spot (blue tape). The route is measured from the blue base. In a headless test model it finished in about 11.6 s of the 30 s autonomous.

## What it does in a match

*The flowchart of the steps is in `BioBuzzStarterbotAuto_How_It_Works.pdf` in this folder.*

Each step ends only when its condition holds, so a timer or safety check is always running. In the headless test model, spin-up took about 1.4 s, shooting 4 s, leaving 3.7 s, the turn 0.9 s and parking 1.6 s. A missing launcher or intake skips from INIT straight to LEAVE.

## How the code is organized

It is an iterative OpMode: the SDK calls `loop()` about 50 times a second, and each call does a little of the current step. Nothing waits or blocks, so timers and safety checks keep running.

- **Settings at the top:** route, hardware names, motor directions, shooting and driving values. These are the only lines you should need to change.
- **`ROUTE` list (top of the file):** the steps in the order they run: SPIN_UP, SHOOT, LEAVE, TURN_TO_PARK, TO_PARK, FINAL_TURN. See "Skipping steps for troubleshooting" below.
- **One section per step:** each step has a `begin…()` method that runs once when the step starts (for example `beginLeave()` sets the drive target) and a `run…()` method that `loop()` calls every time until the step is finished (for example `runLeave()`). When a step is finished it calls `nextStep()`, which starts the next step in `ROUTE`, or ends the auto after the last one.
- **`init()`:** calls `initDrive()`, `initShooter()`, `initWindmill()` and `initImu()`, then writes the route to the Events list. Together they find each device by name, set motor directions and brake mode, and reset the encoders. They set up the launcher's velocity control with goBILDA's tuning (PIDF 40, 0, 0, 12.5) and resets the IMU heading. The launcher, windmill and IMU are each wrapped in `try`, so a missing device switches to a fallback instead of crashing. All four motors are loaded as `DcMotorEx` so the telemetry can read their encoder speed; this doesn't change how they drive. Each device's result (found with its settings, or missing with the error) is written to the Events list.
- **`init_loop()`:** shows the ENCODER CHECK section (see "The INIT hand check" below), the Motors section and the INIT events (device status and the route). gamepad1 A restarts the encoder check. There is no alliance choice: red and blue run the same route.
- **`start()`:** resets the auto clock, sets heading 0 to the way the robot faces now, and begins the first step in `ROUTE`. If the launcher hardware is missing, SPIN_UP and SHOOT are skipped, so it starts at LEAVE.
- **`loop()`:** calls the current step's `run…()` method, then `showTelemetry()` (see "Reading the Driver Station screen" below).
- **Driving straight (`startDrive` / `driveDone`):** the code itself stops the robot. The drive motors run in `RUN_USING_ENCODER`, which keeps each wheel at the speed the code asks for. Each loop, `driveDone()` works out how far the robot has moved (the average of the two wheel encoders, in mm) and how far it still has to go:
  - Far from the end: both sides run at `DRIVE_SPEED` (0.5).
  - In the last `SLOW_DOWN_MM` (150 mm): the speed drops in step with the distance left, down to `MIN_DRIVE_POWER` (0.1).
  - Within `POSITION_TOLERANCE_MM` (10 mm) of the distance: power 0 (BRAKE). The robot then has 0.25 s to settle before the next step.
  - To stay straight, the IMU heading error × 0.03 is added to the right side's power and taken from the left side's.
  - If one wheel goes 100 mm past the whole distance (`WHEEL_MISMATCH_MM`), the other wheel's encoder isn't counting: the robot stops and a WARNING is shown.

  The distance still comes from the wheel encoders, so wheel slip or a wrong `WHEEL_DIAMETER_MM` still changes how far the robot really goes.
- **Turning (`startTurn` / `turnDone`):** `startTurn` first sets both drive motors to power 0, so the robot is stopped before the turn starts. Turn power = heading error × 0.02, kept between 0.08 and 0.4; left wheels get `-power`, right wheels `+power`. Headings are measured from the start, so a small error in one turn is fixed by the next.
- **Safety:** every drive and turn has a timeout. Shooting must end by 17 s so parking always gets time. A turn that moves 30° further from its target stops the robot and shows "Turned the WRONG WAY". A launcher encoder reading about 0 switches the launcher to plain 0.85 power. A motor that turns against its command is reported as `WRONG WAY!`; this only reports, it doesn't stop the robot.

## Skipping steps for troubleshooting

To test one part of the auto on its own, comment out steps in the `ROUTE` list at the top of the file. The auto goes straight from the step before to the step after. For example, to test only the drive and the turn, without shooting:

```java
private final State[] ROUTE = {
        // State.SPIN_UP,
        // State.SHOOT,
        State.LEAVE,
        State.TURN_TO_PARK,
        State.TO_PARK,
        State.FINAL_TURN,
};
```

- The Events list shows the route at INIT (`route: LEAVE > TURN_TO_PARK > ...`), so you can check which steps are on before you press START.
- The launcher stays on only from SPIN_UP into SHOOT. If the next step is anything else, it is turned off.
- Keep SPIN_UP before SHOOT. SHOOT feeds only when the launcher is at speed, and SPIN_UP is the step that notices a missing launcher encoder.
- Skipping LEAVE means TURN_TO_PARK turns in place at the start.
- Remove the `//` marks again before a match.

The code also skips steps on its own and writes `skip <step>: <reason>` to Events: SPIN_UP and SHOOT when the launcher or intake is missing, SHOOT when the launcher never reached speed, and FINAL_TURN when the robot already faces the final heading.

## What it needs in the Control Hub configuration

Only the two drive motors are required; every other device has a fallback, but shooting needs all three shooter devices.

| Name | Configuration type | Needed? | If it's missing or wrong |
| --- | --- | --- | --- |
| `left_drive` | Motor (goBILDA 5203, 312 RPM) | Yes, with its encoder cable plugged in | INIT crashes. Without the encoder, the drive steps time out. |
| `right_drive` | Motor (goBILDA 5203, 312 RPM) | Yes, with its encoder cable plugged in | Same as `left_drive` |
| `intake` | Motor | For shooting | If this or `launcher` is missing: no shooting, only LEAVE and PARK |
| `launcher` | Motor (6000 RPM), encoder plugged in | For shooting | Missing: no shooting. No encoder: plain power, so shot speed varies with the battery. |
| `windmill` | Continuous Rotation Servo | For shooting well | Feeds with the intake only, and a warning shows on screen |
| `imu` | Control Hub built-in IMU, added automatically when you scan | Strongly recommended | Falls back to encoder turns, which are less accurate |

- **Ports:** any port works as long as the names match. To match the simulator's Ultron Bot, use motor ports 0 (left), 1 (right), 2 (intake) and 3 (shooter).
- **Not used:** `left_intake_servo` and `right_intake_servo`. They can stay in the configuration.
- **Motor directions:** in this auto, left REVERSE and right REVERSE (the left was changed from FORWARD after the drive cables were re-seated, so both wheels turn the same way). The route values are negative, and negative drives the robot toward the hive (the shooter end).
- **Windmill direction:** REVERSE, so `setPower(1)` feeds the balls, as in goBILDA's code.
- **Intake and launcher directions:** both REVERSE, the same as ShooterAutoTest, ShooterTest and HardwareTest (the shooter tests have been verified on the robot). Without these, the launcher spins backward and the intake pushes balls the wrong way.
- **Hub mounting:** the hub on this robot stands on its edge, with the REV logo facing the robot's left (`HUB_LOGO = LEFT`). `HUB_USB = FORWARD` is unchanged. If the hub is mounted differently, change these two lines; the hand check catches a wrong setting.

## The INIT hand check

After INIT and before START, two hand movements confirm the motor directions, the encoders and the IMU. "Backward" means toward the hive: the direction LEAVE drives (`LEAVE_MM` is negative).

### 1. Encoder check: push the robot backward 1 ft

Press INIT, then push the robot backward 1 ft (305 mm) by hand, toward the hive. The top of the screen shows the ENCODER CHECK section:

| Line | What it shows |
| --- | --- |
| `Left / Right pos` | The raw encoder counts. Both must go NEGATIVE, to about −543 for 1 ft. |
| `Left encoder`, `Right encoder` | mm moved by each wheel since the check started, and a verdict for that wheel |
| `Encoder check` | The overall result |
| `Heading` | The IMU heading, for check 2 |

| Verdict | Meaning | Fix |
| --- | --- | --- |
| `OK: negative (backward)` | This encoder counts the right way | — |
| `WRONG: STAYS AT 0` | The other wheel moved 250 mm or more but this one didn't count | That encoder's cable is unplugged or in the wrong port |
| `WRONG: POSITIVE - flip LEFT_DIRECTION` (or `RIGHT_DIRECTION`) | This motor's direction setting is backward: positive power would drive this side backward | Flip that setting at the top of the code |
| `FAIL - both POSITIVE` | Usually the robot was pushed forward by mistake | Press gamepad1 A and push toward the hive. If it really was backward, flip both settings. |
| `keep pushing` | Not far enough to judge yet | Keep pushing to about 1 ft |
| `PASS - both encoders count backward as negative` | Both encoders are good | — |
| `PASS ... but L and R differ` | Both are negative, but by more than 15% different amounts | A wheel slipped, or the robot turned while you pushed. Press A and push again, straight. |

- **gamepad1 A** starts the check again from where the robot is now (it zeroes the mm, not the encoders).
- The first PASS or FAIL of each kind is also written to Events (`encoder check PASS (L -303 / R -306 mm)`), so it is still there to read after the match.
- While you push, the Motors section should read `turning REV` on both drive lines, with the status `coasting / pushed` (you are moving them, not the code).

### 2. Turn check

Turn the robot counter-clockwise (left) by hand. The heading must go UP, the left count down and the right count up. If the heading goes down, fix `HUB_LOGO` / `HUB_USB`. If the left count goes up, the motors are swapped in the configuration.

## Reading the Driver Station screen

The screen has three parts, top to bottom. It updates every loop from INIT to DONE.

### 1. Live values for the current step

Always shown: the state, the time spent in this step, the auto clock out of 30 s, and the loop time in ms. Under about 50 ms is fine. Below the step's values, the old lines stay: launcher speed, fed seconds, heading / target, encoder positions and targets, and any WARNING.

| Step | What it shows |
| --- | --- |
| `SPIN_UP` | Launcher speed against the target (1210) and the ready level (1160), the steady-at-speed time out of 0.3 s, and the spin-up time out of 4 s. With no launcher encoder: the plain-power countdown instead. |
| `SHOOT` | Feeding ON or PAUSED (launcher below 1160), windmill and intake power, fed seconds out of 4.0 with an estimate of balls fired, the number of feed pauses, and the shoot time against the 10 s and 17 s limits. |
| `LEAVE`, `TO_PARK` | The distance asked for and its direction, how far the robot has moved (average of both wheels) and how far it still has to go, mm driven by each wheel, the heading error and the steering correction it causes, left / right power (− = toward the hive), whether the code has stopped the robot (and how long it has settled, out of 0.25 s), and the time against the timeout. |
| `TURN_TO_PARK`, `FINAL_TURN` | Turn error (and the error at the start of the turn), turn power for each side, the on-target hold time, and the time against the 3 s timeout. Without an IMU: the ticks left for each wheel. |
| `DONE` | The auto time when it finished. |

### 2. Motors

One line per motor: `set | told | turning | status`.

- **set:** the direction setting read from the motor (FORWARD or REVERSE).
- **told:** which way the code is telling it to turn right now: FWD, REV or stop. For the drive motors it comes from the sign of the power. The exception is an encoder turn without an IMU: there the power is always positive, so "told" comes from which side of the target the wheel is on.
- **turning:** which way the encoder says it really turns, with its speed in ticks per second. The SDK applies the direction setting to the encoder too, so "told" and "turning" should always match.

| Status | Meaning |
| --- | --- |
| `OK` | Turning the way it's told, or stopped when told to stop |
| `starting` / `reversing` | Short, normal moments: speeding up, or overshooting a target |
| `coasting / pushed` | Told to stop but still moving: the launcher slowing down, or you pushing the robot in INIT |
| `WRONG WAY!` | Turning against the command for 0.3 s or more. Check that motor's direction setting and its motor and encoder ports. Also written to Events and shown as a WARNING. |
| `NOT TURNING (no encoder? stuck?)` | Told to turn, but no encoder movement for 1 s. Check the encoder cable, or whether the robot is stuck. Written to Events once. The intake shows this if its encoder isn't plugged in, which is harmless. |

The windmill is a servo with no encoder, so its line only shows its setting and the power it's told.

The limits are settings in the Telemetry section at the top of the code: `DIRECTION_MIN_VELOCITY` (50 ticks/s counts as moving), `WRONG_WAY_SECONDS` (0.3) and `NOT_TURNING_SECONDS` (1.0).

### 3. Events

One line each time a method changes something, stamped with the auto time (or `INIT`). While running, the newest 12 show; at DONE the whole run shows (up to 40), so you can read the full story after the match.

| Event | Values shown |
| --- | --- |
| INIT, per device | Found with its directions, PIDF and hub mounting, or missing with the error |
| `START` | The starting encoder counts |
| Step change | From → to, and how long the old step took |
| Launcher ready / never ready / no encoder | Speed and time taken |
| First ball fed | Windmill and intake power |
| `SHOOT` done | Why (all fed, 10 s limit, or 17 s deadline), fed seconds, time taken, number of feed pauses |
| Launcher off | Velocity 0 or power 0 |
| Drive start | mm, ticks, the targets, the heading held, the timeout |
| Drive arrived / timed out | mm each wheel drove, the final heading |
| Turn start | From → to heading, LEFT or RIGHT, the starting error (or the turn arc in ticks without an IMU) |
| Turn done / timed out / wrong way | Final heading and error |
| `FINAL_TURN` skipped, `DONE` | The heading and encoder counts at the end |
| Motor `WRONG WAY` / `NOT TURNING` | Which motor, what it was told, what the encoder read |

## Before the first field run

The route was measured on the field from the blue base. BioBuzzStarterbotAuto.java now holds these values, and red and blue both use them (there is no alliance setting).

| Setting | Value in the code | Measured |
| --- | --- | --- |
| `LEAVE_MM` | -2133.6 | 7 ft (84 in), toward and under the hive |
| `PARK_HEADING_DEG` | 90 | Turn left, on both blue and red |
| `TO_PARK_MM` | -914 | 3 ft, on to the blue tape |

Fine-tune the route with the Test runs sheet in `TeamCode/docs/BioBuzzStarterbotAuto_Route.xlsx`.

- [ ] Test the `windmill` servo in the TestHardware utility; it has never been tested.
- [ ] Run the route three times and log each miss in the Test runs sheet. After each run, read the Events list at DONE and write down any timeout or warning.
- [ ] During the first run, check that every Motors line says `OK` (the intake may say `NOT TURNING` if its encoder isn't plugged in).
- [ ] Copy BioBuzzStarterbotAuto.java into the simulator folder, and delete the old UltronAuto and V2 files there so only one auto shows.
