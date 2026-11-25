# BioBuzzStarterbotOdoAuto: how the Pinpoint odometry and heading work

This explains how `BioBuzzStarterbotOdoAuto.java` gets its position and heading. The route, shooting and telemetry are in `BioBuzzStarterbotAuto_How_It_Works.md` (the two-wheel auto), and the same shooting logic is used here. The original `BioBuzzStarterbotAuto.java` is not changed.

---

## 1. Hardware

| Part | Config name | What it does |
| --- | --- | --- |
| goBILDA **Pinpoint** odometry computer (V2.0) | `pinpoint` | Reads the two pods and its own IMU, and gives x, y and heading |
| Odometry pod on the **front** of the robot | plugged into the Pinpoint's **X** port | Measures forward/back motion |
| Odometry pod on the **left side** of the robot | plugged into the Pinpoint's **Y** port | Measures sideways motion |
| Four mecanum drive motors | `frontLeft`, `backLeft`, `frontRight`, `backRight` | Move the robot. Their encoders are not used. |
| Launcher, intake, windmill | `launcher`, `intake`, `windmill` | Shooting (same as the two-wheel auto) |

The Pinpoint is an **I2C device**, so it goes in an I2C port on the Control Hub, not a motor port. The pods don't go into motor ports at all.

The Pinpoint board has arrows on it: **X points up (forward)** and **Y points left**. Mount it so those arrows match the robot's forward and left. If it is mounted another way, the heading and the x/y directions will be wrong, and the INIT check will show it.

### 1.1 Control Hub configuration

Names must match the Driver Station configuration exactly. Port numbers are the Control Hub's own ports.

| Config name | Device type | Hub port | Notes |
| --- | --- | --- | --- |
| `frontLeft` | DC motor | Motor port 0 | Front-left wheel |
| `frontRight` | DC motor | Motor port 1 | Front-right wheel |
| `backLeft` | DC motor | Motor port 2 | Back-left wheel |
| `backRight` | DC motor | Motor port 3 | Back-right wheel |
| `pinpoint` | goBILDA Pinpoint (I2C) | Any free **I2C** port | The two odometry pods plug into the Pinpoint, not into the hub |
| `launcher` | DC motor | **Not on the Control Hub** (all 4 motor ports are the drive wheels) | Needs an Expansion Hub (see 1.2) |
| `intake` | DC motor | **Not on the Control Hub** | Needs an Expansion Hub (see 1.2) |
| `windmill` | Continuous rotation servo | Any servo port | |

The drive motor names above match the code (`LEFT_FRONT_NAME`, `LEFT_BACK_NAME`, `RIGHT_FRONT_NAME`, `RIGHT_BACK_NAME`). Port numbers follow the REV layout: Port 0 Front-Left, Port 1 Front-Right, Port 2 Back-Left, Port 3 Back-Right. The Driver Station names must be exactly `frontLeft`, `frontRight`, `backLeft` and `backRight`, with the same capitals.

### 1.2 Motor ports: the launcher and intake need an Expansion Hub

The Control Hub has four motor ports, and the four drive wheels use all of them. The launcher and intake are motors too, so they need another hub. Options:

- **Add an Expansion Hub** and plug the launcher and intake into its motor ports. Name them in the Driver Station configuration as the same names (`launcher`, `intake`). The code doesn't change.
- Or move one of the wheels to the Expansion Hub, which is the same change in the other direction.

Until one of these is done, the launcher and intake can't be configured, so SPIN_UP and SHOOT are skipped (the auto logs "launcher/intake missing").

---

## 2. What the Pinpoint gives us

The Pinpoint is a separate computer. Once a loop, the code asks it for the latest pose:

```java
pinpoint.update();                                  // once per loop: reads the pods and the IMU
Pose2D pose = pinpoint.getPosition();
poseXmm = pose.getX(DistanceUnit.MM);               // forward from START (+ = forward)
poseYmm = pose.getY(DistanceUnit.MM);               // left from START (+ = left)
poseHeadingDeg = pose.getHeading(AngleUnit.DEGREES); // + = turned left (counter-clockwise)
```

The Pinpoint combines the two pods and its IMU to produce this pose. The code doesn't do any odometry arithmetic itself. The Pinpoint also handles the pod turning correction, using the offsets we give it (section 3).

**Frame:** x is the field-frame position in the direction the robot faced at START, and y is its left. At START the code calls `pinpoint.setPosition(0, 0, heading 0)`, so "here" becomes (0, 0) and the heading becomes 0. Turning the robot left makes the heading go up.

---

## 3. Setup in INIT

```java
pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
pinpoint.setOffsets(FRONT_POD_SIDE_OFFSET_MM, LEFT_POD_FORWARD_OFFSET_MM, DistanceUnit.MM);
pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
pinpoint.setEncoderDirections(FRONT_POD_DIRECTION, LEFT_POD_DIRECTION);
pinpoint.resetPosAndIMU();
```

- **`setOffsets`** tells the Pinpoint where each pod is relative to the robot's centre of rotation (section 4).
- **`setEncoderResolution(goBILDA_4_BAR_POD)`** sets the tick count for the 32 mm 4-Bar pod, so the Pinpoint converts ticks to mm correctly. The code doesn't need the tick count.
- **`setEncoderDirections`** sets which way each pod counts. Both are `FORWARD` in the file. The INIT check tells you if one needs to be `REVERSED` (section 6).
- **`resetPosAndIMU()`** zeroes the position and recalibrates the Pinpoint's IMU. **The robot must be still** when this runs, during INIT.

At START the code also calls `setPosition(0, 0, 0)`. This only zeroes the position and heading; it doesn't recalibrate the IMU.

---

## 4. The pod offsets

The Pinpoint measures everything from one point: the robot's centre of rotation. A pod away from that point rolls when the robot turns, even though the centre hasn't moved forward. The Pinpoint corrects for this, but it needs to know where the pods are.

The offsets use the same sign convention as the SDK sample:

| Offset | Meaning | Sign | Set in the file |
| --- | --- | --- | --- |
| `FRONT_POD_SIDE_OFFSET_MM` | How far to the **side** of the centre the front pod is | + = left | `0` (placeholder) |
| `LEFT_POD_FORWARD_OFFSET_MM` | How far **forward** of the centre the left pod is | + = front | `0` (placeholder) |

Only these two offsets matter:
- The front pod measures forward motion, so its **sideways** position matters (it rolls when the robot turns if it sits left or right of centre).
- The left pod measures sideways motion, so its **forward** position matters. Its sideways distance from the centre doesn't matter.

Both are placeholders at 0. That is right only if the front pod is centred side-to-side and the left pod is level with the centre. Measure them (section 6).

---

## 5. How the auto uses the pose

### 5.1 Straight drives (LEAVE and TO_PARK)

1. When the step starts, the code records the pose: `driveStartX`, `driveStartY` and the heading `driveStartHeadingRad`.
2. Every loop it computes how far the robot has gone **forward along the heading it started with**:

   ```
   dx = x − driveStartX
   dy = y − driveStartY
   forward = dx × cos(h) + dy × sin(h)      h = heading at the start of the drive
   ```

   This is the robot-frame forward distance, so it works even if the heading has drifted a little. Negative = backward (toward the hive).
3. When `driveMm − forward` is within `POSITION_TOLERANCE_MM` (10 mm), the code sets the power to 0, then waits `ARRIVE_HOLD_SECONDS` (0.25 s) for the robot to settle.
4. Power is reduced over the last `SLOW_DOWN_MM` (150 mm), but never below `MIN_DRIVE_POWER` (0.1).
5. **Steering:** the heading error from the Pinpoint's heading keeps the robot straight:

   ```
   error = target heading − heading now        (wrapped to −180…180)
   correction = clip(error × 0.03, ±0.2)
   left  = base − correction      right = base + correction
   ```

   Positive correction makes the right side faster, which turns the robot left. The correction is clipped to the drive speed so no side reverses.

Safety checks:
- **Timeout** after `moveTimeout` (3 s or distance ÷ 250 mm/s), with a warning.
- **Pinpoint not counting:** after 1.5 s, if the robot has moved less than 5 mm, the drive stops and shows "Pinpoint is not counting".

### 5.2 Turns in place (TURN_TO_PARK and FINAL_TURN)

The heading error from the Pinpoint's heading is used directly:

- Power = `clip(error × 0.02, ±0.4)`, raised to at least 0.08 when the error is outside 2°.
- Left side = −power, right side = +power. Left wheels go back and right wheels go forward, so positive power turns the robot left.
- The turn ends after the robot has been within 2° of the target for 0.25 s, or after 3 s.
- **Wrong-way safety:** if the error grows to more than the starting error + 30°, the robot is turning away from the target. The auto stops and shows "Turned the WRONG WAY". This usually means the Pinpoint is mounted the wrong way, or left and right are swapped.

### 5.3 Pinpoint missing

- **Pinpoint not in the config (or the name is wrong):** LEAVE, TO_PARK, TURN_TO_PARK and FINAL_TURN are skipped with a reason in Events. SPIN_UP and SHOOT still run.

---

## 6. The INIT odometry check

Before START, the Driver Station shows the **ODOMETRY CHECK**. The checked values are the Pinpoint's x and y, measured from when the OpMode was initialised:

1. Push the robot **backward** 1 ft (toward the hive). **x must go negative** (about −305 mm).
   - Positive → "WRONG: POSITIVE - flip FRONT_POD_DIRECTION" (set it to `REVERSED`).
2. Push the robot **left** by hand. **y must go positive**.
   - Negative → "WRONG: NEGATIVE - flip LEFT_POD_DIRECTION" (set it to `REVERSED`).
3. Both OK → **PASS**.

Press **gamepad1 A** to restart the check from where the robot is now.

The "Heading" line shows the Pinpoint heading. Turn the robot left by hand: the heading must go **up**. If it goes down, the Pinpoint is mounted the wrong way.

---

## 7. Calibration (on the robot)

Do these in order.

1. **Distance scale.** Push the robot backward by hand for exactly 1000 mm (tape on the mat). The Driver Station's x should read about −1000 mm. If it reads a fixed percentage off, the pod resolution or wheel size is wrong. The Pinpoint sample describes a custom-resolution option if the goBILDA pod setting isn't correct for your pods.
2. **Straight drive.** Run a route with only LEAVE (comment out the other steps in `ROUTE`). Measure the distance with a tape and compare it with the Driver Station's `Moved (forward)` value. If they differ by a fixed percentage, the distance scale is off (step 1).
3. **Front pod offset.** Turn the robot in place by 90° LEFT (a route with only TURN_TO_PARK, both offsets still 0). Read x on the Driver Station after the turn. The robot did not move forward, so x is the turning error. Then:

   ```
   FRONT_POD_SIDE_OFFSET_MM = FRONT_POD_SIDE_OFFSET_MM + x / (π/2)
   ```

   Use the current value of the constant in the file (0 at first). Enter the new value in the file, then run steps 3–5 again to check it.

4. **Left pod offset.** Same 90° left turn, read y after the turn. Then:

   ```
   LEFT_POD_FORWARD_OFFSET_MM = LEFT_POD_FORWARD_OFFSET_MM − y / (π/2)
   ```

   The minus sign is because a pod in front of the centre rolls right when the robot turns left.
5. **Heading.** Turn the robot 90° left. The heading should read 90 ± 2°. If it turns the wrong way, the Pinpoint is mounted the wrong way.

After changing the offsets, write them in the file and run steps 3–5 again to check.

---

## 8. Data recording (x, y, heading while the robot moves)

The data is recorded in every state, from START until the run ends, including after DONE. One row is written every **0.05 s** (20 rows per second).

| Destination | Where to find it | Notes |
| --- | --- | --- |
| CSV file | `/sdcard/FIRST/odo_yyyyMMdd_HHmmss.csv` on the Control Hub | One file per run. Created at START, closed at STOP. Open it in a spreadsheet. |
| Robot Log | Driver Station **Robot Log** (or `adb logcat`), tag `BioBuzzOdo` | Same rows as the file. Backup if the file can't be opened. |
| Driver Station screen | `Pose` and `Heading` lines | Live only. Not saved. |

If the file can't be opened (for example, `/sdcard/FIRST` isn't writable), Events says "data file NOT opened" and the rows still go to the Robot Log.

**Columns in the CSV**

| Column | Meaning |
| --- | --- |
| `time_s` | Seconds since START |
| `state` | The route step running (SPIN_UP, SHOOT, LEAVE, TURN_TO_PARK, TO_PARK, FINAL_TURN, DONE) |
| `x_mm` | Forward position from START (+ = forward), from the Pinpoint |
| `y_mm` | Left position from START (+ = left), from the Pinpoint |
| `heading_deg` | Heading. + = turned left (counter-clockwise), 0 = facing at START |
| `target_deg` | Heading the drive holds or the turn aims for |
| `drive_mm` | Distance asked for by the current drive (− = toward the hive) |
| `drive_forward_mm` | Forward distance driven so far in the current drive (0 when not driving) |
| `power_left`, `power_right` | Drive power sent to the left and right sides (−1 to +1) |
| `launcher_vel` | Launcher speed in ticks/s |

**How to check the data**

1. Open the CSV. `time_s` should increase by about 0.05 per row, and `state` should change at the right times.
2. During LEAVE, `drive_forward_mm` should fall toward −1,829 mm (7 ft), and `power_left` and `power_right` should drop as the robot slows.
3. During a turn, `heading_deg` should move toward `target_deg`. `x_mm` and `y_mm` should stay near their values from before the turn if the offsets are correct.
4. If `x_mm` and `y_mm` don't change while `power_*` is non-zero, the Pinpoint isn't counting (see section 9).

---

## 9. Things that can go wrong

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| "Pinpoint MISSING" at INIT | Config name isn't `pinpoint`, or it's on the wrong port | Name it `pinpoint` in the Control Hub config, on an I2C port |
| INIT check says WRONG for x or y | That pod's direction is reversed | Set `FRONT_POD_DIRECTION` or `LEFT_POD_DIRECTION` to `REVERSED` |
| Heading goes down when turning left | Pinpoint mounted the wrong way | Mount it with X forward and Y left, as the board arrows show |
| Distance stop too long or too short by a fixed percentage | Distance scale | Section 7, step 1–2 |
| Robot curves on straight drives | Mecanum slip, or a drive motor direction wrong | Check the four drive directions; the heading correction helps but can't fix a wrong motor |
| "Pinpoint is not counting" | Pod cables unplugged, a pod not touching the floor, or the wrong port | Check the cables and that the pods rest on the floor |
| "Turned the WRONG WAY" | Pinpoint mounted backwards, or left and right drive directions swapped | Check the heading line in the INIT check |
| x or y drifts after several turns | Pod offsets are still 0 | Section 7, steps 3–4 |
| Pinpoint heading wrong after START | `resetPosAndIMU()` ran while the robot was moving | Keep the robot still during INIT |

---

## 10. Differences from BioBuzzStarterbotAuto (two-wheel)

| | BioBuzzStarterbotAuto | BioBuzzStarterbotOdoAuto |
| --- | --- | --- |
| Drive motors | `left_drive`, `right_drive` (two) | Four mecanum motors |
| Distance | Average of the two drive encoders | Pinpoint pose (two pods) |
| Heading | Control Hub IMU (`imu`) | Pinpoint's own IMU |
| Sideways position | None | Yes (`y_mm`) |
| Turn correction for distance | None | Done by the Pinpoint using the offsets |
| Shooting, timing, telemetry | — | Same values as the two-wheel auto |

The shooter values must stay in sync with the other OpModes (`CLAUDE.md`).
