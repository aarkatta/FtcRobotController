# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

FTC SDK v12.0 (BIOBUZZ, 2026-2027 season) Android project. Two Gradle modules (`settings.gradle`):
- `FtcRobotController` — stock SDK app plus sample OpModes (`.../robotcontroller/external/samples`). Treat as upstream SDK code; don't edit it.
- `TeamCode` — all team code goes here (`org.firstinspires.ftc.teamcode`). It depends on `FtcRobotController`.

Build config is shared: `build.common.gradle` and `build.dependencies.gradle` (FTC libs pinned at 12.0.0). Keep `TeamCode/build.gradle` minimal so SDK updates merge cleanly.

## Commands

- Build APK: `./gradlew :TeamCode:assembleDebug`
- Install to a connected Control Hub / phone: `./gradlew :TeamCode:installDebug` (or run from Android Studio)
- There is no unit-test or lint setup; OpModes are verified on the robot (or in the team's headless simulator). Requires Android Studio Narwhal 3 Feature Drop or later.

## Architecture

OpModes in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` are discovered by annotation (`@TeleOp` / `@Autonomous`); there is no registration list. All current OpModes are iterative `OpMode` subclasses (`init` / `init_loop` / `start` / `loop`), not `LinearOpMode`. Nothing may block in `loop()`; autonomous is a non-blocking state machine driven by timers and encoder/IMU conditions.

- `BioBuzzStarterbotTeleop` — teleop for the starter bot (Driver Station: "BioBuzz StarterBot Teleop").
- `StarterBotAuto` — goBILDA starter-bot sample auto (reference for motor directions and launcher PIDF). Not for matches: its only team change is the Driver Station name "goBILDA Sample Auto (reference)", so it isn't confused with `BioBuzzStarterbotAuto`.
- `ShooterTest` (TeleOp), `ShooterAutoTest` (Autonomous), `ShooterAccuracyTest` (Autonomous), `HardwareTest` (TeleOp) — `group = "Test"` bench/field tools, not for matches. `HardwareTest` checks each motor/servo one at a time; `ShooterTest` tunes launcher speed with the gamepad; `ShooterAutoTest` is a hands-off PASS/FAIL smoke test of launcher + windmill + intake; `ShooterAccuracyTest` drives to a shot spot, aims by IMU and fires ball by ball to tune `SHOT_MM` / `SHOT_HEADING_DEG` / `SHOT_VELOCITY`. The shooter tests were verified on the robot and are the reference for intake/launcher directions and shooter values.
- `BioBuzzStarterbotAuto` — the competition auto (Driver Station: "BioBuzz StarterBot Auto"; called `UltronAuto` until 2026-11-12, and the simulator may still hold that old file): starts facing the hive, shoots 4 preloaded balls, drives under the hive, turns left, parks on the blue tape. Settings (route, hardware names, motor directions, shoot/drive values) are constants at the top of the file. Route is written in red form and mirrored for blue via `heading()`; alliance chosen in `init_loop()` (B = red, X = blue; default BLUE). Devices other than the drive motors are wrapped in `try` with fallbacks (missing launcher/intake skips to LEAVE; missing IMU falls back to encoder turns). Drive uses `RUN_TO_POSITION` (target, then mode, then power — order matters) with IMU heading correction; every step has a timeout.

Shooter values are copied, not shared: `LAUNCHER_TARGET_VELOCITY` / `LAUNCHER_MIN_VELOCITY` (1250 / 1200 ticks/s; keep MIN about 50 below TARGET), and the launcher PIDF appear separately in `BioBuzzStarterbotAuto`, `StarterBotAuto`, `BioBuzzStarterbotTeleop` and the Shooter tests. When one changes, change all of them. The intake/launcher REVERSE directions are set in `BioBuzzStarterbotAuto`, `BioBuzzStarterbotTeleop` and the Test OpModes; the goBILDA sample `StarterBotAuto` does not set them (leave it as the stock reference). `BioBuzzStarterbotAuto` compares the launcher velocity signed (not `abs`) so a launcher spinning the wrong way is reported instead of fed.

Shared robot hardware config names: `left_drive`, `right_drive` (left FORWARD, right REVERSE), `intake` (REVERSE), `launcher` (DcMotorEx, REVERSE, velocity PIDF 40/0/0/12.5), `windmill` (CRServo, REVERSE), `imu`. Names in code must match the Control Hub configuration exactly.

`TeamCode/docs/BioBuzzStarterbotAuto_How_It_Works.md` (also .pdf/.docx, plus `BioBuzzStarterbotAuto_Route.xlsx`) documents BioBuzzStarterbotAuto's steps, hub config, INIT hand-check, telemetry screen, and measured route values; update it when changing BioBuzzStarterbotAuto behavior. The `.md` is the source: after editing it, run `TeamCode/docs/BioBuzzStarterbotAuto_build_docs.sh` to regenerate the `.docx` and `.pdf` (flowchart source: `BioBuzzStarterbotAuto_flowchart.html`). BioBuzzStarterbotAuto is also run in the team's separate "techtitans" simulator, which lives outside this repo: after changing `BioBuzzStarterbotAuto.java`, copy it into the simulator folder (its "Ultron Bot" config must use the same device names).
