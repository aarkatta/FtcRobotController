package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * BioBuzzStarterbotAuto - the team's competition autonomous for the BioBuzz starter bot
 * (Driver Station: "BioBuzz StarterBot Auto"). Its teleop partner is BioBuzzStarterbotTeleop.
 * Called UltronAuto until Nov 12, 2026. Not to be confused with StarterBotAuto, goBILDA's sample.
 *
 * Plan:
 *   1. SPIN_UP      launcher up to speed and steady
 *   2. SHOOT        windmill feeds the preloaded balls, ONLY while the launcher is at speed
 *   3. LEAVE        drive out of the start zone ..................... LEAVE (+3)
 *   4. TURN_TO_PARK turn to face the parking spot (IMU)
 *   5. TO_PARK      drive into the parking spot ..................... PARK  (+5)
 *   6. FINAL_TURN   optional: turn to the heading you want to finish at, then DONE
 *
 * Telemetry (Driver Station):
 *   - top:    state, timers, and the live values of the CURRENT step (speeds, distances, errors, powers)
 *   - middle: "Motors" - for each motor: its direction SETTING (FORWARD/REVERSE), which way the code TELLS
 *             it to turn, and which way its encoder says it REALLY turns. "WRONG WAY!" = the motor turns
 *             against the command (bad direction setting, swapped ports/cables). Windmill: no encoder.
 *   - bottom: "Events" - one line per method call that changes something (start of a drive/turn, a step
 *             finishing and why, launcher on/off, ...), each with the auto time and its values.
 */
@Autonomous(name = "BioBuzz StarterBot Auto", group = "StarterBot")
public class BioBuzzStarterbotAuto extends OpMode {

    final double LEAVE_MM = -2540;          // 1) straight toward the hive and under it: 8 ft 4 in = 100 in = 2540 mm
    final double PARK_HEADING_DEG = -90;    // 2) quarter turn: RED turns right, BLUE turns left
    final double TO_PARK_MM = -914;         // 3) straight on to the parking spot (blue tape): 3 ft = 914 mm
    final double FINAL_HEADING_DEG = -90;   // 4) finish at this heading (same as step 2 = no extra turn)
    final boolean MIRROR_FOR_BLUE = true;

    // ======= Configuration names (goBILDA StarterBot names; must match the Driver Station config) =======
    final String LEFT_DRIVE_NAME = "left_drive";
    final String RIGHT_DRIVE_NAME = "right_drive";
    final String INTAKE_NAME = "intake";
    final String LAUNCHER_NAME = "launcher";
    final String WINDMILL_NAME = "windmill";
    final String IMU_NAME = "imu";

    // ================= Directions (goBILDA StarterBot values) =================
    final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction WINDMILL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    // Same as ShooterAutoTest / ShooterTest / HardwareTest (verified on the robot). If either spins the
    // wrong way, flip it here AND in those test files so they stay in sync.
    final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.REVERSE;
    // How the Control Hub is mounted (printed logo side, and which way the USB ports point).
    final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ================= Shooting =================
    final int BALLS_TO_SHOOT = 4;               // preloaded balls
    final double FEED_SECONDS_PER_BALL = 1.0;   // seconds of feeding (at speed) per ball
    final double FEED_INTAKE_POWER = 0.5;       // intake helps push (goBILDA adds 0.5 while launching)
    final int LAUNCHER_TARGET_VELOCITY = 1250;  // ticks/sec (goBILDA)
    final int LAUNCHER_MIN_VELOCITY = 1200;     // only feed at/above this
    final double READY_HOLD_SECONDS = 0.3;      // must stay at speed this long before the first ball
    final double SPIN_UP_TIMEOUT = 4.0;         // never got to speed: skip shooting, go score LEAVE+PARK
    final double SHOOT_MAX_SECONDS = 10.0;      // never spend longer than this shooting
    final double SHOOT_DEADLINE_SECONDS = 17.0; // auto clock: stop shooting by here, go park
    // No encoder on the launcher -> velocity reads ~0 while the motor races at full power.
    final double NO_ENCODER_CHECK_SECONDS = 1.5;
    final double NO_ENCODER_VELOCITY = 50;
    final double LAUNCHER_FALLBACK_POWER = 0.85;
    final double OPEN_LOOP_SPIN_UP_SECONDS = 2.5;

    // ================= Driving =================
    final double ENCODER_TICKS_PER_REV = 537.7;    // goBILDA 312 RPM
    final double WHEEL_DIAMETER_MM = 96;
    final double TICKS_PER_MM = ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    final double TRACK_WIDTH_MM = 402;             // only used if the IMU is missing
    final double DRIVE_SPEED = 0.5;
    final double P_DRIVE_GAIN = 0.03;              // heading correction while driving straight
    final double POSITION_TOLERANCE_MM = 10;
    final double MAX_TURN_SPEED = 0.4;
    final double MIN_TURN_POWER = 0.08;            // enough power to actually move on the mat
    final double P_TURN_GAIN = 0.02;
    final double HEADING_TOLERANCE_DEG = 2.0;
    final double ARRIVE_HOLD_SECONDS = 0.25;       // stay on target this long before the next step
    final double TURN_TIMEOUT_SECONDS = 3.0;

    // ================= Telemetry =================
    final int EVENT_LINES_SHOWN = 12;              // events shown while running (all of them once DONE)
    final int EVENT_LINES_KEPT = 40;
    // Motor direction check: below this encoder speed a motor counts as "not turning".
    final double DIRECTION_MIN_VELOCITY = 50;      // ticks/sec
    final double WRONG_WAY_SECONDS = 0.3;          // turning against the command this long = WRONG WAY
    final double NOT_TURNING_SECONDS = 1.0;        // told to turn but no encoder motion this long

    private enum State { SPIN_UP, SHOOT, LEAVE, TURN_TO_PARK, TO_PARK, FINAL_TURN, DONE }
    private enum Alliance { RED, BLUE }

    // DcMotorEx (not plain DcMotor) so the telemetry can read each motor's encoder speed and sign.
    private DcMotorEx leftDrive;
    private DcMotorEx rightDrive;
    private DcMotorEx intake;
    private DcMotorEx launcher;
    private CRServo windmill;
    private IMU imu;
    private boolean shooterOk = false;
    private boolean windmillOk = false;
    private boolean imuOk = false;
    private boolean openLoop = false;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime arriveTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private State state = State.DONE;
    private Alliance alliance = Alliance.BLUE;   // default = the measured route; press B in INIT for RED
    private double fedSeconds = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    private double driveDirection = 1;    // +1 forward, -1 backward
    private double targetHeading = 0;     // degrees, the heading we drive along / turn to
    private double turnStartError = 0;
    private double moveTimeout = 4.0;
    private String warning = "";

    // ---- Telemetry only: values the helpers remember so loop() can show them ----
    private final String[] events = new String[EVENT_LINES_KEPT];
    private int eventCount = 0;
    private boolean started = false;
    private boolean firstState = true;
    private double lastHeading = 0;        // last IMU yaw read, degrees
    private double driveMm = 0;            // distance asked for by the current drive
    private int leftStart = 0;             // encoder counts when the current drive/turn started
    private int rightStart = 0;
    private double lastDriveError = 0;     // heading error while driving, degrees
    private double lastCorrection = 0;     // heading correction while driving
    private double lastLeftPower = 0;
    private double lastRightPower = 0;
    private double lastTurnError = 0;
    private double lastTurnPower = 0;
    private boolean lastInTol = false;
    private boolean feedOn = false;
    private int feedPauses = 0;            // times feeding paused to let the launcher recover
    private double finishTime = 0;
    private final DirectionCheck leftCheck = new DirectionCheck(LEFT_DRIVE_NAME);
    private final DirectionCheck rightCheck = new DirectionCheck(RIGHT_DRIVE_NAME);
    private final DirectionCheck launcherCheck = new DirectionCheck(LAUNCHER_NAME);
    private final DirectionCheck intakeCheck = new DirectionCheck(INTAKE_NAME);

    // ======================================================================================
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotorEx.class, LEFT_DRIVE_NAME);
        rightDrive = hardwareMap.get(DcMotorEx.class, RIGHT_DRIVE_NAME);
        leftDrive.setDirection(LEFT_DIRECTION);
        rightDrive.setDirection(RIGHT_DIRECTION);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        log("init drive: %s %s / %s %s, encoders reset, BRAKE",
                LEFT_DRIVE_NAME, LEFT_DIRECTION, RIGHT_DRIVE_NAME, RIGHT_DIRECTION);

        // Missing shooter hardware: still score LEAVE + PARK.
        try {
            intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
            launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER_NAME);
            intake.setDirection(INTAKE_DIRECTION);
            launcher.setDirection(LAUNCHER_DIRECTION);
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
            shooterOk = true;
            log("init shooter OK: intake %s, launcher %s, PIDF 40/0/0/12.5",
                    INTAKE_DIRECTION, LAUNCHER_DIRECTION);
        } catch (Exception e) {
            shooterOk = false;
            log("init shooter MISSING: %s", e.getMessage());
        }

        // Missing windmill: fall back to feeding with the intake only (works poorly - fix the config).
        try {
            windmill = hardwareMap.get(CRServo.class, WINDMILL_NAME);
            windmill.setDirection(WINDMILL_DIRECTION);
            windmill.setPower(0);
            windmillOk = true;
            log("init windmill OK: %s", WINDMILL_DIRECTION);
        } catch (Exception e) {
            windmillOk = false;
            log("init windmill MISSING: %s", e.getMessage());
        }

        // Missing IMU: fall back to encoder turns (less accurate).
        try {
            imu = hardwareMap.get(IMU.class, IMU_NAME);
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw();
            imuOk = true;
            log("init IMU OK: logo %s, USB %s, yaw reset", HUB_LOGO, HUB_USB);
        } catch (Exception e) {
            imuOk = false;
            log("init IMU MISSING: %s", e.getMessage());
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.x) alliance = Alliance.BLUE;

        telemetry.addData("Alliance", "%s   (gamepad1: B = RED, X = BLUE)", alliance);
        telemetry.addData("Route", "leave %.0f mm, turn to %.0f deg, park %.0f mm",
                LEAVE_MM, heading(PARK_HEADING_DEG), TO_PARK_MM);
        telemetry.addData("Route ticks", "leave %d, park %d   (%.3f ticks/mm)",
                Math.round(LEAVE_MM * TICKS_PER_MM), Math.round(TO_PARK_MM * TICKS_PER_MM), TICKS_PER_MM);
        telemetry.addData("Final heading", "%.0f deg%s", heading(FINAL_HEADING_DEG),
                heading(FINAL_HEADING_DEG) == heading(PARK_HEADING_DEG) ? "  (same: FINAL_TURN skipped)" : "");
        telemetry.addData("Shooter", "%d balls x %.1f s, target %d, feed at >= %d ticks/s",
                BALLS_TO_SHOOT, FEED_SECONDS_PER_BALL, LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Drive", "speed %.2f, tol %.0f mm, turn max %.2f, tol %.1f deg",
                DRIVE_SPEED, POSITION_TOLERANCE_MM, MAX_TURN_SPEED, HEADING_TOLERANCE_DEG);
        telemetry.addData("Launcher/intake", shooterOk ? "OK" : "MISSING - will NOT shoot");
        telemetry.addData("Windmill", windmillOk ? "OK" : "MISSING - intake-only feeding");
        telemetry.addData("IMU", imuOk ? "OK" : "MISSING - encoder turns");
        telemetry.addLine();
        telemetry.addLine("HAND CHECK: wheels forward -> both counts UP");
        telemetry.addLine("turn robot LEFT -> heading UP, left DOWN, right UP");
        telemetry.addData("Heading", "%.1f deg", imuOk ? currentHeading() : 0.0);
        telemetry.addData("Left / Right counts", "%d / %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        showMotors();
        showEvents(EVENT_LINES_KEPT);
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();
        started = true;
        if (imuOk) imu.resetYaw();   // heading 0 = the way the robot faces right now
        targetHeading = 0;
        log("START %s: yaw reset, heading 0, counts L %d / R %d", alliance,
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        if (shooterOk) {
            enter(State.SPIN_UP);
        } else {
            log("no shooter: skip to LEAVE");
            beginLeave();
        }
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        switch (state) {
            case SPIN_UP: {
                feed(false);
                if (openLoop) {
                    launcher.setPower(LAUNCHER_FALLBACK_POWER);
                    if (stateTimer.seconds() >= OPEN_LOOP_SPIN_UP_SECONDS) {
                        log("open loop spin-up done: power %.2f for %.1f s",
                                LAUNCHER_FALLBACK_POWER, stateTimer.seconds());
                        enter(State.SHOOT);
                    }
                    break;
                }
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                double vel = launcher.getVelocity();
                // Signed on purpose: a launcher spinning the WRONG way reads negative and never "reaches
                // speed", so it shows the "never reached speed" warning instead of feeding balls into it.
                if (vel < LAUNCHER_MIN_VELOCITY) {
                    readyTimer.reset();   // not at speed yet: restart the steady-hold timer
                }
                if (stateTimer.seconds() > NO_ENCODER_CHECK_SECONDS
                        && Math.abs(vel) < NO_ENCODER_VELOCITY) {
                    openLoop = true;      // no encoder feedback: switch to plain power
                    warning = "Launcher encoder reads 0 - check its cable. Using plain power.";
                    log("NO launcher encoder: |vel| %.0f < %.0f after %.1f s -> power %.2f",
                            Math.abs(vel), NO_ENCODER_VELOCITY, stateTimer.seconds(), LAUNCHER_FALLBACK_POWER);
                    launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    launcher.setPower(LAUNCHER_FALLBACK_POWER);
                    stateTimer.reset();
                    break;
                }
                if (readyTimer.seconds() >= READY_HOLD_SECONDS) {
                    log("launcher ready: %.0f ticks/s, held %.2f s, spin-up %.1f s",
                            vel, readyTimer.seconds(), stateTimer.seconds());
                    enter(State.SHOOT);
                } else if (stateTimer.seconds() > SPIN_UP_TIMEOUT) {
                    warning = "Launcher never reached speed - skipped shooting.";
                    log("launcher NEVER ready: %.0f < %d after %.1f s -> skip shooting",
                            vel, LAUNCHER_MIN_VELOCITY, stateTimer.seconds());
                    stopLauncher();
                    beginLeave();
                }
                break;
            }

            case SHOOT: {
                if (openLoop) {
                    launcher.setPower(LAUNCHER_FALLBACK_POWER);
                } else {
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                }
                // Feed only while at speed; pause to let the launcher recover between balls.
                if (openLoop || launcher.getVelocity() >= LAUNCHER_MIN_VELOCITY) {
                    feed(true);
                    fedSeconds += dt;
                } else {
                    if (feedOn) feedPauses++;
                    feed(false);
                }
                double feedNeeded = BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL;
                boolean allFed = fedSeconds >= feedNeeded;
                boolean tooLong = stateTimer.seconds() > SHOOT_MAX_SECONDS;
                boolean deadline = autoTimer.seconds() > SHOOT_DEADLINE_SECONDS;
                if (allFed || tooLong || deadline) {
                    log("SHOOT done (%s): fed %.1f / %.1f s in %.1f s, %d pauses",
                            allFed ? "all fed" : tooLong ? "SHOOT_MAX_SECONDS" : "deadline",
                            fedSeconds, feedNeeded, stateTimer.seconds(), feedPauses);
                    stopLauncher();
                    beginLeave();
                }
                break;
            }

            case LEAVE:
                if (driveDone()) {
                    startTurn(heading(PARK_HEADING_DEG));
                    enter(State.TURN_TO_PARK);
                }
                break;

            case TURN_TO_PARK:
                if (turnDone()) {
                    startDrive(TO_PARK_MM);
                    enter(State.TO_PARK);
                }
                break;

            case TO_PARK:
                if (driveDone()) {
                    if (Math.abs(heading(FINAL_HEADING_DEG) - targetHeading) > 0.5) {
                        startTurn(heading(FINAL_HEADING_DEG));
                        enter(State.FINAL_TURN);
                    } else {
                        log("FINAL_TURN skipped: already at %.0f deg", targetHeading);
                        finish();
                    }
                }
                break;

            case FINAL_TURN:
                if (turnDone()) finish();
                break;

            case DONE:
                break;
        }

        // ---------------- Telemetry ----------------
        telemetry.addData("State", "%s   (%s)   %.1f s in step", state, alliance, stateTimer.seconds());
        telemetry.addData("Auto time", "%.1f / 30 s   (loop %.0f ms)", autoTimer.seconds(), dt * 1000);
        double launcherVel = shooterOk ? launcher.getVelocity() : 0;
        double headingNow = imuOk ? currentHeading() : 0.0;

        switch (state) {
            case SPIN_UP:
                if (openLoop) {
                    telemetry.addData("Spin-up", "OPEN LOOP power %.2f, %.1f / %.1f s",
                            LAUNCHER_FALLBACK_POWER, stateTimer.seconds(), OPEN_LOOP_SPIN_UP_SECONDS);
                } else {
                    telemetry.addData("Launcher", "%.0f ticks/s  (target %d, ready at >= %d)",
                            launcherVel, LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
                    telemetry.addData("Steady at speed", "%.2f / %.2f s",
                            readyTimer.seconds(), READY_HOLD_SECONDS);
                    telemetry.addData("Spin-up time", "%.1f / %.1f s  (encoder check at %.1f s)",
                            stateTimer.seconds(), SPIN_UP_TIMEOUT, NO_ENCODER_CHECK_SECONDS);
                }
                break;

            case SHOOT:
                telemetry.addData("Feeding", "%s   windmill %.1f, intake %.1f",
                        feedOn ? "ON" : "PAUSED (launcher < " + LAUNCHER_MIN_VELOCITY + ")",
                        feedOn && windmillOk ? 1.0 : 0.0, feedOn ? FEED_INTAKE_POWER : 0.0);
                telemetry.addData("Fed seconds", "%.2f / %.1f   (~%d of %d balls)", fedSeconds,
                        BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL,
                        Math.min(BALLS_TO_SHOOT, (int) (fedSeconds / FEED_SECONDS_PER_BALL)), BALLS_TO_SHOOT);
                telemetry.addData("Feed pauses", "%d", feedPauses);
                telemetry.addData("Shoot limits", "step %.1f / %.0f s, auto %.1f / %.0f s",
                        stateTimer.seconds(), SHOOT_MAX_SECONDS, autoTimer.seconds(), SHOOT_DEADLINE_SECONDS);
                break;

            case LEAVE:
            case TO_PARK:
                telemetry.addData("Drive", "%.0f mm %s  (%d ticks)", driveMm,
                        driveDirection < 0 ? "BACKWARD" : "FORWARD", Math.round(driveMm * TICKS_PER_MM));
                telemetry.addData("Driven L / R", "%.0f / %.0f mm",
                        (leftDrive.getCurrentPosition() - leftStart) / TICKS_PER_MM,
                        (rightDrive.getCurrentPosition() - rightStart) / TICKS_PER_MM);
                telemetry.addData("Remaining L / R", "%.0f / %.0f mm",
                        (leftTarget - leftDrive.getCurrentPosition()) / TICKS_PER_MM,
                        (rightTarget - rightDrive.getCurrentPosition()) / TICKS_PER_MM);
                telemetry.addData("Steer", "heading error %.1f deg -> correction %+.3f",
                        lastDriveError, lastCorrection);
                telemetry.addData("Power L / R", "%.2f / %.2f", lastLeftPower, lastRightPower);
                telemetry.addData("On target", "%s, held %.2f / %.2f s   timeout %.1f / %.1f s",
                        lastInTol ? "YES" : "no", lastInTol ? arriveTimer.seconds() : 0.0,
                        ARRIVE_HOLD_SECONDS, stateTimer.seconds(), moveTimeout);
                break;

            case TURN_TO_PARK:
            case FINAL_TURN:
                if (imuOk) {
                    telemetry.addData("Turn error", "%.1f deg   (start %.1f, tol %.1f)",
                            lastTurnError, turnStartError, HEADING_TOLERANCE_DEG);
                    telemetry.addData("Turn power", "%+.2f   (L %.2f / R %.2f)",
                            lastTurnPower, lastLeftPower, lastRightPower);
                } else {
                    telemetry.addData("Encoder turn", "remaining L %d / R %d ticks",
                            leftTarget - leftDrive.getCurrentPosition(),
                            rightTarget - rightDrive.getCurrentPosition());
                }
                telemetry.addData("On target", "%s, held %.2f / %.2f s   timeout %.1f / %.1f s",
                        lastInTol ? "YES" : "no", lastInTol ? arriveTimer.seconds() : 0.0,
                        ARRIVE_HOLD_SECONDS, stateTimer.seconds(), TURN_TIMEOUT_SECONDS);
                break;

            case DONE:
                telemetry.addData("Finished", "at %.1f s of auto, motors off", finishTime);
                break;
        }

        if (shooterOk) {
            telemetry.addData("Launcher vel", "%.0f / %d%s", launcherVel,
                    LAUNCHER_TARGET_VELOCITY, openLoop ? "  (NO ENCODER)" : "");
            telemetry.addData("Fed seconds", "%.1f / %.1f", fedSeconds,
                    BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL);
        }
        telemetry.addData("Heading / target", "%.1f / %.1f deg", headingNow, targetHeading);
        telemetry.addData("Left / Right pos", "%d / %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Left / Right target", "%d / %d", leftTarget, rightTarget);
        if (warning.length() > 0) telemetry.addData("WARNING", warning);
        showMotors();
        showEvents(state == State.DONE ? EVENT_LINES_KEPT : EVENT_LINES_SHOWN);
        telemetry.update();
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        stopLauncher();
        log("STOP: all motors off");
    }

    // ======================================================================================
    // Telemetry helpers
    // ======================================================================================

    /** Adds one line to the event list, stamped with the auto time (or INIT before start). */
    private void log(String format, Object... args) {
        String time = started ? String.format("%5.1fs", autoTimer.seconds()) : " INIT ";
        String line = time + "  " + String.format(format, args);
        if (eventCount < EVENT_LINES_KEPT) {
            events[eventCount++] = line;
        } else {   // full: drop the oldest
            System.arraycopy(events, 1, events, 0, EVENT_LINES_KEPT - 1);
            events[EVENT_LINES_KEPT - 1] = line;
        }
    }

    /** One line per motor: direction SETTING | which way it is TOLD to turn | which way it REALLY turns. */
    private void showMotors() {
        telemetry.addLine("---- Motors (set | told | turning) ----");
        motorLine("Left drive", leftDrive, driveCommand(leftDrive, leftTarget), leftCheck);
        motorLine("Right drive", rightDrive, driveCommand(rightDrive, rightTarget), rightCheck);
        if (shooterOk) {
            // The launcher is only ever told to spin forward, and only in SPIN_UP / SHOOT.
            boolean launcherOn = state == State.SPIN_UP || state == State.SHOOT;
            motorLine("Launcher", launcher, launcherOn ? 1 : 0, launcherCheck);
            motorLine("Intake", intake, Math.signum(intake.getPower()), intakeCheck);
        }
        if (windmillOk) {
            double power = windmill.getPower();
            telemetry.addData("Windmill", "set %s | told %s %+.2f | no encoder",
                    windmill.getDirection(), dirWord(power), power);
        }
    }

    private void motorLine(String label, DcMotorEx motor, double told, DirectionCheck check) {
        double vel = motor.getVelocity();   // signed: + = the motor's forward (after its direction setting)
        String status = check.update(told, vel);
        telemetry.addData(label, "set %s | told %s | turning %s %+.0f t/s | %s",
                motor.getDirection(), dirWord(told),
                dirWord(Math.abs(vel) > DIRECTION_MIN_VELOCITY ? vel : 0), vel, status);
    }

    /** Which way a drive motor is being told to turn: +1 forward, -1 reverse, 0 stop. */
    private double driveCommand(DcMotorEx motor, int target) {
        double power = motor.getPower();
        if (power == 0) return 0;
        if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            // In RUN_TO_POSITION the power is only a speed limit; the direction comes from the target.
            int remaining = target - motor.getCurrentPosition();
            return Math.abs(remaining) <= POSITION_TOLERANCE_MM * TICKS_PER_MM ? 0 : Math.signum(remaining);
        }
        return Math.signum(power);
    }

    private String dirWord(double sign) {
        return sign > 0 ? "FWD" : sign < 0 ? "REV" : "stop";
    }

    /**
     * Compares which way a motor is TOLD to turn with which way its encoder says it turns.
     * Short differences are normal (starting up, overshooting a target), so a problem is only
     * reported when it lasts WRONG_WAY_SECONDS / NOT_TURNING_SECONDS. Each problem is logged once.
     */
    private class DirectionCheck {
        private final String name;
        private final ElapsedTime wrongWayTimer = new ElapsedTime();
        private final ElapsedTime stillTimer = new ElapsedTime();
        private boolean loggedWrongWay = false;
        private boolean loggedStill = false;

        DirectionCheck(String name) {
            this.name = name;
        }

        /** told: +1 forward, -1 reverse, 0 stop. vel: encoder ticks/sec. Returns a short status. */
        String update(double told, double vel) {
            boolean moving = Math.abs(vel) > DIRECTION_MIN_VELOCITY;
            if (told == 0) {
                wrongWayTimer.reset();
                stillTimer.reset();
                return moving ? "coasting / pushed" : "OK";
            }
            if (!moving) {
                wrongWayTimer.reset();
                if (stillTimer.seconds() < NOT_TURNING_SECONDS) return "starting";
                if (!loggedStill) {
                    loggedStill = true;
                    log("%s NOT TURNING: told %s, encoder %.0f t/s for %.1f s (no encoder? stuck?)",
                            name, dirWord(told), vel, stillTimer.seconds());
                }
                return "NOT TURNING (no encoder? stuck?)";
            }
            stillTimer.reset();
            if (Math.signum(vel) == told) {
                wrongWayTimer.reset();
                return "OK";
            }
            if (wrongWayTimer.seconds() < WRONG_WAY_SECONDS) return "reversing";
            if (!loggedWrongWay) {
                loggedWrongWay = true;
                log("%s WRONG WAY: told %s, turning %s %.0f t/s", name, dirWord(told), dirWord(vel), vel);
                if (warning.length() == 0) {
                    warning = name + " turns the WRONG WAY - check its direction setting and ports.";
                }
            }
            return "WRONG WAY!";
        }
    }

    /** Shows the newest 'lines' events, oldest first. */
    private void showEvents(int lines) {
        telemetry.addLine("---- Events ----");
        for (int i = Math.max(0, eventCount - lines); i < eventCount; i++) {
            telemetry.addLine(events[i]);
        }
    }

    // ======================================================================================
    // Helpers
    // ======================================================================================

    private void enter(State next) {
        if (firstState) {
            log("-> %s", next);
            firstState = false;
        } else {
            log("%s -> %s  (%.1f s in %s)", state, next, stateTimer.seconds(), state);
        }
        state = next;
        stateTimer.reset();
        arriveTimer.reset();
        readyTimer.reset();
        loopTimer.reset();
    }

    /** RED route heading -> heading for the selected alliance. */
    private double heading(double redHeadingDeg) {
        return (alliance == Alliance.BLUE && MIRROR_FOR_BLUE) ? -redHeadingDeg : redHeadingDeg;
    }

    private void feed(boolean on) {
        if (!shooterOk) return;
        if (on && !feedOn && fedSeconds == 0) {
            log("first feed: windmill %.1f, intake %.1f",
                    windmillOk ? 1.0 : 0.0, FEED_INTAKE_POWER);
        }
        feedOn = on;
        if (windmillOk) {
            windmill.setPower(on ? 1.0 : 0.0);
            intake.setPower(on ? FEED_INTAKE_POWER : 0.0);
        } else {
            intake.setPower(on ? FEED_INTAKE_POWER : 0.0);
        }
    }

    private void stopLauncher() {
        if (!shooterOk) return;
        feed(false);
        if (openLoop) {
            launcher.setPower(0);
        } else {
            launcher.setVelocity(0);
        }
        log("launcher + feed off (%s)", openLoop ? "power 0" : "velocity 0");
    }

    private void beginLeave() {
        startDrive(LEAVE_MM);
        enter(State.LEAVE);
    }

    private void finish() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        finishTime = autoTimer.seconds();
        log("DONE: heading %.1f deg, counts L %d / R %d",
                imuOk ? currentHeading() : 0.0,
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        enter(State.DONE);
    }

    // ---------------- Heading ----------------

    private double currentHeading() {
        lastHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return lastHeading;
    }

    /** How far (degrees) the robot must still turn to reach 'target'. + = turn LEFT. */
    private double headingError(double target) {
        double error = target - currentHeading();
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    // ---------------- Straight drive (encoders for distance, IMU to stay straight) ----------------

    private void startDrive(double mm) {
        int ticks = (int) Math.round(mm * TICKS_PER_MM);
        leftStart = leftDrive.getCurrentPosition();
        rightStart = rightDrive.getCurrentPosition();
        leftTarget = leftStart + ticks;
        rightTarget = rightStart + ticks;
        driveMm = mm;
        driveDirection = mm < 0 ? -1 : 1;
        moveTimeout = Math.max(3.0, Math.abs(mm) / 250.0);   // generous: ~250 mm per second
        // SDK order: target -> RUN_TO_POSITION -> power
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(DRIVE_SPEED);
        rightDrive.setPower(DRIVE_SPEED);
        lastLeftPower = DRIVE_SPEED;
        lastRightPower = DRIVE_SPEED;
        lastCorrection = 0;
        lastInTol = false;
        arriveTimer.reset();
        log("drive %.0f mm = %d ticks: target L %d / R %d, hold %.1f deg, timeout %.1f s",
                mm, ticks, leftTarget, rightTarget, targetHeading, moveTimeout);
    }

    /** Call every loop while driving. True once both wheels have arrived (or the move timed out). */
    private boolean driveDone() {
        // Steer back onto targetHeading. In RUN_TO_POSITION the power is a speed limit per side.
        double correction = 0;
        lastDriveError = 0;
        if (imuOk) {
            lastDriveError = headingError(targetHeading);
            correction = Range.clip(lastDriveError * P_DRIVE_GAIN, -0.2, 0.2);
            if (driveDirection < 0) correction = -correction;
        }
        lastCorrection = correction;
        lastLeftPower = Range.clip(DRIVE_SPEED - correction, 0.05, 1.0);
        lastRightPower = Range.clip(DRIVE_SPEED + correction, 0.05, 1.0);
        leftDrive.setPower(lastLeftPower);
        rightDrive.setPower(lastRightPower);

        double tolTicks = POSITION_TOLERANCE_MM * TICKS_PER_MM;
        int leftPos = leftDrive.getCurrentPosition();
        int rightPos = rightDrive.getCurrentPosition();
        boolean inTol = Math.abs(leftTarget - leftPos) <= tolTicks
                && Math.abs(rightTarget - rightPos) <= tolTicks;
        lastInTol = inTol;
        if (!inTol) arriveTimer.reset();
        if (stateTimer.seconds() > moveTimeout) {
            warning = state + " timed out - check drive encoders, or the robot hit something.";
            log("%s TIMEOUT %.1f s: drove L %.0f / R %.0f of %.0f mm, heading %.1f",
                    state, stateTimer.seconds(), (leftPos - leftStart) / TICKS_PER_MM,
                    (rightPos - rightStart) / TICKS_PER_MM, driveMm, lastHeading);
            return true;
        }
        boolean done = arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
        if (done) {
            log("%s arrived %.1f s: drove L %.0f / R %.0f mm, heading %.1f",
                    state, stateTimer.seconds(), (leftPos - leftStart) / TICKS_PER_MM,
                    (rightPos - rightStart) / TICKS_PER_MM, lastHeading);
        }
        return done;
    }

    // ---------------- Turn in place (IMU) ----------------

    private void startTurn(double headingDeg) {
        double turnDeg = headingDeg - targetHeading;   // only used without an IMU
        double fromDeg = targetHeading;
        targetHeading = headingDeg;
        leftStart = leftDrive.getCurrentPosition();
        rightStart = rightDrive.getCurrentPosition();
        lastInTol = false;
        if (imuOk) {
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turnStartError = Math.abs(headingError(targetHeading));
            log("turn %.0f -> %.0f deg (%+.0f, %s) by IMU: now %.1f, error %.1f",
                    fromDeg, headingDeg, turnDeg, turnDeg > 0 ? "LEFT" : "RIGHT",
                    lastHeading, turnStartError);
        } else {
            // No IMU: old encoder turn. Each wheel travels (track/2) x angle, in opposite directions.
            double arcMm = Math.toRadians(turnDeg) * (TRACK_WIDTH_MM / 2.0);
            int arcTicks = (int) Math.round(arcMm * TICKS_PER_MM);
            leftTarget = leftDrive.getCurrentPosition() - arcTicks;
            rightTarget = rightDrive.getCurrentPosition() + arcTicks;
            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setPower(MAX_TURN_SPEED);
            rightDrive.setPower(MAX_TURN_SPEED);
            lastLeftPower = MAX_TURN_SPEED;
            lastRightPower = MAX_TURN_SPEED;
            log("turn %+.0f deg by ENCODERS: arc %.0f mm = %d ticks, target L %d / R %d",
                    turnDeg, arcMm, arcTicks, leftTarget, rightTarget);
        }
        arriveTimer.reset();
    }

    /** Call every loop while turning. True once the heading is reached and held (or timed out). */
    private boolean turnDone() {
        if (!imuOk) {
            double tolTicks = POSITION_TOLERANCE_MM * TICKS_PER_MM;
            boolean inTol = Math.abs(leftTarget - leftDrive.getCurrentPosition()) <= tolTicks
                    && Math.abs(rightTarget - rightDrive.getCurrentPosition()) <= tolTicks;
            lastInTol = inTol;
            if (!inTol) arriveTimer.reset();
            boolean arrived = arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
            boolean timedOut = stateTimer.seconds() > TURN_TIMEOUT_SECONDS;
            if (arrived || timedOut) {
                log("%s encoder turn %s %.1f s: off by L %d / R %d ticks", state,
                        arrived ? "done" : "TIMEOUT", stateTimer.seconds(),
                        leftTarget - leftDrive.getCurrentPosition(),
                        rightTarget - rightDrive.getCurrentPosition());
            }
            return arrived || timedOut;
        }

        double error = headingError(targetHeading);
        lastTurnError = error;

        // Safety: turning AWAY from the target means left/right are swapped (or the hub mounting is
        // wrong). Stop instead of spinning around the field.
        if (Math.abs(error) > turnStartError + 30) {
            warning = "Turned the WRONG WAY - left/right motors swapped or HUB_LOGO/HUB_USB wrong. "
                    + "Do the INIT hand check.";
            log("WRONG WAY: error %.1f > start %.1f + 30, heading %.1f -> stop",
                    Math.abs(error), turnStartError, lastHeading);
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            lastLeftPower = 0;
            lastRightPower = 0;
            enter(State.DONE);
            return false;
        }

        double power = Range.clip(error * P_TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);
        if (Math.abs(error) > HEADING_TOLERANCE_DEG && Math.abs(power) < MIN_TURN_POWER) {
            power = Math.signum(error) * MIN_TURN_POWER;
        }
        lastTurnPower = power;
        lastLeftPower = -power;
        lastRightPower = power;
        leftDrive.setPower(-power);   // + power = turn LEFT: left wheels back, right wheels forward
        rightDrive.setPower(power);

        lastInTol = Math.abs(error) <= HEADING_TOLERANCE_DEG;
        if (!lastInTol) arriveTimer.reset();
        if (stateTimer.seconds() > TURN_TIMEOUT_SECONDS) {
            warning = state + " timed out at " + Math.round(lastHeading) + " deg.";
            log("%s TIMEOUT %.1f s: heading %.1f, error %.1f deg",
                    state, stateTimer.seconds(), lastHeading, error);
            return true;
        }
        boolean done = arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
        if (done) {
            log("%s done %.1f s: heading %.1f, error %.1f deg",
                    state, stateTimer.seconds(), lastHeading, error);
        }
        return done;
    }
}
