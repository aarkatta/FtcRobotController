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


@Autonomous(name = "BioBuzz StarterBot Auto", group = "StarterBot")
public class BioBuzzStarterbotAuto extends OpMode {

    // The steps of the auto. Each one has a begin...() and a run...() method further down.
    private enum State { SPIN_UP, SHOOT, LEAVE, TURN_TO_PARK, TO_PARK, FINAL_TURN, DONE }

    // ================= ROUTE: the steps run in this order =================
    // TROUBLESHOOTING: comment out a line to skip that step; the auto goes straight on to the next one.
    // (SHOOT needs SPIN_UP before it to detect a missing launcher encoder.)
    private final State[] ROUTE = {
            State.SPIN_UP,        // 1) spin the launcher up to speed
            State.SHOOT,          // 2) feed the preloaded balls into the launcher
            State.LEAVE,          // 3) drive LEAVE_MM straight under the hive
            State.TURN_TO_PARK,   // 4) quarter turn to PARK_HEADING_DEG
            State.TO_PARK,        // 5) drive TO_PARK_MM on to the parking spot
            State.FINAL_TURN,     // 6) turn to FINAL_HEADING_DEG (skipped if already there)
    };

    final double LEAVE_MM = -7 * 12 * 25.4;  // 3) 7 ft straight toward and under the hive (negative = toward the hive)
    final double PARK_HEADING_DEG = 90;     // 4) quarter turn LEFT (+ = left); same route for RED and BLUE
    final double TO_PARK_MM = -914;         // 5) straight on to the parking spot (blue tape): 3 ft = 914 mm
    final double FINAL_HEADING_DEG = 90;    // 6) finish at this heading (same as step 4 = no extra turn)

    // ======= Configuration names (goBILDA StarterBot names; must match the Driver Station config) =======
    final String LEFT_DRIVE_NAME = "left_drive";
    final String RIGHT_DRIVE_NAME = "right_drive";
    final String INTAKE_NAME = "intake";
    final String LAUNCHER_NAME = "launcher";
    final String WINDMILL_NAME = "windmill";
    final String IMU_NAME = "imu";

    // ================= Directions (goBILDA StarterBot values) =================
    final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction WINDMILL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.REVERSE;
    // How the Control Hub is mounted (printed logo side, and which way the USB ports point).
    final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;   // hub stands on its edge; REV logo faces the robot's left
    final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ================= Shooting =================
    final int BALLS_TO_SHOOT = 4;               // preloaded balls
    final double FEED_SECONDS_PER_BALL = 1.0;   // seconds of feeding (at speed) per ball
    final double FEED_INTAKE_POWER = 0.5;       // intake helps push (goBILDA adds 0.5 while launching)
    final int LAUNCHER_TARGET_VELOCITY = 1210;  // ticks/sec (goBILDA)
    final int LAUNCHER_MIN_VELOCITY = 1160;     // only feed at/above this
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
    final double POSITION_TOLERANCE_MM = 10;       // drive: stop this close to the distance (also encoder turns)
    final double SLOW_DOWN_MM = 150;               // drive: start slowing down this far from the end
    final double MIN_DRIVE_POWER = 0.1;            // drive: slowest speed while slowing down (lower may stall)
    final double WHEEL_MISMATCH_MM = 100;          // drive: one wheel this far past the distance = other encoder dead

    // ================= INIT encoder check (push the robot BACKWARD 1 ft by hand) =================
    final double CHECK_PUSH_MM = 12 * 25.4;        // 1 ft = 305 mm, the same direction as LEAVE_MM
    final double CHECK_MOVED_MM = 20;              // a wheel has "moved" once it reads this far from 0
    final double CHECK_DONE_MM = 250;              // both wheels this far back = enough to judge PASS
    final double CHECK_MISMATCH = 0.15;            // L and R differ by more than 15% = slipping or turned
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
    private boolean openLoop = false;          // launcher has no encoder: run it on plain power
    private boolean launcherRunning = false;   // launcher was started and not yet stopped
    private boolean launcherFailed = false;    // SPIN_UP never reached speed: SHOOT is skipped

    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime arriveTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();

    private State state = State.DONE;
    private int routeIndex = -1;                 // which ROUTE entry is running (-1 = not started)
    private double fedSeconds = 0;
    private int leftTarget = 0;
    private int rightTarget = 0;
    private double driveDirection = 1;    // +1 forward, -1 backward
    private boolean driveStopped = false; // the current drive has reached its distance and powered off
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
    private double lastMovedMm = 0;        // how far the robot has moved in the current drive (average of both wheels)
    private double lastRemainingMm = 0;    // how far it still has to go (- = went past)
    private int checkLeftZero = 0;         // INIT encoder check: counts where the check started (gamepad1 A resets)
    private int checkRightZero = 0;
    private String checkResultLogged = ""; // last check result written to Events (each result logged once)
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
    // OpMode: INIT, START, LOOP, STOP
    // ======================================================================================

    /** INIT button: find every device and set it up. */
    @Override
    public void init() {
        initDrive();
        initShooter();
        initWindmill();
        initImu();
        log("route: %s", routeText());
    }

    /** After INIT, before START: the encoder check (push the robot backward 1 ft) and the Motors lines. */
    @Override
    public void init_loop() {
        if (gamepad1.a) {   // A = start the encoder check again from where the robot is now
            checkLeftZero = leftDrive.getCurrentPosition();
            checkRightZero = rightDrive.getCurrentPosition();
            checkResultLogged = "";
        }
        showEncoderCheck();
        showMotors();
        showEvents(EVENT_LINES_KEPT);
        telemetry.update();
    }

    /** START button: zero the clock and heading, then begin the first step of ROUTE. */
    @Override
    public void start() {
        autoTimer.reset();
        started = true;
        if (imuOk) imu.resetYaw();   // heading 0 = the way the robot faces right now
        targetHeading = 0;
        log("START: yaw reset, heading 0, counts L %d / R %d",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        nextStep();
    }

    /** Runs over and over: one call to the current step's run...() method, then telemetry. */
    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        switch (state) {
            case SPIN_UP:      runSpinUp();      break;
            case SHOOT:        runShoot(dt);     break;
            case LEAVE:        runLeave();       break;
            case TURN_TO_PARK: runTurnToPark();  break;
            case TO_PARK:      runToPark();      break;
            case FINAL_TURN:   runFinalTurn();   break;
            case DONE:                           break;
        }

        showTelemetry(dt);
    }

    /** STOP button (or the 30 s end): everything off. */
    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        stopLauncher();
        log("STOP: all motors off");
    }

    // ======================================================================================
    // INIT: one method per device
    // ======================================================================================

    private void initDrive() {
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
    }

    /** Missing shooter hardware: SPIN_UP and SHOOT are skipped, the robot still scores LEAVE + PARK. */
    private void initShooter() {
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
    }

    /** Missing windmill: feed with the intake only (works poorly - fix the config). */
    private void initWindmill() {
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
    }

    /** Missing IMU: turns fall back to encoders (less accurate). */
    private void initImu() {
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

    // ======================================================================================
    // Moving from one step to the next
    // ======================================================================================

    /** Ends the current step and begins the next ROUTE step that can run. After the last one: DONE. */
    private void nextStep() {
        while (true) {
            routeIndex++;
            if (routeIndex >= ROUTE.length) {
                finish();
                return;
            }
            State next = ROUTE[routeIndex];
            String skip = skipReason(next);
            if (skip != null) {
                log("skip %s: %s", next, skip);
                continue;
            }
            if (next != State.SHOOT) stopLauncher();   // launcher only stays on from SPIN_UP into SHOOT
            enter(next);
            switch (next) {
                case SPIN_UP:      beginSpinUp();      break;
                case SHOOT:        beginShoot();       break;
                case LEAVE:        beginLeave();       break;
                case TURN_TO_PARK: beginTurnToPark();  break;
                case TO_PARK:      beginToPark();      break;
                case FINAL_TURN:   beginFinalTurn();   break;
                default:                               break;
            }
            return;
        }
    }

    /** Why a step can't run right now (null = it can run). */
    private String skipReason(State step) {
        switch (step) {
            case SPIN_UP:
                return shooterOk ? null : "launcher/intake missing";
            case SHOOT:
                if (!shooterOk) return "launcher/intake missing";
                if (launcherFailed) return "launcher never reached speed";
                return null;
            case FINAL_TURN:
                if (Math.abs(FINAL_HEADING_DEG - targetHeading) <= 0.5) {
                    return String.format("already at %.0f deg", targetHeading);
                }
                return null;
            default:
                return null;
        }
    }

    /** Switches the state and restarts the step timers. */
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

    /** End of the auto (or a safety stop): everything off, state DONE. */
    private void finish() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lastLeftPower = 0;
        lastRightPower = 0;
        stopLauncher();
        routeIndex = ROUTE.length;
        finishTime = autoTimer.seconds();
        enter(State.DONE);
        log("DONE: heading %.1f deg, counts L %d / R %d",
                imuOk ? currentHeading() : 0.0,
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
    }

    // ======================================================================================
    // STEP 1: SPIN_UP - bring the launcher up to speed
    // ======================================================================================

    private void beginSpinUp() {
        launcherRunning = true;
        feed(false);
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        log("spin up launcher to %d ticks/s (ready at >= %d)",
                LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
    }

    private void runSpinUp() {
        feed(false);
        if (openLoop) {
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
            if (stateTimer.seconds() >= OPEN_LOOP_SPIN_UP_SECONDS) {
                log("open loop spin-up done: power %.2f for %.1f s",
                        LAUNCHER_FALLBACK_POWER, stateTimer.seconds());
                nextStep();
            }
            return;
        }
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        double vel = launcher.getVelocity();
        // Signed on purpose: a launcher spinning the WRONG way reads negative and never "reaches
        // speed", so it shows the "never reached speed" warning instead of feeding balls into it.
        if (vel < LAUNCHER_MIN_VELOCITY) {
            readyTimer.reset();   // not at speed yet: restart the steady-hold timer
        }
        if (stateTimer.seconds() > NO_ENCODER_CHECK_SECONDS && Math.abs(vel) < NO_ENCODER_VELOCITY) {
            openLoop = true;      // no encoder feedback: switch to plain power
            warning = "Launcher encoder reads 0 - check its cable. Using plain power.";
            log("NO launcher encoder: |vel| %.0f < %.0f after %.1f s -> power %.2f",
                    Math.abs(vel), NO_ENCODER_VELOCITY, stateTimer.seconds(), LAUNCHER_FALLBACK_POWER);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
            stateTimer.reset();
            return;
        }
        if (readyTimer.seconds() >= READY_HOLD_SECONDS) {
            log("launcher ready: %.0f ticks/s, held %.2f s, spin-up %.1f s",
                    vel, readyTimer.seconds(), stateTimer.seconds());
            nextStep();
        } else if (stateTimer.seconds() > SPIN_UP_TIMEOUT) {
            warning = "Launcher never reached speed - skipped shooting.";
            log("launcher NEVER ready: %.0f < %d after %.1f s -> skip shooting",
                    vel, LAUNCHER_MIN_VELOCITY, stateTimer.seconds());
            launcherFailed = true;   // nextStep() skips SHOOT and turns the launcher off
            nextStep();
        }
    }

    // ======================================================================================
    // STEP 2: SHOOT - feed the balls while the launcher is at speed
    // ======================================================================================

    private void beginShoot() {
        launcherRunning = true;
        commandLauncher();
        log("shoot %d balls: %.1f s of feeding at >= %d ticks/s%s", BALLS_TO_SHOOT,
                BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL, LAUNCHER_MIN_VELOCITY,
                openLoop ? " (open loop)" : "");
    }

    private void runShoot(double dt) {
        commandLauncher();
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
            nextStep();   // turns the launcher and feed off
        }
    }

    // ======================================================================================
    // STEP 3: LEAVE - drive straight under the hive
    // ======================================================================================

    private void beginLeave() {
        startDrive(LEAVE_MM);
    }

    private void runLeave() {
        if (driveDone()) nextStep();
    }

    // ======================================================================================
    // STEP 4: TURN_TO_PARK - quarter turn towards the parking spot
    // ======================================================================================

    private void beginTurnToPark() {
        startTurn(PARK_HEADING_DEG);
    }

    private void runTurnToPark() {
        if (turnDone()) nextStep();
    }

    // ======================================================================================
    // STEP 5: TO_PARK - drive straight on to the parking spot
    // ======================================================================================

    private void beginToPark() {
        startDrive(TO_PARK_MM);
    }

    private void runToPark() {
        if (driveDone()) nextStep();
    }

    // ======================================================================================
    // STEP 6: FINAL_TURN - finish at FINAL_HEADING_DEG (skipped when already there)
    // ======================================================================================

    private void beginFinalTurn() {
        startTurn(FINAL_HEADING_DEG);
    }

    private void runFinalTurn() {
        if (turnDone()) nextStep();
    }

    // ======================================================================================
    // Shooter helpers
    // ======================================================================================

    /** Keeps the launcher running: velocity control, or plain power if it has no encoder. */
    private void commandLauncher() {
        if (openLoop) {
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
        } else {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
    }

    private void feed(boolean on) {
        if (!shooterOk) return;
        if (on && !feedOn && fedSeconds == 0) {
            log("first feed: windmill %.1f, intake %.1f",
                    windmillOk ? 1.0 : 0.0, FEED_INTAKE_POWER);
        }
        feedOn = on;
        if (windmillOk) windmill.setPower(on ? 1.0 : 0.0);
        intake.setPower(on ? FEED_INTAKE_POWER : 0.0);
    }

    /** Launcher and feed off. Does nothing if the launcher isn't running. */
    private void stopLauncher() {
        if (!shooterOk || !launcherRunning) return;
        launcherRunning = false;
        feed(false);
        if (openLoop) {
            launcher.setPower(0);
        } else {
            launcher.setVelocity(0);
        }
        log("launcher + feed off (%s)", openLoop ? "power 0" : "velocity 0");
    }

    // ======================================================================================
    // Heading
    // ======================================================================================

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

    // ======================================================================================
    // Straight drive - used by LEAVE and TO_PARK.
    // The CODE stops the robot: RUN_USING_ENCODER keeps each wheel at the speed we ask for, and
    // driveDone() sets the power to 0 once the robot (the average of both wheels) has moved driveMm.
    // The IMU keeps it straight. Distance still comes from the wheel encoders.
    // ======================================================================================

    private void startDrive(double mm) {
        int ticks = (int) Math.round(mm * TICKS_PER_MM);
        leftStart = leftDrive.getCurrentPosition();
        rightStart = rightDrive.getCurrentPosition();
        leftTarget = leftStart + ticks;     // for telemetry only: the code, not the motor, decides when to stop
        rightTarget = rightStart + ticks;
        driveMm = mm;                       // the distance the robot must move (- = toward the hive)
        driveDirection = mm < 0 ? -1 : 1;
        driveStopped = false;
        lastMovedMm = 0;
        lastRemainingMm = Math.abs(mm);
        moveTimeout = Math.max(3.0, Math.abs(mm) / 250.0);   // generous: ~250 mm per second
        // Power 0 first, then the mode. From here on driveDone() sets the power every loop.
        stopDrive();
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastCorrection = 0;
        lastInTol = false;
        arriveTimer.reset();
        log("drive %.0f mm (%d ticks): stop by code at the average of both wheels, hold %.1f deg, timeout %.1f s",
                mm, ticks, targetHeading, moveTimeout);
    }

    /** Call every loop while driving. True once the robot has moved driveMm and settled (or timed out). */
    private boolean driveDone() {
        // 1) How far has the robot moved? The average of both wheels = the middle of the robot.
        double leftMovedMm = (leftDrive.getCurrentPosition() - leftStart) / TICKS_PER_MM;
        double rightMovedMm = (rightDrive.getCurrentPosition() - rightStart) / TICKS_PER_MM;
        double movedMm = (leftMovedMm + rightMovedMm) / 2.0;
        // Distance still to go, in the direction of travel: + = not there yet, - = went past.
        double remainingMm = (driveMm - movedMm) * driveDirection;
        lastMovedMm = movedMm;
        lastRemainingMm = remainingMm;

        if (!driveStopped) {
            boolean arrived = remainingMm <= POSITION_TOLERANCE_MM;
            // One wheel went well past the whole distance: the other wheel's encoder is probably not counting.
            boolean wheelRanAway = Math.max(Math.abs(leftMovedMm), Math.abs(rightMovedMm))
                    > Math.abs(driveMm) + WHEEL_MISMATCH_MM;

            if (arrived || wheelRanAway) {
                // 2) THE STOP: power 0. BRAKE mode stops the wheels quickly.
                stopDrive();
                driveStopped = true;
                arriveTimer.reset();   // now wait ARRIVE_HOLD_SECONDS for the robot to settle
                if (wheelRanAway) {
                    warning = state + ": one drive encoder isn't counting - stopped early. Check the encoder cables.";
                    log("%s STOP (encoder mismatch): L %.0f / R %.0f mm of %.0f",
                            state, leftMovedMm, rightMovedMm, driveMm);
                } else {
                    log("%s STOP: moved %.0f of %.0f mm (L %.0f / R %.0f)",
                            state, movedMm, driveMm, leftMovedMm, rightMovedMm);
                }
            } else {
                // 3) Still driving. Full DRIVE_SPEED until the last SLOW_DOWN_MM, then slow down
                //    in step with the distance left (never below MIN_DRIVE_POWER, or it would stall).
                double speed = Range.clip(DRIVE_SPEED * remainingMm / SLOW_DOWN_MM, MIN_DRIVE_POWER, DRIVE_SPEED);

                // 4) Stay straight: heading error -> turn a little. + correction = turn LEFT
                //    (right side faster). Powers are signed here, so this works driving both ways.
                double correction = 0;
                lastDriveError = 0;
                if (imuOk) {
                    lastDriveError = headingError(targetHeading);
                    correction = Range.clip(lastDriveError * P_DRIVE_GAIN, -0.2, 0.2);
                    correction = Range.clip(correction, -speed, speed);   // never spin a wheel backwards
                }
                lastCorrection = correction;
                double base = driveDirection * speed;   // - = toward the hive
                lastLeftPower = Range.clip(base - correction, -1.0, 1.0);
                lastRightPower = Range.clip(base + correction, -1.0, 1.0);
                leftDrive.setPower(lastLeftPower);
                rightDrive.setPower(lastRightPower);
            }
        }
        lastInTol = driveStopped;

        // 5) Safety: never drive longer than moveTimeout.
        if (stateTimer.seconds() > moveTimeout) {
            stopDrive();
            warning = state + " timed out - check drive encoders, or the robot hit something.";
            log("%s TIMEOUT %.1f s: moved %.0f of %.0f mm (L %.0f / R %.0f), heading %.1f",
                    state, stateTimer.seconds(), movedMm, driveMm, leftMovedMm, rightMovedMm, lastHeading);
            return true;
        }

        // 6) Done once the robot has been stopped for ARRIVE_HOLD_SECONDS (it may roll a few mm after the stop).
        boolean done = driveStopped && arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
        if (done) {
            log("%s arrived %.1f s: moved %.0f of %.0f mm (L %.0f / R %.0f), heading %.1f",
                    state, stateTimer.seconds(), movedMm, driveMm, leftMovedMm, rightMovedMm, lastHeading);
        }
        return done;
    }

    /** Both drive motors to power 0. */
    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lastLeftPower = 0;
        lastRightPower = 0;
    }

    // ======================================================================================
    // Turn in place (IMU) - used by TURN_TO_PARK and FINAL_TURN
    // ======================================================================================

    private void startTurn(double headingDeg) {
        double turnDeg = headingDeg - targetHeading;   // only used without an IMU
        double fromDeg = targetHeading;
        targetHeading = headingDeg;
        leftStart = leftDrive.getCurrentPosition();
        rightStart = rightDrive.getCurrentPosition();
        lastInTol = false;
        // STOP before the turn: the last drive power (~0.5) is still set. In RUN_TO_POSITION it was only a
        // speed limit, but after the mode change below it would push both wheels forward for one loop.
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        lastLeftPower = 0;
        lastRightPower = 0;
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
        // wrong). Stop the whole auto instead of spinning around the field.
        if (Math.abs(error) > turnStartError + 30) {
            warning = "Turned the WRONG WAY - left/right motors swapped or HUB_LOGO/HUB_USB wrong. "
                    + "Do the INIT hand check.";
            log("WRONG WAY: error %.1f > start %.1f + 30, heading %.1f -> stop",
                    Math.abs(error), turnStartError, lastHeading);
            finish();
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

    // ======================================================================================
    // Telemetry
    // ======================================================================================

    /** The whole Driver Station screen while running: step values, motors, events. */
    private void showTelemetry(double dt) {
        telemetry.addData("State", "%s   %.1f s in step", state, stateTimer.seconds());
        telemetry.addData("Auto time", "%.1f / 30 s   (loop %.0f ms)", autoTimer.seconds(), dt * 1000);
        double launcherVel = shooterOk ? launcher.getVelocity() : 0;
        double headingNow = imuOk ? currentHeading() : 0.0;

        switch (state) {
            case SPIN_UP:      showSpinUpTelemetry(launcherVel);  break;
            case SHOOT:        showShootTelemetry();              break;
            case LEAVE:
            case TO_PARK:      showDriveTelemetry();              break;
            case TURN_TO_PARK:
            case FINAL_TURN:   showTurnTelemetry();               break;
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

    private void showSpinUpTelemetry(double launcherVel) {
        if (openLoop) {
            telemetry.addData("Spin-up", "OPEN LOOP power %.2f, %.1f / %.1f s",
                    LAUNCHER_FALLBACK_POWER, stateTimer.seconds(), OPEN_LOOP_SPIN_UP_SECONDS);
            return;
        }
        telemetry.addData("Launcher", "%.0f ticks/s  (target %d, ready at >= %d)",
                launcherVel, LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
        telemetry.addData("Steady at speed", "%.2f / %.2f s",
                readyTimer.seconds(), READY_HOLD_SECONDS);
        telemetry.addData("Spin-up time", "%.1f / %.1f s  (encoder check at %.1f s)",
                stateTimer.seconds(), SPIN_UP_TIMEOUT, NO_ENCODER_CHECK_SECONDS);
    }

    private void showShootTelemetry() {
        telemetry.addData("Feeding", "%s   windmill %.1f, intake %.1f",
                feedOn ? "ON" : "PAUSED (launcher < " + LAUNCHER_MIN_VELOCITY + ")",
                feedOn && windmillOk ? 1.0 : 0.0, feedOn ? FEED_INTAKE_POWER : 0.0);
        telemetry.addData("Fed seconds", "%.2f / %.1f   (~%d of %d balls)", fedSeconds,
                BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL,
                Math.min(BALLS_TO_SHOOT, (int) (fedSeconds / FEED_SECONDS_PER_BALL)), BALLS_TO_SHOOT);
        telemetry.addData("Feed pauses", "%d", feedPauses);
        telemetry.addData("Shoot limits", "step %.1f / %.0f s, auto %.1f / %.0f s",
                stateTimer.seconds(), SHOOT_MAX_SECONDS, autoTimer.seconds(), SHOOT_DEADLINE_SECONDS);
    }

    private void showDriveTelemetry() {
        telemetry.addData("Drive", "%.0f mm %s  (%d ticks)", driveMm,
                driveDirection < 0 ? "BACKWARD" : "FORWARD", Math.round(driveMm * TICKS_PER_MM));
        telemetry.addData("Moved (avg)", "%.0f mm, %.0f mm to go  (slow down in the last %.0f)",
                lastMovedMm, lastRemainingMm, SLOW_DOWN_MM);
        telemetry.addData("Driven L / R", "%.0f / %.0f mm",
                (leftDrive.getCurrentPosition() - leftStart) / TICKS_PER_MM,
                (rightDrive.getCurrentPosition() - rightStart) / TICKS_PER_MM);
        telemetry.addData("Steer", "heading error %.1f deg -> correction %+.3f",
                lastDriveError, lastCorrection);
        telemetry.addData("Power L / R", "%+.2f / %+.2f", lastLeftPower, lastRightPower);
        telemetry.addData("Stopped", "%s, settled %.2f / %.2f s   timeout %.1f / %.1f s",
                driveStopped ? "YES" : "no", driveStopped ? arriveTimer.seconds() : 0.0,
                ARRIVE_HOLD_SECONDS, stateTimer.seconds(), moveTimeout);
    }

    private void showTurnTelemetry() {
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
    }

    /** The steps left in ROUTE, e.g. "SPIN_UP > SHOOT > LEAVE". */
    private String routeText() {
        if (ROUTE.length == 0) return "(no steps)";
        StringBuilder text = new StringBuilder();
        for (State step : ROUTE) {
            if (text.length() > 0) text.append(" > ");
            text.append(step);
        }
        return text.toString();
    }

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

    /**
     * INIT encoder check. Push the robot BACKWARD 1 ft by hand (the LEAVE direction). Both drive counts
     * must go NEGATIVE. A count that stays at 0 = encoder cable unplugged or in the wrong port. A count that
     * goes POSITIVE = that motor's direction setting is backward (positive power would drive it backward).
     */
    private void showEncoderCheck() {
        int leftNow = leftDrive.getCurrentPosition();
        int rightNow = rightDrive.getCurrentPosition();
        double leftMm = (leftNow - checkLeftZero) / TICKS_PER_MM;
        double rightMm = (rightNow - checkRightZero) / TICKS_PER_MM;

        telemetry.addLine("---- ENCODER CHECK: push the robot BACKWARD 1 ft (toward the hive) ----");
        telemetry.addData("Left / Right pos", "%d / %d   (want both NEGATIVE, about %d)",
                leftNow, rightNow, -Math.round(CHECK_PUSH_MM * TICKS_PER_MM));
        telemetry.addData("Left encoder", "%+.0f mm  %s", leftMm,
                wheelCheck(leftMm, rightMm, "LEFT_DIRECTION", LEFT_DRIVE_NAME));
        telemetry.addData("Right encoder", "%+.0f mm  %s", rightMm,
                wheelCheck(rightMm, leftMm, "RIGHT_DIRECTION", RIGHT_DRIVE_NAME));

        String result = encoderCheckResult(leftMm, rightMm);
        telemetry.addData("Encoder check", result);
        // Write each new kind of PASS / FAIL to Events once, so it is still there after START.
        String kind = result.replaceAll("[0-9]", "");   // ignore the changing mm numbers
        if ((result.startsWith("PASS") || result.startsWith("FAIL")) && !kind.equals(checkResultLogged)) {
            checkResultLogged = kind;
            log("encoder check %s (L %.0f / R %.0f mm)", result.startsWith("PASS") ? "PASS" : "FAIL",
                    leftMm, rightMm);
        }
        if (imuOk) {
            telemetry.addData("Heading", "%.1f deg   (turn the robot LEFT by hand -> goes UP)", currentHeading());
        }
        telemetry.addLine("gamepad1 A = start the check again from here");
    }

    /** The verdict for one wheel. own / other: mm moved by this wheel and by the other one. */
    private String wheelCheck(double ownMm, double otherMm, String directionSetting, String name) {
        if (ownMm <= -CHECK_MOVED_MM) return "OK: negative (backward)";
        if (ownMm >= CHECK_MOVED_MM) {
            return "WRONG: POSITIVE - flip " + directionSetting;
        }
        if (Math.abs(otherMm) >= CHECK_DONE_MM) {   // the other wheel moved a long way, this one didn't
            return "WRONG: STAYS AT 0 - " + name + " encoder cable unplugged or in the wrong port";
        }
        return "waiting - push the robot backward";
    }

    /** The overall result: waiting, keep pushing, PASS or FAIL. */
    private String encoderCheckResult(double leftMm, double rightMm) {
        if (leftMm >= CHECK_MOVED_MM && rightMm >= CHECK_MOVED_MM) {
            return "FAIL - both POSITIVE. Pushed FORWARD by mistake? Press A and push toward the hive. "
                    + "If it was backward, flip LEFT_DIRECTION and RIGHT_DIRECTION.";
        }
        boolean leftWrong = wheelCheck(leftMm, rightMm, "", "").startsWith("WRONG");
        boolean rightWrong = wheelCheck(rightMm, leftMm, "", "").startsWith("WRONG");
        if (leftWrong || rightWrong) {
            return "FAIL - " + (leftWrong && rightWrong ? "both wheels" : leftWrong ? "left wheel" : "right wheel")
                    + ", see the line above. Fix it before running the auto.";
        }
        if (leftMm > -CHECK_DONE_MM || rightMm > -CHECK_DONE_MM) {
            return String.format("keep pushing: %.0f / %.0f mm of about %.0f", leftMm, rightMm, -CHECK_PUSH_MM);
        }
        double avg = Math.abs(leftMm + rightMm) / 2.0;
        if (Math.abs(leftMm - rightMm) > CHECK_MISMATCH * avg) {
            return String.format("PASS (both negative) but L and R differ by %.0f mm - "
                    + "a wheel slipped, or the robot turned while pushed", Math.abs(leftMm - rightMm));
        }
        return "PASS - both encoders count backward as negative";
    }

    /** One line per motor: direction SETTING | which way it is TOLD to turn | which way it REALLY turns. */
    private void showMotors() {
        telemetry.addLine("---- Motors (set | told | turning) ----");
        motorLine("Left drive", leftDrive, driveCommand(leftDrive, leftTarget), leftCheck);
        motorLine("Right drive", rightDrive, driveCommand(rightDrive, rightTarget), rightCheck);
        if (shooterOk) {
            // The launcher is only ever told to spin forward, and only while it's running.
            motorLine("Launcher", launcher, launcherRunning ? 1 : 0, launcherCheck);
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
}
