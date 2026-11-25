package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * BioBuzz StarterBot Odo Auto: the same route and shooting as {@link BioBuzzStarterbotAuto}, for the
 * FOUR-WHEEL (mecanum) robot with a goBILDA PINPOINT odometry computer and two odometry pods.
 *
 * <ul>
 *   <li>Position and heading come from the Pinpoint: x (forward), y (left) and heading, in mm and degrees,
 *       measured from START. The Pinpoint reads the two pods (front pod = its X port, left pod = its Y port)
 *       and its own IMU, and combines them.</li>
 *   <li>LEAVE and TO_PARK stop when the distance driven forward reaches the target. The drive-motor
 *       encoders are NOT used.</li>
 *   <li>Turns and straight-line steering use the Pinpoint heading.</li>
 *   <li>{@link BioBuzzStarterbotAuto} is unchanged and still uses the two-wheel setup.</li>
 * </ul>
 *
 * How the sensor works is explained in TeamCode/docs/BioBuzzStarterbotOdoAuto_Odometry_and_IMU.md.
 */
@Autonomous(name = "BioBuzz StarterBot Odo Auto", group = "StarterBot")
public class BioBuzzStarterbotOdoAuto extends OpMode {

    // The steps of the auto. Each one has a begin...() / run...() method further down.
    private enum State { SPIN_UP, SHOOT, LEAVE, TURN_TO_PARK, TO_PARK, FINAL_TURN, DONE }

    // ================= ROUTE: the steps run in this order =================
    // TROUBLESHOOTING: comment out a line to skip that step; the auto goes straight on to the next one.
    private final State[] ROUTE = {
            State.SPIN_UP,        // 1) spin the launcher up to speed
            State.SHOOT,          // 2) feed the preloaded balls into the launcher
            State.LEAVE,          // 3) drive LEAVE_MM straight under the hive (Pinpoint distance)
            State.TURN_TO_PARK,   // 4) quarter turn to PARK_HEADING_DEG (Pinpoint heading)
            State.TO_PARK,        // 5) drive TO_PARK_MM on to the parking spot (Pinpoint distance)
            State.FINAL_TURN,     // 6) turn to FINAL_HEADING_DEG (skipped if already there)
    };

    // Same starting values as BioBuzzStarterbotAuto. Re-measure them with the Pinpoint.
    final double LEAVE_MM = -7 * 12 * 25.4;  // 3) 7 ft toward and under the hive (negative = toward the hive)
    final double PARK_HEADING_DEG = 90;      // 4) quarter turn LEFT (+ = left)
    final double TO_PARK_MM = -914;          // 5) on to the parking spot: 3 ft = 914 mm
    final double FINAL_HEADING_DEG = 90;     // 6) finish at this heading

    // ======= Configuration names (must match the Driver Station configuration exactly) =======
    final String LEFT_FRONT_NAME = "frontLeft";
    final String LEFT_BACK_NAME = "backLeft";
    final String RIGHT_FRONT_NAME = "frontRight";
    final String RIGHT_BACK_NAME = "backRight";
    final String PINPOINT_NAME = "pinpoint";
    final String INTAKE_NAME = "intake";
    final String LAUNCHER_NAME = "launcher";
    final String WINDMILL_NAME = "windmill";

    // ================= Directions =================
    // Drive: the mecanum wheels are set like the two-wheel auto (both sides REVERSE). Verify on the floor.
    final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction WINDMILL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // ================= Pinpoint (odometry computer) =================
    // Pod directions: FORWARD = the count rises when the robot moves forward (front pod) or left (left pod).
    // Checked by the INIT odometry check. Change to REVERSED if the check says WRONG.
    final GoBildaPinpointDriver.EncoderDirection FRONT_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    final GoBildaPinpointDriver.EncoderDirection LEFT_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // The pod type sets the tick count (goBILDA 4-Bar Mini Odometry Pod, 32 mm wheel).
    final GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    // Where the pods sit relative to the robot's centre of rotation (mm). PLACEHOLDER 0: measure on the robot.
    //   FRONT_POD_SIDE_OFFSET_MM: how far LEFT of centre the front pod is (+ = left). 0 if it is centred.
    //   LEFT_POD_FORWARD_OFFSET_MM: how far IN FRONT of centre the left pod is (+ = front). 0 if level with centre.
    final double FRONT_POD_SIDE_OFFSET_MM = 0;
    final double LEFT_POD_FORWARD_OFFSET_MM = 0;

    // ================= Shooting (same values as BioBuzzStarterbotAuto) =================
    final int BALLS_TO_SHOOT = 4;
    final double FEED_SECONDS_PER_BALL = 1.0;
    final double FEED_INTAKE_POWER = 0.5;
    final int LAUNCHER_TARGET_VELOCITY = 1210;   // ticks/sec (goBILDA)
    final int LAUNCHER_MIN_VELOCITY = 1160;      // only feed at/above this
    final double READY_HOLD_SECONDS = 0.3;
    final double SPIN_UP_TIMEOUT = 4.0;
    final double SHOOT_MAX_SECONDS = 10.0;
    final double SHOOT_DEADLINE_SECONDS = 17.0;  // auto clock: stop shooting by here, then park
    final double NO_ENCODER_CHECK_SECONDS = 1.5;
    final double NO_ENCODER_VELOCITY = 50;
    final double LAUNCHER_FALLBACK_POWER = 0.85;
    final double OPEN_LOOP_SPIN_UP_SECONDS = 2.5;

    // ================= Driving (distance and heading from the Pinpoint) =================
    final double DRIVE_SPEED = 0.5;
    final double P_DRIVE_GAIN = 0.03;             // heading correction while driving straight
    final double MAX_DRIVE_CORRECTION = 0.2;
    final double POSITION_TOLERANCE_MM = 10;      // stop this close to the distance
    final double SLOW_DOWN_MM = 150;              // start slowing down this far from the end
    final double MIN_DRIVE_POWER = 0.1;           // slowest speed while slowing down
    final double NOT_COUNTING_SECONDS = 1.5;      // no Pinpoint movement after this long = pods not counting
    final double NOT_COUNTING_MM = 5;
    final double ARRIVE_HOLD_SECONDS = 0.25;      // stay still this long before the next step
    final double MAX_TURN_SPEED = 0.4;
    final double MIN_TURN_POWER = 0.08;
    final double P_TURN_GAIN = 0.02;
    final double HEADING_TOLERANCE_DEG = 2.0;
    final double TURN_TIMEOUT_SECONDS = 3.0;

    // ================= INIT odometry check (push by hand) =================
    final double CHECK_MOVED_MM = 20;             // a direction has "moved" once it changes this far

    // ================= Telemetry and data recording =================
    final int EVENT_LINES_SHOWN = 12;
    final int EVENT_LINES_KEPT = 40;
    final double DIRECTION_MIN_VELOCITY = 50;     // ticks/sec: below this a motor counts as "not turning"
    final double WRONG_WAY_SECONDS = 0.3;
    final double NOT_TURNING_SECONDS = 1.0;
    // One row every LOG_INTERVAL_SEC from START until the run ends. Rows go to the Robot Log (tag BioBuzzOdo)
    // and to a CSV file in LOG_FOLDER on the Control Hub.
    private static final double LOG_INTERVAL_SEC = 0.05;   // 20 rows per second
    private static final String LOG_FOLDER = "/sdcard/FIRST";
    private static final String LOG_TAG = "BioBuzzOdo";
    private static final String LOG_HEADER = "time_s,state,x_mm,y_mm,heading_deg,target_deg,"
            + "drive_mm,drive_forward_mm,power_left,power_right,launcher_vel";

    // ---- Hardware ----
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx intake;
    private DcMotorEx launcher;
    private CRServo windmill;
    private boolean pinpointOk = false;
    private boolean shooterOk = false;
    private boolean windmillOk = false;

    // ---- Pose (read from the Pinpoint once per loop; zero at START) ----
    private double poseXmm = 0;          // forward from START (+ = forward)
    private double poseYmm = 0;          // left from START (+ = left)
    private double poseHeadingDeg = 0;   // + = turned left (counter-clockwise), 0 = facing at START

    // ---- INIT check: the pose when the check started (A resets it) ----
    private double checkX = 0;
    private double checkY = 0;
    private String checkResultLogged = "";

    // ---- Shooter state ----
    private boolean openLoop = false;
    private boolean launcherRunning = false;
    private boolean launcherFailed = false;
    private double fedSeconds = 0;
    private boolean feedOn = false;
    private int feedPauses = 0;

    // ---- Timers ----
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime arriveTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime dataTimer = new ElapsedTime();

    // ---- Auto state ----
    private State state = State.DONE;
    private int routeIndex = -1;
    private boolean started = false;
    private boolean firstState = true;
    private double finishTime = 0;
    private double targetHeading = 0;      // degrees: the heading the drive holds and the turn aims for
    private double turnStartError = 0;
    private double lastTurnError = 0;
    private double lastTurnPower = 0;
    private double driveMm = 0;            // distance asked for by the current drive (- = toward the hive)
    private double driveDirection = 1;
    private double driveStartX = 0;        // pose when the current drive started
    private double driveStartY = 0;
    private double driveStartHeadingRad = 0;
    private double moveTimeout = 4.0;
    private boolean driveStopped = false;
    private double lastMovedMm = 0;
    private double lastRemainingMm = 0;
    private double lastDriveError = 0;
    private double lastCorrection = 0;
    private double lastPower = 0;          // drive power sent to the left and right sides
    private double lastRightPower = 0;
    private boolean lastInTol = false;
    private String warning = "";

    // ---- Telemetry and recording ----
    private final String[] events = new String[EVENT_LINES_KEPT];
    private int eventCount = 0;
    private FileWriter dataFile = null;    // null = no file (the Robot Log still gets the rows)
    private final DirectionCheck launcherCheck = new DirectionCheck(LAUNCHER_NAME);
    private final DirectionCheck intakeCheck = new DirectionCheck(INTAKE_NAME);

    // ======================================================================================
    // OpMode: INIT, START, LOOP, STOP
    // ======================================================================================

    /** INIT button: find every device and set it up. */
    @Override
    public void init() {
        initDrive();
        initPinpoint();
        initShooter();
        initWindmill();
        log("route: %s", routeText());
    }

    /** After INIT, before START: the odometry check (push by hand) and the Motors lines. */
    @Override
    public void init_loop() {
        if (pinpointOk) readPose();
        if (gamepad1.a && pinpointOk) {   // A = start the check again from where the robot is now
            checkX = poseXmm;
            checkY = poseYmm;
            checkResultLogged = "";
        }
        showOdometryCheck();
        showMotors();
        showEvents(EVENT_LINES_KEPT);
        telemetry.update();
    }

    /** START button: zero the pose and the clock, open the data file, then begin the first step. */
    @Override
    public void start() {
        autoTimer.reset();
        started = true;
        targetHeading = 0;
        if (pinpointOk) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));   // here = (0, 0), heading 0
            readPose();
        }
        log("START: pose zeroed (x 0, y 0, heading 0)");
        openDataLog();
        nextStep();
    }

    /** Runs over and over: read the Pinpoint, one call to the current step, record, telemetry. */
    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (pinpointOk) readPose();

        switch (state) {
            case SPIN_UP:      runSpinUp();      break;
            case SHOOT:        runShoot(dt);     break;
            case LEAVE:        runDrive();       break;
            case TURN_TO_PARK: runTurn();        break;
            case TO_PARK:      runDrive();       break;
            case FINAL_TURN:   runTurn();        break;
            case DONE:                           break;
        }

        recordData();
        showTelemetry(dt);
    }

    /** STOP button (or the 30 s end): everything off, data file closed. */
    @Override
    public void stop() {
        stopDrive();
        stopLauncher();
        log("STOP: all motors off");
        closeDataLog();
    }

    // ======================================================================================
    // INIT: one method per device
    // ======================================================================================

    private void initDrive() {
        leftFront = hardwareMap.get(DcMotor.class, LEFT_FRONT_NAME);
        leftBack = hardwareMap.get(DcMotor.class, LEFT_BACK_NAME);
        rightFront = hardwareMap.get(DcMotor.class, RIGHT_FRONT_NAME);
        rightBack = hardwareMap.get(DcMotor.class, RIGHT_BACK_NAME);
        for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // the Pinpoint measures distance, not these
        }
        leftFront.setDirection(LEFT_DIRECTION);
        leftBack.setDirection(LEFT_DIRECTION);
        rightFront.setDirection(RIGHT_DIRECTION);
        rightBack.setDirection(RIGHT_DIRECTION);
        log("init drive: 4 mecanum motors, left %s, right %s, BRAKE", LEFT_DIRECTION, RIGHT_DIRECTION);
    }

    /**
     * Pinpoint: the pods and its IMU. The robot must be STILL here, because resetPosAndIMU() recalibrates the
     * IMU. Pinpoint missing: no distance and no heading, so LEAVE, TO_PARK and both turns are skipped.
     */
    private void initPinpoint() {
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_NAME);
            pinpoint.setOffsets(FRONT_POD_SIDE_OFFSET_MM, LEFT_POD_FORWARD_OFFSET_MM, DistanceUnit.MM);
            pinpoint.setEncoderResolution(POD_TYPE);
            pinpoint.setEncoderDirections(FRONT_POD_DIRECTION, LEFT_POD_DIRECTION);
            pinpoint.resetPosAndIMU();
            pinpointOk = true;
            log("init Pinpoint OK: front pod %s, left pod %s, offsets front %.0f / left %.0f mm",
                    FRONT_POD_DIRECTION, LEFT_POD_DIRECTION, FRONT_POD_SIDE_OFFSET_MM, LEFT_POD_FORWARD_OFFSET_MM);
        } catch (Exception e) {
            pinpointOk = false;
            log("init Pinpoint MISSING: %s", e.getMessage());
        }
    }

    /** Missing shooter hardware: SPIN_UP and SHOOT are skipped, the robot still drives. */
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
            log("init shooter OK: intake %s, launcher %s, PIDF 40/0/0/12.5", INTAKE_DIRECTION, LAUNCHER_DIRECTION);
        } catch (Exception e) {
            shooterOk = false;
            log("init shooter MISSING: %s", e.getMessage());
        }
    }

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
                case SPIN_UP:      beginSpinUp();                 break;
                case SHOOT:        beginShoot();                  break;
                case LEAVE:        startDrive(LEAVE_MM);          break;
                case TURN_TO_PARK: startTurn(PARK_HEADING_DEG);   break;
                case TO_PARK:      startDrive(TO_PARK_MM);        break;
                case FINAL_TURN:   startTurn(FINAL_HEADING_DEG);  break;
                default:                                          break;
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
            case LEAVE:
            case TO_PARK:
                return pinpointOk ? null : "Pinpoint missing (no distance to stop on)";
            case TURN_TO_PARK:
            case FINAL_TURN:
                if (!pinpointOk) return "Pinpoint missing (no heading to turn to)";
                if (step == State.FINAL_TURN && Math.abs(FINAL_HEADING_DEG - targetHeading) <= 0.5) {
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
        stopDrive();
        stopLauncher();
        routeIndex = ROUTE.length;
        finishTime = autoTimer.seconds();
        enter(State.DONE);
        log("DONE: x %.0f mm, y %.0f mm, heading %.1f deg", poseXmm, poseYmm, poseHeadingDeg);
    }

    // ======================================================================================
    // STEP 1: SPIN_UP - bring the launcher up to speed
    // ======================================================================================

    private void beginSpinUp() {
        launcherRunning = true;
        feed(false);
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        log("spin up launcher to %d ticks/s (ready at >= %d)", LAUNCHER_TARGET_VELOCITY, LAUNCHER_MIN_VELOCITY);
    }

    private void runSpinUp() {
        feed(false);
        if (openLoop) {
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
            if (stateTimer.seconds() >= OPEN_LOOP_SPIN_UP_SECONDS) {
                log("open loop spin-up done: power %.2f for %.1f s", LAUNCHER_FALLBACK_POWER, stateTimer.seconds());
                nextStep();
            }
            return;
        }
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        double vel = launcher.getVelocity();
        // Signed on purpose: a launcher spinning the WRONG way reads negative and never "reaches speed".
        if (vel < LAUNCHER_MIN_VELOCITY) {
            readyTimer.reset();
        }
        if (stateTimer.seconds() > NO_ENCODER_CHECK_SECONDS && Math.abs(vel) < NO_ENCODER_VELOCITY) {
            openLoop = true;
            warning = "Launcher encoder reads 0 - check its cable. Using plain power.";
            log("NO launcher encoder: |vel| %.0f < %.0f after %.1f s -> power %.2f",
                    Math.abs(vel), NO_ENCODER_VELOCITY, stateTimer.seconds(), LAUNCHER_FALLBACK_POWER);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
            stateTimer.reset();
            return;
        }
        if (readyTimer.seconds() >= READY_HOLD_SECONDS) {
            log("launcher ready: %.0f ticks/s, held %.2f s, spin-up %.1f s", vel, readyTimer.seconds(), stateTimer.seconds());
            nextStep();
        } else if (stateTimer.seconds() > SPIN_UP_TIMEOUT) {
            warning = "Launcher never reached speed - skipped shooting.";
            log("launcher NEVER ready: %.0f < %d after %.1f s -> skip shooting",
                    vel, LAUNCHER_MIN_VELOCITY, stateTimer.seconds());
            launcherFailed = true;
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
                BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL, LAUNCHER_MIN_VELOCITY, openLoop ? " (open loop)" : "");
    }

    private void runShoot(double dt) {
        commandLauncher();
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
            nextStep();
        }
    }

    // ======================================================================================
    // Straight drive - used by LEAVE and TO_PARK.
    // The CODE stops the robot when the distance driven forward (from the Pinpoint pose) reaches driveMm.
    // The Pinpoint heading keeps it straight. The drive-motor encoders are not used.
    // ======================================================================================

    private void startDrive(double mm) {
        driveMm = mm;
        driveDirection = mm < 0 ? -1 : 1;
        driveStartX = poseXmm;
        driveStartY = poseYmm;
        driveStartHeadingRad = Math.toRadians(poseHeadingDeg);
        driveStopped = false;
        lastMovedMm = 0;
        lastRemainingMm = Math.abs(mm);
        moveTimeout = Math.max(3.0, Math.abs(mm) / 250.0);   // generous: ~250 mm per second
        stopDrive();
        lastCorrection = 0;
        lastInTol = false;
        arriveTimer.reset();
        log("drive %.0f mm by PINPOINT: stop when forward distance reaches it, hold %.1f deg, timeout %.1f s",
                mm, targetHeading, moveTimeout);
    }

    /**
     * Distance driven FORWARD since the drive started, measured along the heading the robot had at the start:
     * forward = dx*cos(h) + dy*sin(h). Negative = backward.
     */
    private double forwardSinceDriveStart() {
        double dx = poseXmm - driveStartX;
        double dy = poseYmm - driveStartY;
        return dx * Math.cos(driveStartHeadingRad) + dy * Math.sin(driveStartHeadingRad);
    }

    /** Call every loop while driving. True once the distance is reached (or on timeout). */
    private boolean driveDone() {
        double movedMm = forwardSinceDriveStart();
        // Distance still to go, in the direction of travel: + = not there yet, - = went past.
        double remainingMm = (driveMm - movedMm) * driveDirection;
        lastMovedMm = movedMm;
        lastRemainingMm = remainingMm;

        if (!driveStopped) {
            boolean arrived = remainingMm <= POSITION_TOLERANCE_MM;

            // Pods not counting: the robot is driving but the Pinpoint hasn't moved. Stop and say so.
            boolean notCounting = stateTimer.seconds() > NOT_COUNTING_SECONDS && Math.abs(movedMm) < NOT_COUNTING_MM;
            if (notCounting) {
                stopDrive();
                driveStopped = true;
                warning = state + ": Pinpoint is not counting - check the pod cables and the Pinpoint port.";
                log("%s STOP: Pinpoint not counting after %.1f s (moved %.1f mm)",
                        state, stateTimer.seconds(), movedMm);
                return true;
            }

            if (arrived) {
                stopDrive();
                driveStopped = true;
                arriveTimer.reset();
                log("%s STOP: forward %.0f of %.0f mm", state, movedMm, driveMm);
            } else {
                // Full DRIVE_SPEED until the last SLOW_DOWN_MM, then slow down (never below MIN_DRIVE_POWER).
                double speed = Range.clip(DRIVE_SPEED * remainingMm / SLOW_DOWN_MM, MIN_DRIVE_POWER, DRIVE_SPEED);

                // Stay straight with the Pinpoint heading: + correction = turn LEFT (right side faster).
                lastDriveError = headingError(targetHeading);
                double correction = Range.clip(lastDriveError * P_DRIVE_GAIN, -MAX_DRIVE_CORRECTION, MAX_DRIVE_CORRECTION);
                correction = Range.clip(correction, -speed, speed);
                lastCorrection = correction;

                double base = driveDirection * speed;
                setSidePower(Range.clip(base - correction, -1.0, 1.0),
                             Range.clip(base + correction, -1.0, 1.0));
            }
        }
        lastInTol = driveStopped;

        // Safety: never drive longer than moveTimeout.
        if (stateTimer.seconds() > moveTimeout) {
            stopDrive();
            warning = state + " timed out - the robot may be blocked.";
            log("%s TIMEOUT %.1f s: forward %.0f of %.0f mm, heading %.1f",
                    state, stateTimer.seconds(), movedMm, driveMm, poseHeadingDeg);
            return true;
        }

        // Done once stopped for ARRIVE_HOLD_SECONDS (the robot may roll a few mm after the stop).
        boolean done = driveStopped && arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
        if (done) {
            log("%s arrived %.1f s: forward %.0f of %.0f mm, heading %.1f",
                    state, stateTimer.seconds(), movedMm, driveMm, poseHeadingDeg);
        }
        return done;
    }

    /** Runs the LEAVE or TO_PARK drive each loop, and goes to the next step when it is done. */
    private void runDrive() {
        if (driveDone()) nextStep();
    }

    /** Power for the two sides: left front and back together, right front and back together. */
    private void setSidePower(double left, double right) {
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);
        lastPower = left;
        lastRightPower = right;
    }

    /** All four drive motors to power 0. */
    private void stopDrive() {
        setSidePower(0, 0);
    }

    // ======================================================================================
    // Turn in place (Pinpoint heading) - used by TURN_TO_PARK and FINAL_TURN
    // ======================================================================================

    private void startTurn(double headingDeg) {
        double turnDeg = headingDeg - targetHeading;
        double fromDeg = targetHeading;
        targetHeading = headingDeg;
        lastInTol = false;
        stopDrive();   // stop before the turn so no drive power carries into it
        turnStartError = Math.abs(headingError(targetHeading));
        arriveTimer.reset();
        log("turn %.0f -> %.0f deg (%+.0f, %s) by Pinpoint: now %.1f, error %.1f",
                fromDeg, headingDeg, turnDeg, turnDeg > 0 ? "LEFT" : "RIGHT", poseHeadingDeg, turnStartError);
    }

    private void runTurn() {
        if (turnDone()) nextStep();
    }

    /** Call every loop while turning. True once the heading is reached and held (or timed out). */
    private boolean turnDone() {
        double error = headingError(targetHeading);
        lastTurnError = error;

        // Safety: turning AWAY from the target means left/right are swapped or the Pinpoint is mounted wrong.
        if (Math.abs(error) > turnStartError + 30) {
            warning = "Turned the WRONG WAY - left/right motors swapped, or the Pinpoint is mounted the other way.";
            log("WRONG WAY: error %.1f > start %.1f + 30, heading %.1f -> stop",
                    Math.abs(error), turnStartError, poseHeadingDeg);
            finish();
            return false;
        }

        double power = Range.clip(error * P_TURN_GAIN, -MAX_TURN_SPEED, MAX_TURN_SPEED);
        if (Math.abs(error) > HEADING_TOLERANCE_DEG && Math.abs(power) < MIN_TURN_POWER) {
            power = Math.signum(error) * MIN_TURN_POWER;
        }
        lastTurnPower = power;
        // + power = turn LEFT: left wheels back, right wheels forward (same for all four wheels).
        setSidePower(-power, power);

        lastInTol = Math.abs(error) <= HEADING_TOLERANCE_DEG;
        if (!lastInTol) arriveTimer.reset();
        if (stateTimer.seconds() > TURN_TIMEOUT_SECONDS) {
            warning = state + " timed out at " + Math.round(poseHeadingDeg) + " deg.";
            log("%s TIMEOUT %.1f s: heading %.1f, error %.1f deg", state, stateTimer.seconds(), poseHeadingDeg, error);
            stopDrive();
            return true;
        }
        boolean done = arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
        if (done) {
            stopDrive();
            log("%s done %.1f s: heading %.1f, error %.1f deg", state, stateTimer.seconds(), poseHeadingDeg, error);
        }
        return done;
    }

    // ======================================================================================
    // Shooter helpers
    // ======================================================================================

    private void commandLauncher() {
        if (openLoop) {
            launcher.setPower(LAUNCHER_FALLBACK_POWER);
        } else {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }
    }

    private void feed(boolean on) {
        if (!shooterOk) return;
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
    // Pose and heading (from the Pinpoint)
    // ======================================================================================

    /** Once per loop: ask the Pinpoint for its latest reading and store x, y and heading. */
    private void readPose() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        poseXmm = pose.getX(DistanceUnit.MM);
        poseYmm = pose.getY(DistanceUnit.MM);
        poseHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
    }

    /** How far (degrees) the robot must still turn to reach 'target'. + = turn LEFT. */
    private double headingError(double target) {
        double error = target - poseHeadingDeg;
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    // ======================================================================================
    // Data recording: x, y, heading, drive power, launcher speed
    // ======================================================================================

    /** START: open a new CSV file named by the date and time, and write the header row. */
    private void openDataLog() {
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        String fileName = LOG_FOLDER + "/odo_" + stamp + ".csv";
        try {
            new File(LOG_FOLDER).mkdirs();
            dataFile = new FileWriter(fileName);
            dataFile.write(LOG_HEADER + "\n");
            dataFile.flush();
            log("data file: %s", fileName);
        } catch (IOException e) {
            dataFile = null;
            log("data file NOT opened (%s) - rows go to the Robot Log only", e.getMessage());
        }
        RobotLog.ii(LOG_TAG, LOG_HEADER);
        dataTimer.reset();
    }

    /** Every loop: write one row when LOG_INTERVAL_SEC has passed. Runs in every state, including DONE. */
    private void recordData() {
        if (dataTimer.seconds() < LOG_INTERVAL_SEC) return;
        dataTimer.reset();
        boolean driving = state == State.LEAVE || state == State.TO_PARK;
        String row = String.format(Locale.US,
                "%.2f,%s,%.1f,%.1f,%.2f,%.1f,%.0f,%.1f,%.3f,%.3f,%.0f",
                autoTimer.seconds(), state,
                poseXmm, poseYmm, poseHeadingDeg, targetHeading,
                driveMm, driving ? forwardSinceDriveStart() : 0.0,
                lastPower, lastRightPower,
                shooterOk ? launcher.getVelocity() : 0.0);
        RobotLog.ii(LOG_TAG, row);
        if (dataFile != null) {
            try {
                dataFile.write(row + "\n");
                dataFile.flush();
            } catch (IOException e) {
                dataFile = null;
                log("data file write failed (%s) - rows go to the Robot Log only", e.getMessage());
            }
        }
    }

    /** STOP: close the CSV file so every row is saved. */
    private void closeDataLog() {
        if (dataFile == null) return;
        try {
            dataFile.close();
        } catch (IOException e) {
            RobotLog.ee(LOG_TAG, "closing data file failed: %s", e.getMessage());
        }
        dataFile = null;
    }

    // ======================================================================================
    // Telemetry
    // ======================================================================================

    /** The whole Driver Station screen while running: step values, pose, motors, events. */
    private void showTelemetry(double dt) {
        telemetry.addData("State", "%s   %.1f s in step", state, stateTimer.seconds());
        telemetry.addData("Auto time", "%.1f / 30 s   (loop %.0f ms)", autoTimer.seconds(), dt * 1000);
        double launcherVel = shooterOk ? launcher.getVelocity() : 0;

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
            telemetry.addData("Fed seconds", "%.1f / %.1f", fedSeconds, BALLS_TO_SHOOT * FEED_SECONDS_PER_BALL);
        }
        if (pinpointOk) {
            telemetry.addData("Pose", "x %.0f mm (forward), y %.0f mm (left)", poseXmm, poseYmm);
            telemetry.addData("Heading / target", "%.1f / %.1f deg", poseHeadingDeg, targetHeading);
        } else {
            telemetry.addData("Pinpoint", "MISSING - distance and heading steps are skipped");
        }
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
        telemetry.addData("Steady at speed", "%.2f / %.2f s", readyTimer.seconds(), READY_HOLD_SECONDS);
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
        telemetry.addData("Drive", "%.0f mm %s by Pinpoint", driveMm, driveDirection < 0 ? "BACKWARD" : "FORWARD");
        telemetry.addData("Moved (forward)", "%.0f mm, %.0f mm to go  (slow down in the last %.0f)",
                lastMovedMm, lastRemainingMm, SLOW_DOWN_MM);
        telemetry.addData("Steer", "heading error %.1f deg -> correction %+.3f", lastDriveError, lastCorrection);
        telemetry.addData("Power L / R", "%+.2f / %+.2f", lastPower, lastRightPower);
        telemetry.addData("Stopped", "%s, settled %.2f / %.2f s   timeout %.1f / %.1f s",
                driveStopped ? "YES" : "no", driveStopped ? arriveTimer.seconds() : 0.0,
                ARRIVE_HOLD_SECONDS, stateTimer.seconds(), moveTimeout);
    }

    private void showTurnTelemetry() {
        telemetry.addData("Turn error", "%.1f deg   (start %.1f, tol %.1f)",
                lastTurnError, turnStartError, HEADING_TOLERANCE_DEG);
        telemetry.addData("Turn power", "%+.2f", lastTurnPower);
        telemetry.addData("On target", "%s, held %.2f / %.2f s   timeout %.1f / %.1f s",
                lastInTol ? "YES" : "no", lastInTol ? arriveTimer.seconds() : 0.0,
                ARRIVE_HOLD_SECONDS, stateTimer.seconds(), TURN_TIMEOUT_SECONDS);
    }

    /** The steps in ROUTE, e.g. "SPIN_UP > SHOOT > LEAVE". */
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
        } else {
            System.arraycopy(events, 1, events, 0, EVENT_LINES_KEPT - 1);
            events[EVENT_LINES_KEPT - 1] = line;
        }
    }

    /**
     * INIT odometry check. Push the robot BACKWARD 1 ft (toward the hive): x must go NEGATIVE.
     * Then push it LEFT by hand: y must go POSITIVE. Wrong sign = flip that pod's direction constant.
     */
    private void showOdometryCheck() {
        telemetry.addLine("---- ODOMETRY CHECK (Pinpoint) ----");
        if (!pinpointOk) {
            telemetry.addData("Pinpoint", "MISSING - LEAVE, TO_PARK and turns will be skipped");
            return;
        }
        double xMm = poseXmm - checkX;
        double yMm = poseYmm - checkY;
        telemetry.addLine("1) push the robot BACKWARD 1 ft (toward the hive), 2) push it LEFT");
        telemetry.addData("x (forward, front pod)", "%+.0f mm  %s", xMm, xVerdict(xMm));
        telemetry.addData("y (left, left pod)", "%+.0f mm  %s", yMm, yVerdict(yMm));

        String xv = xVerdict(xMm);
        String yv = yVerdict(yMm);
        String result;
        if (xv.startsWith("WRONG") || yv.startsWith("WRONG")) {
            result = "FAIL - fix the pod marked WRONG (flip its direction constant), then restart the OpMode";
        } else if (xv.startsWith("OK") && yv.startsWith("OK")) {
            result = "PASS - both pods count the right way";
        } else {
            result = "waiting for both pushes";
        }
        telemetry.addData("Odometry check", result);
        if (result.startsWith("PASS") || result.startsWith("FAIL")) {
            String kind = result.substring(0, 4);
            if (!kind.equals(checkResultLogged)) {
                checkResultLogged = kind;
                log("odometry check %s (x %.0f / y %.0f mm)", kind.trim(), xMm, yMm);
            }
        }
        telemetry.addData("Heading", "%.1f deg   (turn the robot LEFT by hand -> goes UP)", poseHeadingDeg);
        telemetry.addLine("gamepad1 A = start the check again from here");
    }

    private String xVerdict(double mm) {
        if (mm >= CHECK_MOVED_MM) return "WRONG: POSITIVE - flip FRONT_POD_DIRECTION";
        if (mm <= -CHECK_MOVED_MM) return "OK (backward = negative)";
        return "waiting";
    }

    private String yVerdict(double mm) {
        if (mm <= -CHECK_MOVED_MM) return "WRONG: NEGATIVE - flip LEFT_POD_DIRECTION";
        if (mm >= CHECK_MOVED_MM) return "OK (left = positive)";
        return "waiting";
    }

    /** Drive powers and the launcher / intake lines. */
    private void showMotors() {
        telemetry.addLine("---- Motors ----");
        telemetry.addData("Drive powers", "LF %+.2f  LB %+.2f  RF %+.2f  RB %+.2f",
                leftFront.getPower(), leftBack.getPower(), rightFront.getPower(), rightBack.getPower());
        if (shooterOk) {
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
        double vel = motor.getVelocity();
        String status = check.update(told, vel);
        telemetry.addData(label, "set %s | told %s | turning %s %+.0f t/s | %s",
                motor.getDirection(), dirWord(told),
                dirWord(Math.abs(vel) > DIRECTION_MIN_VELOCITY ? vel : 0), vel, status);
    }

    private String dirWord(double sign) {
        return sign > 0 ? "FWD" : sign < 0 ? "REV" : "stop";
    }

    /** Compares which way a motor is TOLD to turn with which way its encoder says it turns (launcher, intake). */
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
                    log("%s NOT TURNING: told %s, encoder %.0f t/s for %.1f s", name, dirWord(told), vel, stillTimer.seconds());
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
