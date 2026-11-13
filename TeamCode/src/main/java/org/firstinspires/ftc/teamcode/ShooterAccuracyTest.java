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
 * SHOOTER ACCURACY TEST - fixed shooting spot, no camera, no gamepad.
 * Place the robot on its start mark, facing the way it faced when you measured SHOT_MM / SHOT_HEADING_DEG.
 * INIT, then START:
 *
 *   1. DRIVE    drive SHOT_MM to the shooting spot (encoders + IMU to stay straight)
 *   2. TURN     turn to SHOT_HEADING_DEG so the launcher points at the hive (IMU)
 *   3. SETTLE   stand still so the robot stops rocking before the first ball
 *   4. SPIN_UP  launcher to SHOT_VELOCITY and hold within VELOCITY_TOLERANCE for READY_HOLD_SECONDS
 *   5. FEED     feed ONE ball; the velocity dip tells us it left the launcher
 *   6. RECOVER  wait for the launcher to climb back to speed, then feed the next ball
 *   7. DONE     everything off; summary with the launch velocity of each ball
 *
 * Why this is accurate: every ball leaves at the same launcher speed (never fed while slow), the robot
 * is stationary and pointed by the IMU (not by eye), and the ball count is detected from the velocity dip
 * instead of a fixed timer.
 *
 * !! TUNE ON THE FIELD (the numbers below are placeholders): SHOT_MM, SHOT_HEADING_DEG, SHOT_VELOCITY.
 *    Shoot, see where balls land, adjust ONE value, repeat. Farther from the hive = higher SHOT_VELOCITY.
 *    Too far left/right = SHOT_HEADING_DEG (+ = left/counter-clockwise).
 * !! Balls WILL be launched. Keep clear of the launcher exit. STOP cuts everything.
 */
@Autonomous(name = "ShooterAccuracyTest", group = "Test")
public class ShooterAccuracyTest extends OpMode {

    // ================= SHOT SETUP (measure/tune on the field) =================
    final double SHOT_MM = 0;               // + = forward, - = backward from the start mark (0 = shoot from the start)
    final double SHOT_HEADING_DEG = 0;      // heading relative to START (+ = left). Aim at the hive.
    final int SHOT_VELOCITY = 1250;         // launcher ticks/sec - the main distance knob
    final int BALLS = 4;

    // ================= Shooting behavior =================
    final int VELOCITY_TOLERANCE = 30;      // "at speed" = within this of SHOT_VELOCITY
    final int DIP_TICKS = 80;               // velocity drop that means a ball just left
    final double READY_HOLD_SECONDS = 0.3;  // must stay at speed this long before feeding
    final double SETTLE_SECONDS = 0.5;
    final double SPIN_UP_TIMEOUT = 5.0;
    final double FEED_TIMEOUT = 2.0;        // no dip detected: ball didn't leave, stop feeding it
    final double RECOVER_TIMEOUT = 3.0;
    final double FEED_INTAKE_POWER = 0.5;
    final boolean USE_INTAKE_ASSIST = false; // false = ONLY the windmill pushes the pollen; true = intake helps too
    final int FEED_FLOOR = 150;             // safety: windmill stops if the launcher falls more than this below target

    // ================= Config names / directions (same as BioBuzzStarterbotAuto) =================
    final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final DcMotorSimple.Direction WINDMILL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ================= Driving =================
    final double TICKS_PER_MM = 537.7 / (96 * Math.PI);
    final double DRIVE_SPEED = 0.5;
    final double P_DRIVE_GAIN = 0.03;
    final double POSITION_TOLERANCE_MM = 10;
    final double MAX_TURN_SPEED = 0.4;
    final double MIN_TURN_POWER = 0.08;
    final double P_TURN_GAIN = 0.02;
    final double HEADING_TOLERANCE_DEG = 1.0;   // tighter than the auto: aim matters here
    final double ARRIVE_HOLD_SECONDS = 0.25;
    final double TURN_TIMEOUT_SECONDS = 3.0;

    // Intake motor direction. If the intake spins the wrong way, flip this one line.
    final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Launcher (shooter) motor direction. If it spins the wrong way, flip this one line.
    final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    enum State { DRIVE, TURN, SETTLE, SPIN_UP, FEED, RECOVER, DONE }

    DcMotor leftDrive, rightDrive, intake;
    DcMotorEx launcher;
    CRServo windmill;
    IMU imu;
    boolean imuOk, shooterOk, windmillOk;

    State state = State.DRIVE;
    final ElapsedTime stateTimer = new ElapsedTime();
    final ElapsedTime readyTimer = new ElapsedTime();
    final ElapsedTime arriveTimer = new ElapsedTime();

    int leftTarget, rightTarget;
    double moveTimeout;
    int ballsFired = 0;
    double[] launchVelocity = new double[16];   // velocity just before each ball left
    double velocityBeforeFeed = 0;
    double minVelocityThisBall = 0;
    boolean windmillPushing = false;
    String result = "";
    String warning = "";

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(LEFT_DIRECTION);
        rightDrive.setDirection(RIGHT_DIRECTION);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(INTAKE_DIRECTION);
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setDirection(LAUNCHER_DIRECTION);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
            shooterOk = true;
        } catch (Exception e) {
            shooterOk = false;
        }
        try {
            windmill = hardwareMap.get(CRServo.class, "windmill");
            windmill.setDirection(WINDMILL_DIRECTION);
            windmillOk = true;
        } catch (Exception e) {
            windmillOk = false;
        }
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
            imu.resetYaw();
            imuOk = true;
        } catch (Exception e) {
            imuOk = false;
        }
    }

    @Override
    public void init_loop() {
        telemetry.addData("Launcher/intake", shooterOk ? "OK" : "MISSING - cannot run");
        telemetry.addData("Windmill", windmillOk ? "OK" : "MISSING");
        telemetry.addData("IMU", imuOk ? "OK" : "MISSING - cannot aim");
        telemetry.addData("Shot", "%.0f mm, heading %.0f deg, velocity %d, %d balls",
                SHOT_MM, SHOT_HEADING_DEG, SHOT_VELOCITY, BALLS);
        telemetry.addLine("Balls will be LAUNCHED. Stay clear. Press START.");
    }

    @Override
    public void start() {
        if (!shooterOk || !imuOk) {
            finish("FAIL: " + (!shooterOk ? "launcher/intake not found. " : "")
                    + (!imuOk ? "IMU not found." : ""));
            return;
        }
        if (SHOT_MM != 0) {
            startDrive(SHOT_MM);
            enter(State.DRIVE);
        } else {
            startTurn();
            enter(State.TURN);
        }
    }

    @Override
    public void loop() {
        double velocity = shooterOk ? launcher.getVelocity() : 0;
        double error = imuOk ? headingError(SHOT_HEADING_DEG) : 0;
        boolean atSpeed = Math.abs(velocity - SHOT_VELOCITY) <= VELOCITY_TOLERANCE;

        switch (state) {
            case DRIVE:
                if (driveDone()) {
                    stopDrive();
                    startTurn();
                    enter(State.TURN);
                }
                break;

            case TURN:
                if (turnDone(error)) {
                    stopDrive();
                    enter(State.SETTLE);
                }
                break;

            case SETTLE:
                // Spin the launcher up while the robot stops rocking.
                launcher.setVelocity(SHOT_VELOCITY);
                if (stateTimer.seconds() >= SETTLE_SECONDS) enter(State.SPIN_UP);
                break;

            case SPIN_UP:
            case RECOVER:
                launcher.setVelocity(SHOT_VELOCITY);
                feed(false);
                windmillPushing = false;
                if (!atSpeed) readyTimer.reset();
                if (readyTimer.seconds() >= READY_HOLD_SECONDS) {
                    // Re-check aim before every ball; a bump or recoil can move the robot.
                    if (Math.abs(error) > 3 * HEADING_TOLERANCE_DEG) {
                        warning = "Aim drifted " + String.format("%.1f", error) + " deg - re-turning";
                        startTurn();
                        enter(State.TURN);
                    } else if (ballsFired >= BALLS) {
                        finish("PASS: " + ballsFired + " balls fired");
                    } else {
                        velocityBeforeFeed = velocity;
                        minVelocityThisBall = velocity;
                        enter(State.FEED);
                    }
                } else if (stateTimer.seconds() > (state == State.SPIN_UP ? SPIN_UP_TIMEOUT : RECOVER_TIMEOUT)) {
                    finish("FAIL: launcher not at speed (" + (int) velocity + " vs " + SHOT_VELOCITY
                            + ") after " + ballsFired + " balls");
                }
                break;

            case FEED:
                launcher.setVelocity(SHOT_VELOCITY);
                // The windmill pushes pollen into the launcher ONLY while the launcher is fast enough.
                // (Entered at speed; the floor stops pushing if the launcher stalls or sags.)
                windmillPushing = velocity >= SHOT_VELOCITY - FEED_FLOOR;
                feed(windmillPushing);
                minVelocityThisBall = Math.min(minVelocityThisBall, velocity);
                if (velocityBeforeFeed - minVelocityThisBall >= DIP_TICKS) {
                    // The ball left the launcher.
                    if (ballsFired < launchVelocity.length) launchVelocity[ballsFired] = velocityBeforeFeed;
                    ballsFired++;
                    feed(false);
                    readyTimer.reset();
                    enter(State.RECOVER);
                } else if (stateTimer.seconds() > FEED_TIMEOUT) {
                    feed(false);
                    finish("FAIL: no velocity dip after feeding ball " + (ballsFired + 1)
                            + " - jam, empty hopper, or windmill/intake not feeding");
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", "%s   balls %d/%d", state, ballsFired, BALLS);
        telemetry.addData("Launcher", "%.0f / %d  %s", velocity, SHOT_VELOCITY, atSpeed ? "AT SPEED" : "");
        telemetry.addData("Heading", "%.1f deg (target %.1f, error %.1f)",
                currentHeading(), SHOT_HEADING_DEG, error);
        telemetry.addData("Battery", "%.1f V", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Windmill", state == State.FEED && windmillPushing ? "PUSHING POLLEN"
                : state == State.FEED ? "HELD - launcher too slow" : "off (waiting for launcher speed)");
        for (int i = 0; i < ballsFired && i < launchVelocity.length; i++) {
            telemetry.addData("Ball " + (i + 1), "left at %.0f ticks/s", launchVelocity[i]);
        }
        if (warning.length() > 0) telemetry.addData("WARNING", warning);
        if (state == State.DONE) telemetry.addData("RESULT", result);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopDrive();
        allShooterOff();
    }

    // ---------------- Helpers ----------------

    private void enter(State next) {
        state = next;
        stateTimer.reset();
        arriveTimer.reset();
        if (next == State.SPIN_UP || next == State.RECOVER) readyTimer.reset();
    }

    private void finish(String message) {
        result = message;
        stopDrive();
        allShooterOff();
        state = State.DONE;
    }

    private void feed(boolean on) {
        if (windmillOk) windmill.setPower(on ? 1.0 : 0.0);
        intake.setPower(on && USE_INTAKE_ASSIST ? FEED_INTAKE_POWER : 0.0);
    }

    private void allShooterOff() {
        if (windmillOk) windmill.setPower(0);
        if (intake != null) intake.setPower(0);
        if (launcher != null) launcher.setPower(0);
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double currentHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /** Degrees still to turn to reach 'target'. + = turn LEFT. */
    private double headingError(double target) {
        double error = target - currentHeading();
        while (error > 180) error -= 360;
        while (error <= -180) error += 360;
        return error;
    }

    private void startDrive(double mm) {
        int ticks = (int) Math.round(mm * TICKS_PER_MM);
        leftTarget = leftDrive.getCurrentPosition() + ticks;
        rightTarget = rightDrive.getCurrentPosition() + ticks;
        moveTimeout = Math.max(3.0, Math.abs(mm) / 250.0);
        // SDK order: target -> RUN_TO_POSITION -> power
        leftDrive.setTargetPosition(leftTarget);
        rightDrive.setTargetPosition(rightTarget);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(DRIVE_SPEED);
        rightDrive.setPower(DRIVE_SPEED);
    }

    private boolean driveDone() {
        // Hold the START heading (0) while driving straight.
        double dir = SHOT_MM < 0 ? -1 : 1;
        double correction = Range.clip(headingError(0) * P_DRIVE_GAIN, -0.2, 0.2) * dir;
        leftDrive.setPower(Range.clip(DRIVE_SPEED - correction, 0.05, 1.0));
        rightDrive.setPower(Range.clip(DRIVE_SPEED + correction, 0.05, 1.0));

        double tol = POSITION_TOLERANCE_MM * TICKS_PER_MM;
        boolean inTol = Math.abs(leftTarget - leftDrive.getCurrentPosition()) <= tol
                && Math.abs(rightTarget - rightDrive.getCurrentPosition()) <= tol;
        if (!inTol) arriveTimer.reset();
        if (stateTimer.seconds() > moveTimeout) {
            warning = "Drive to shooting spot timed out - check encoders / obstruction." +
                    "WARNING: BALL MIGHT SHOOT ONTO USERS FACE AT HIGH SPEED VELOCITY, THIS IS AN INTENDED FEATURE, NOT A BUG, FOR MORE INFO PLEASE";
            return true;
        }
        return arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
    }

    private void startTurn() {
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arriveTimer.reset();
    }

    /** Turn in place by the IMU. True once on target for ARRIVE_HOLD_SECONDS (or timed out). */
    private boolean turnDone(double error) {
        if (Math.abs(error) <= HEADING_TOLERANCE_DEG) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        } else {
            arriveTimer.reset();
            double p = Range.clip(Math.abs(error) * P_TURN_GAIN, MIN_TURN_POWER, MAX_TURN_SPEED)
                    * Math.signum(error);
            // + error = turn left: left wheels back, right wheels forward.
            leftDrive.setPower(-p);
            leftDrive.setPower(+p);
            leftDrive.setPower(-p);
            rightDrive.setPower(p);
            
        }
        if (stateTimer.seconds() > TURN_TIMEOUT_SECONDS) {
            warning = "Turn timed out at " + String.format("%.1f", error) + " deg error.";
            return true;
        }
        return arriveTimer.seconds() >= ARRIVE_HOLD_SECONDS;
    }
}
