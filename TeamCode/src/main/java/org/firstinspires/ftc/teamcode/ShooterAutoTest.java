package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * SHOOTER AUTO TEST - no gamepad needed. Press INIT, then START. Drive motors are never touched.
 *
 *   1. SPIN_UP  launcher to target velocity (fails the test if not at speed within SPIN_UP_TIMEOUT)
 *   2. FEED     windmill + intake run for FEED_SECONDS (only if launcher is at speed)
 *   3. COOLDOWN everything off, then DONE with a PASS / FAIL summary on the Driver Station
 *
 * !! If balls are loaded, they WILL be launched. Leave the balls out for a first test, and keep clear
 * of the launcher exit. Press STOP at any time to cut everything.
 * Values match BioBuzzStarterbotAuto (target 1250, min 1200, PIDF 40/0/0/12.5, windmill REVERSE, intake 0.5).
 */
@Autonomous(name = "ShooterAutoTest", group = "Test")
public class ShooterAutoTest extends OpMode {

    final int TARGET_VELOCITY = 1250;
    final int MIN_VELOCITY = 1200;
    final double SPIN_UP_TIMEOUT = 5.0;
    final double FEED_SECONDS = 3.0;
    final double FEED_INTAKE_POWER = 0.5;

    enum State { SPIN_UP, FEED, DONE }

    // Intake motor direction. If the intake spins the wrong way, flip this one line.
    final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Launcher (shooter) motor direction. If it spins the wrong way, flip this one line.
    final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    DcMotorEx launcher;
    DcMotor intake;
    CRServo windmill;
    final ElapsedTime timer = new ElapsedTime();
    State state = State.SPIN_UP;

    double peakVelocity = 0;
    boolean reachedSpeed = false;
    String result = "";

    @Override
    public void init() {
        try {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setDirection(LAUNCHER_DIRECTION);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(40, 0, 0, 12.5));
        } catch (Exception e) {
            launcher = null;
        }
        try {
            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setDirection(INTAKE_DIRECTION);
        } catch (Exception e) {
            intake = null;
        }
        try {
            windmill = hardwareMap.get(CRServo.class, "windmill");
            windmill.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            windmill = null;
        }
        telemetry.addData("launcher", launcher != null ? "OK" : "NOT FOUND");
        telemetry.addData("intake", intake != null ? "OK" : "NOT FOUND");
        telemetry.addData("windmill", windmill != null ? "OK" : "NOT FOUND");
        telemetry.addLine("Press START to run. Balls out, clear of launcher!");
    }

    @Override
    public void init_loop() {
        telemetry.addData("launcher", launcher != null ? "OK" : "NOT FOUND");
        telemetry.addData("intake", intake != null ? "OK" : "NOT FOUND");
        telemetry.addData("windmill", windmill != null ? "OK" : "NOT FOUND");
    }

    @Override
    public void start() {
        timer.reset();
        if (launcher == null) {
            finish("FAIL: launcher not found in config");
        }
    }

    @Override
    public void loop() {
        double velocity = launcher != null ? launcher.getVelocity() : 0;
        peakVelocity = Math.max(peakVelocity, Math.abs(velocity));
        boolean atSpeed = Math.abs(velocity) >= MIN_VELOCITY;

        switch (state) {
            case SPIN_UP:
                launcher.setVelocity(TARGET_VELOCITY);
                if (atSpeed) {
                    reachedSpeed = true;
                    timer.reset();
                    state = State.FEED;
                } else if (timer.seconds() > SPIN_UP_TIMEOUT) {
                    finish(peakVelocity < 10
                            ? "FAIL: launcher never moved (power/encoder/config?)"
                            : "FAIL: launcher too slow, peak " + (int) peakVelocity);
                }
                break;

            case FEED:
                launcher.setVelocity(TARGET_VELOCITY);
                if (windmill != null) windmill.setPower(1.0);
                if (intake != null) intake.setPower(FEED_INTAKE_POWER);
                if (timer.seconds() >= FEED_SECONDS) {
                    String missing = (windmill == null ? " (windmill NOT FOUND)" : "")
                            + (intake == null ? " (intake NOT FOUND)" : "");
                    finish(missing.isEmpty() ? "PASS" : "PARTIAL PASS" + missing);
                }
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Time", "%.1f s", timer.seconds());
        telemetry.addData("Target vel", TARGET_VELOCITY);
        telemetry.addData("Actual vel", "%.0f", velocity);
        telemetry.addData("Peak vel", "%.0f", peakVelocity);
        telemetry.addData("AT SPEED", atSpeed ? "YES" : "no");
        if (state == State.DONE) telemetry.addData("RESULT", result);
        telemetry.update();
    }

    private void finish(String message) {
        result = message;
        state = State.DONE;
        allOff();
    }

    private void allOff() {
        if (launcher != null) launcher.setPower(0);
        if (windmill != null) windmill.setPower(0);
        if (intake != null) intake.setPower(0);
    }

    @Override
    public void stop() {
        allOff();
    }
}
