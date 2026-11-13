package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * SHOOTER TEST - launcher + windmill (+ intake assist) only. Drive motors are never touched.
 * Keep clear of the launcher exit; balls will fly if you feed them.
 *
 *   Right bumper (hold)   launcher spins up to target velocity
 *   Right trigger (hold)  feed: windmill + intake, but ONLY when launcher is at speed
 *   Left bumper           override: feed even if not at speed
 *   D-pad up / down       target velocity +/- 50 ticks/s
 *   B                     launcher reverse (clear a jam), hold
 *
 * Telemetry shows target vs actual velocity and an AT SPEED flag.
 * Values match BioBuzzStarterbotAuto (target 1250, min 1200, PIDF 40/0/0/12.5).
 */
@TeleOp(name = "ShooterTest", group = "Test")
public class ShooterTest extends OpMode {

    final int MIN_MARGIN = 50;             // "at speed" = within this of target (BioBuzzStarterbotAuto: 1250 vs 1200)
    final double FEED_INTAKE_POWER = 0.5;

    // Intake motor direction. If the intake spins the wrong way, flip this one line.
    final DcMotorSimple.Direction INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Launcher (shooter) motor direction. If it spins the wrong way, flip this one line.
    final DcMotorSimple.Direction LAUNCHER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    DcMotorEx launcher;
    DcMotor intake;
    CRServo windmill;

    double target = 1250;
    boolean prevUp, prevDown;

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
        telemetry.addLine("RB = spin up, RT = feed, LB = force feed, D-pad = target, B = reverse");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !prevUp) target += 50;
        if (gamepad1.dpad_down && !prevDown) target -= 50;
        prevUp = gamepad1.dpad_up;
        prevDown = gamepad1.dpad_down;
        target = Math.max(0, Math.min(2000, target));

        double velocity = 0;
        if (launcher != null) {
            if (gamepad1.b) {
                launcher.setVelocity(-600);
            } else if (gamepad1.right_bumper) {
                launcher.setVelocity(target);
            } else {
                launcher.setVelocity(0);
            }
            velocity = launcher.getVelocity();
        }

        boolean atSpeed = launcher != null && gamepad1.right_bumper
                && Math.abs(velocity) >= target - MIN_MARGIN;
        boolean feed = (gamepad1.right_trigger > 0.3 && atSpeed) || gamepad1.left_bumper;

        if (windmill != null) windmill.setPower(feed ? 1.0 : 0.0);
        if (intake != null) intake.setPower(feed ? FEED_INTAKE_POWER : 0.0);

        telemetry.addData("Target vel", "%.0f", target);
        telemetry.addData("Actual vel", "%.0f", velocity);
        telemetry.addData("AT SPEED", atSpeed ? "YES" : "no");
        telemetry.addData("Feeding", feed ? "YES" : "no");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (launcher != null) launcher.setPower(0);
        if (windmill != null) windmill.setPower(0);
        if (intake != null) intake.setPower(0);
    }
}
