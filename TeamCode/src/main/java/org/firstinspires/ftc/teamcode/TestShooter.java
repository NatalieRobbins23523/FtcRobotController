package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TestShooter extends LinearOpMode {

    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;
    private CRServo injector1;
    private CRServo injector2;
    private DcMotor transfer;
    private DcMotor intake;
    double Kp = 0.8*1/2800, Ki = 0.0, Kd = 0.00003; // Example coefficients
    double topIntegralSum = 0;
    double bottomIntegralSum = 0;
    double lastTopError = 0;
    double lastBottomError = 0;
    double targetTopVelocity = 1500;
    double targetBottomVelocity = 1500;
    double currentTopPower = 0.6;
    double currentBottomPower = 0.6;
    final double TICK_INCREMENT = 50;
    final double TICKS_PER_ROTATION = 28;
    final double MAX_RPM = 6000;
    final double MAX_TICKS_PER_SECOND = MAX_RPM * TICKS_PER_ROTATION / 60;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        injector1 = hardwareMap.get(CRServo.class, "transfer2");
        injector2 = hardwareMap.get(CRServo.class, "transfer3");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        topShooter.setDirection(DcMotor.Direction.REVERSE);

//        topShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("G1X:", "turns top shooter up");
        telemetry.addData("G1Y:", "turns top shooter down");
        telemetry.addData("G1A:", "turns bottom shooter up");
        telemetry.addData("G1B:", "turns bottom shooter down");
        telemetry.addData("G2X:", "toggles right injector");
        telemetry.addData("G2Y:", "toggles left injector");
        telemetry.addData("G2A:", "toggles transfer belt");
        telemetry.addData("G2B:", "toggles intake");
        telemetry.update();

        boolean isInjector1Turning = false;
        boolean isInjector2Turning = false;
        boolean isTransferTurning = false;
        boolean isIntakeTurning = false;

        waitForStart();
        topShooter.setPower(currentTopPower);
        sleep (200);
        bottomShooter.setPower(currentBottomPower);
        timer.reset();

        while (opModeIsActive()) {



            // ADJUST TARGET VELOCITY
            if (gamepad1.x) {
                targetTopVelocity = Math.min(targetTopVelocity + TICK_INCREMENT, MAX_TICKS_PER_SECOND);
                telemetry.addData("Top target (RPM, TPS):" , "%.0f, %.0f",
                        targetTopVelocity / TICKS_PER_ROTATION * 60, targetTopVelocity);
                telemetry.update();
                sleep(250);
            }
            else if (gamepad1.y) {
                targetTopVelocity = Math.min(targetTopVelocity - TICK_INCREMENT, MAX_TICKS_PER_SECOND);
                telemetry.addData("Top target (RPM, TPS):", "%.0f, %.0f",
                        targetTopVelocity / TICKS_PER_ROTATION * 60, targetTopVelocity);
                telemetry.update();
                sleep(250);
            }
            else if (gamepad1.a) {
                targetBottomVelocity = Math.min(targetBottomVelocity + TICK_INCREMENT, MAX_TICKS_PER_SECOND);
                telemetry.addData("Bottom target (RPM, TPS):" , "%.0f, %.0f",
                        targetBottomVelocity / TICKS_PER_ROTATION * 60, targetBottomVelocity);
                telemetry.update();
                sleep(250);
            }
            else if (gamepad1.b) {
                targetBottomVelocity = Math.min(targetBottomVelocity - TICK_INCREMENT, MAX_TICKS_PER_SECOND);
                telemetry.addData("Bottom target (RPM, TPS):" , "%.0f, %.0f",
                        targetBottomVelocity / TICKS_PER_ROTATION * 60, targetBottomVelocity);
                telemetry.update();
                sleep(250);
            }
            if (gamepad2.left_bumper) {
                isInjector1Turning = !isInjector1Turning;
                sleep(1000);
                turnBothInjectors(isInjector1Turning);
            }
            if (gamepad2.right_bumper) {
                injector1.setPower(0);
                injector2.setPower(0);
                sleep(250);
            }

            if (gamepad2.x) {
                isInjector1Turning = !isInjector1Turning;
                sleep(1000);
                turnInjector1(isInjector1Turning);
            }
            if (gamepad2.y) {
                isInjector2Turning = !isInjector2Turning;
                sleep(1000);
                turnInjector2(isInjector2Turning);
            }
            if (gamepad2.b) {
                isIntakeTurning = !isIntakeTurning;
                sleep(1000);
                turnIntake(isIntakeTurning);
            }
            if(gamepad2.a) {
                isTransferTurning = !isTransferTurning;
                sleep(1000);
                turnTransfer(isTransferTurning);
            }
            else if (gamepad1.left_stick_y < -0.5) {
                telemetry.addData("Top Target", targetTopVelocity);
                telemetry.addData("Top Actual", topShooter.getVelocity());
                telemetry.addData("Top Power", currentTopPower);
                telemetry.addData("Bottom Target", targetBottomVelocity);
                telemetry.addData("Bottom Actual", bottomShooter.getVelocity());
                telemetry.addData("Bottom Power", currentBottomPower);
                telemetry.update();
                sleep(2000);
            }

            // PID CONTROL - ADJUST ACTUAL VELOCITY

            double currentTopVelocity = topShooter.getVelocity();
            double currentBottomVelocity = bottomShooter.getVelocity();

            // Calculate error
            double topError = targetTopVelocity - currentTopVelocity;
            double bottomError = targetBottomVelocity - currentBottomVelocity;

            // Integral sum (with simple anti-windup)
            double dt = timer.seconds();
            timer.reset();
            topIntegralSum += topError * dt;
            bottomIntegralSum += bottomError * dt;

            // Use this if we need to limit the integral sum to avoid integral wind-up
//            if (integralSum > 1.0) integralSum = 1.0;
//            if (integralSum < -1.0) integralSum = -1.0;

            // Derivative (rate of change)
            double topDerivative = (topError - lastTopError) / dt;
            lastTopError = topError;

            double bottomDerivative = (bottomError - lastBottomError) / dt;
            lastBottomError = bottomError;

            // PID Formula: Output = P + I + D
            double topDeltaPower = (Kp * topError) + (Ki * topIntegralSum) + (Kd * topDerivative);
            currentTopPower += topDeltaPower;
            currentTopPower = Math.max(0.0, Math.min(currentTopPower, 1.0));
            topShooter.setPower(currentTopPower);

            double bottomDeltaPower = (Kp * bottomError) + (Ki * bottomIntegralSum) + (Kd * bottomDerivative);
            currentBottomPower += bottomDeltaPower;
            currentBottomPower = Math.max(0.0, Math.min(currentBottomPower, 1.0));
            bottomShooter.setPower(currentBottomPower);

            // Telemetry for debugging
            telemetry.addData("Top Target", targetTopVelocity);
            telemetry.addData("Top Actual", currentTopVelocity);
            telemetry.addData("Top Power", currentTopPower);
            telemetry.addData("Bottom Target", targetBottomVelocity);
            telemetry.addData("Bottom Actual", currentBottomVelocity);
            telemetry.addData("Bottom Power", currentBottomPower);
            telemetry.update();

            sleep(10);


            // alternative using built-in FTC PIDF Class - delete if not needed
//            PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
//            motorEx.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//
//            waitForStart();
//
//            if (opModeIsActive()) {
//                // Set the target velocity in ticks per second
//                // You will need to calculate the appropriate value for your motor/gearing
//                double targetVelocity = 1000; // Example target velocity
//
//                while (opModeIsActive()) {
//                    // Set the motor velocity using the built-in PID
//                    motorEx.setVelocity(targetVelocity);
//
//                    // Telemetry to monitor actual vs target velocity (useful for tuning)
//                    telemetry.addData("Target Velocity", targetVelocity);
//                    telemetry.addData("Actual Velocity", motorEx.getVelocity());
//                    telemetry.update();
//                }
//            }

        }
    }

    public void turnIntake(boolean on) {
        if (on) {
            intake.setPower(-0.60);
        } else {
            intake.setPower(0);
        }
    }

    public void turnTransfer(boolean on) {
        if (on) {
            transfer.setPower(-1);
        } else {
            transfer.setPower(0);
        }
    }

    public void turnInjector1(boolean on) {
        if (on) {
            injector1.setPower(1);
        }
        else {
            injector1.setPower(-0.03);
        }
    }

    public void turnInjector2 (boolean on) {
        if (on) {
            injector2.setPower(1);
        }
        else {
            injector2.setPower(-0.03);
        }
    }

    public void turnBothInjectors (boolean on) {
        if (on) {
            injector1.setPower(0.75);
            injector2.setPower(0.75);
        }
        else {
            injector1.setPower(-0.2);
            injector2.setPower(-0.2);
        }
    }
}
