package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class TestDriveMain extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor bottomShooter;
    private DcMotor topShooter;
    // private CRServo transfer;
    private CRServo transfer2;
    private DcMotor intake;


    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        int numClicksX = 0;
        int numClicksY = 0;


        bottomShooter = hardwareMap.get(DcMotor.class, "bottomShooter");
        topShooter = hardwareMap.get(DcMotor.class, "topShooter");
        // transfer = hardwareMap.get(CRServo.class, "transfer");
        transfer2 = hardwareMap.get(CRServo.class, "transfer2");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Toggle states
        boolean isReversed = false;
        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;
        boolean isTransferTurning = false;
        boolean isTransferTurningReverse = false;
        boolean isTransfer2Turning = false;
        boolean isTransfer2TurningReverse = false;
        boolean xLast = false;
        boolean yLast = false;

        boolean shooterForward = false;
        boolean shooterReverse = false;
        waitForStart();

        while (opModeIsActive()) {

            // Toggle driving reverse
            if (gamepad1.right_bumper) {
                isReversed = !isReversed;
                telemetry.addData("MainLoop", "Right Bumper Pressed. Reversing");
                telemetry.update();
                sleep(5000);
            }

            // Drive forward/backward
            double tgtPowerY = -gamepad1.left_stick_y; // gamepad returns negative value when pushed forward
            if (Math.abs(tgtPowerY) < 0.5) tgtPowerY /= 2;
            telemetry.addData("MainLoop", "tgtPowerY: %5.2f, ", tgtPowerY);
            driveStraight(tgtPowerY, isReversed);
            // Turning
            turn(gamepad1.right_stick_x);

            // Strafing
            double tgtPowerSide = -gamepad1.left_stick_x; // gamepad returns positive value when pushed right but positive strafe is left
            if (Math.abs(tgtPowerSide) < 0.5) tgtPowerSide /= 2;
            telemetry.addData("MainLoop", "tgtPowerX: %5.2f, ", tgtPowerY);
            driveSideways(tgtPowerSide, isReversed);
            telemetry.update();
            sleep(500);

            // Intake toggle forward
            if (gamepad2.a) {
                frontLeft.setPower(0.25);
                sleep(1000);
                frontLeft.setPower(0);
            }

            // Intake toggle reverse
            if (gamepad2.b) {
               frontRight.setPower(0.25);
                sleep(1000);
                frontRight.setPower(0);
            }

            if (gamepad2.x) {
               backLeft.setPower(0.25);
               sleep(1000);
               backLeft.setPower(0);
            }

            if (gamepad2.y) {
                backRight.setPower(0.25);
                sleep(1000);
                backRight.setPower(0);
            }
            if (gamepad2.left_bumper) {
                topShooter.setPower(0.25);
                sleep(2000);
                topShooter.setPower(0);
            }

            if(gamepad2.right_bumper) {
                bottomShooter.setPower(0.25);
                sleep(2000);
                bottomShooter.setPower(0);
            }
            xLast = gamepad1.x;


// Y = toggle shooter reverse
            if (gamepad1.y && !yLast) {
                shooterReverse = !shooterReverse;   // toggle
                shooterForward = false;             // disable forward if reverse toggled

                if (shooterReverse) {
                    turnShooter(-0.5);              // reverse slow
                } else {
                    turnShooter(0.0);               // stop
                }
            }
            yLast = gamepad1.y;

        }
    }


    // ---------- DRIVE ----------
    public void driveStraight(double power, boolean reversed) {
        double p = reversed ? -power : power;
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);
    }

    public void turn(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void driveSideways(double speed, boolean reversed) {
        double fl = reversed ? speed : -speed;
        double bl = reversed ? -speed : speed;
        double fr = reversed ? -speed : speed;
        double br = reversed ? speed : -speed;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    // ---------- INTAKE ----------
    public void changeIntake(boolean on) {
        if (on) {
            intake.setPower(0.60);
        } else {
            intake.setPower(0);
        }
    }

    public void reverseIntake(boolean on) {
        if (on) {
            intake.setPower(-.60);
        } else {
            intake.setPower(0);
        }
    }

    // ---------- TRANSFER ----------
//    public void turnTransfer(boolean on) {
//        if (on) {
//            transfer.setPower(.60);
//        } else {
//            transfer.setPower(0);
//        }
//    }

//    public void turnTransferReverse(boolean on) {
//        if (on) {
//            transfer.setPower(-.60);
//        } else {
//            transfer.setPower(0);
//        }
//    }

    public void turnTransfer2(boolean on) {
        if (on) {
            transfer2.setPower(1);
        } else {
            transfer2.setPower(0);
        }
    }

    public void turnTransfer2Reverse(boolean on) {
        if (on) {
            transfer2.setPower(-1);
        } else {
            transfer2.setPower(0);
        }
    }


    // ---------- SHOOTER ----------
    public void turnShooter (double power) {
        topShooter.setPower(-power);
        bottomShooter.setPower(power);
    }

}
