package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp
public class DriveMain2 extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor motorIntake;

    private DcMotor motorShoot;

    private DcMotor motorTransfer;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse right side
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //backRight.setDirection(DcMotor.Direction.REVERSE);

        motorShoot = hardwareMap.get(DcMotor.class, "Shooter");

        motorIntake = hardwareMap.get(DcMotor.class, "Intake");

        motorTransfer = hardwareMap.get(DcMotor.class, "Transfer");

        // Toggle states
        boolean isReversed = false;
        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;
        boolean isTransferTurning = false;
        boolean isTransferTurningReverse = false;
        boolean isShooterTurning = false;

        double shooterPower = 1;

        waitForStart();

        while (opModeIsActive()) {

            // Toggle driving reverse
//            if (gamepad1.right_bumper) {
//                isReversed = !isReversed;
//                sleep(250);
//            }

            // Drive forward/backward
            double tgtPowerY = gamepad1.left_stick_y;
            if (Math.abs(tgtPowerY) < 0.5) tgtPowerY /= 2;
            driveStraight(tgtPowerY, isReversed);

            // Turning
            turn(gamepad1.right_stick_x);

            // Strafing
            double tgtPowerSide = gamepad1.left_stick_x;
            if (Math.abs(tgtPowerSide) < 0.5) tgtPowerSide /= 2;
            driveSideways(tgtPowerSide, isReversed);

            // Intake toggle forward
            if (gamepad2.a) {
                isIntakeTurning = !isIntakeTurning;
                sleep(200);
                toggleIntake(isIntakeTurning);
            }

       //      Intake toggle reverse
            if (gamepad2.b) {
                isIntakeTurningReverse = !isIntakeTurningReverse;
                isIntakeTurning = false;
                sleep(200);
                reverseIntake(isIntakeTurningReverse);
            }

            // Toggle shooter on/off
            if (gamepad1.a) {
                isShooterTurning = !isShooterTurning;
                sleep(200);
                changeShooter(isShooterTurning, shooterPower);
            }
            // reset shooter power to full power
            if (gamepad1.left_bumper){
                shooterPower = 1;
                changeShooter(isShooterTurning, shooterPower);
            }
//            if (gamepad1.y) {
//                setMotorLeftShootPower(0.0);
//            }
            //  turn down shooter power
            if (gamepad1.right_bumper) {
                shooterPower = shooterPower - 0.05;
                if (shooterPower < 0) {
                    shooterPower = 0;
                }
                changeShooter(isShooterTurning, shooterPower);
            }

            // hold down to turn transfer forward
            if (gamepad2.x) {
                isTransferTurning = true;
                turnTransfer(isTransferTurning);
            }
            else if (gamepad2.y) {
                isTransferTurningReverse = true;
                turnTransferReverse(isTransferTurningReverse);
            }
            else {
                isTransferTurning = false;
                isTransferTurningReverse = false;
                turnTransfer(isTransferTurning);
            }
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
    public void toggleIntake(boolean on) {
        if (on) {
            motorIntake.setPower(1);
        } else {
            motorIntake.setPower(0);
        }
    }

    public void changeShooter(boolean on, double power) {
        if (on) {
            motorShoot.setPower(-power);
        } else {
            motorShoot.setPower(0);
        }
    }


    public void reverseIntake(boolean on) {
        if (on) {
            motorIntake.setPower(-1);
        } else {
            motorIntake.setPower(0);
        }
    }

    // ---------- TRANSFER ----------
    public void turnTransfer(boolean on) {
        if (on) {
            motorTransfer.setPower(1);
        } else {
            motorTransfer.setPower(0);
        }
    }

    public void turnTransferReverse (boolean on) {
        if(on) {
            motorTransfer.setPower(-1);
        }
        else {
            motorTransfer.setPower(0);
        }
    }





    // ---------- SHOOTER ----------
    public void setMotorShootPower(double power) {
        motorShoot.setPower(power);
    }

}


