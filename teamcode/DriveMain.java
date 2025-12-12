package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class DriveMain extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor bottomShooter;
    private DcMotor topShooter;
    private DcMotor transfer;
    private CRServo transfer2;
    private DcMotor intake;


    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        int numClicksX = 0;
        int numClicksY = 0;

        // Reverse right side
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        bottomShooter = hardwareMap.get(DcMotor.class, "bottomShooter");
        topShooter = hardwareMap.get(DcMotor.class, "topShooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
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
                sleep(250);
            }

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
                isIntakeTurningReverse = false;
                sleep(200);
                changeIntake(isIntakeTurning);
            }

            // Intake toggle reverse
            if (gamepad2.b) {
                isIntakeTurningReverse = !isIntakeTurningReverse;
                isIntakeTurning = false;
                sleep(200);
                reverseIntake(isIntakeTurningReverse);
            }

            if (gamepad1.x && !xLast) {             // runs ONLY on button press edge
                shooterForward = !shooterForward;   // toggle
                shooterReverse = false;             // disable reverse if forward is toggled

                if (shooterForward) {
                    turnShooter(0.8);               // forward full power
                } else {
                    turnShooter(0.0);               // stop
                }
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


            // Transfer toggle
            if (gamepad2.x) {
                isTransferTurning = !isTransferTurning;
                sleep(200);
                turnTransfer(isTransferTurning);
            }

            if (gamepad2.y) {
                isTransferTurningReverse = !isTransferTurningReverse;
                sleep(200);
                turnTransferReverse(isTransferTurningReverse);
            }

            // Transfer2 toggle
            if (gamepad2.left_bumper) {
                isTransfer2Turning = !isTransfer2Turning;
                sleep(200);
                turnTransfer2(isTransfer2Turning);
            }

            if (gamepad2.right_bumper) {
                isTransfer2TurningReverse = !isTransfer2TurningReverse;
                sleep(200);
                turnTransfer2Reverse(isTransfer2TurningReverse);
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
    public void turnTransfer(boolean on) {
        if (on) {
            transfer.setPower(.60);
        } else {
            transfer.setPower(0);
        }
    }

    public void turnTransferReverse(boolean on) {
        if (on) {
            transfer.setPower(-.60);
        } else {
            transfer.setPower(0);
        }
    }

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
