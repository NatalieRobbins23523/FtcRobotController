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

    private CRServo IntakeLeft;
    private CRServo IntakeRight;

    private DcMotor MotorLeftShoot;
    private DcMotor MotorRightShoot;

    private CRServo TransferLeft;
    private CRServo TransferRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse right side
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        MotorLeftShoot = hardwareMap.get(DcMotor.class, "MotorLeftShoot");
        MotorRightShoot = hardwareMap.get(DcMotor.class, "MotorRightShoot");

        IntakeLeft = hardwareMap.get(CRServo.class, "IntakeLeft");
        IntakeRight = hardwareMap.get(CRServo.class, "IntakeRight");

        TransferLeft = hardwareMap.get(CRServo.class, "TransferLeft");
        TransferRight = hardwareMap.get(CRServo.class, "TransferRight");

        // Toggle states
        boolean isReversed = false;
        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;
        boolean isTransferTurning = false;
        boolean isTransferTurningReverse = false;

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
            if (gamepad2.b) {
                isIntakeTurning = !isIntakeTurning;
                isIntakeTurningReverse = false;
                sleep(200);
                changeIntake(isIntakeTurning);
            }

            // Intake toggle reverse
            if (gamepad2.a) {
                isIntakeTurningReverse = !isIntakeTurningReverse;
                isIntakeTurning = false;
                sleep(200);
                reverseIntake(isIntakeTurningReverse);
            }

            // Shooter basic control
            if (gamepad1.x) {
                setMotorLeftShootPower(-1.0);
                setMotorRightShootPower(1.0);
            }
            if (gamepad1.y) {
                setMotorLeftShootPower(0.0);
                setMotorRightShootPower(0.0);
            }

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
            IntakeLeft.setPower(1);
            IntakeRight.setPower(-1);
        } else {
            IntakeLeft.setPower(0);
            IntakeRight.setPower(0);
        }
    }

    public void reverseIntake(boolean on) {
        if (on) {
            IntakeLeft.setPower(-1);
            IntakeRight.setPower(1);
        } else {
            IntakeLeft.setPower(0);
            IntakeRight.setPower(0);
        }
    }

    // ---------- TRANSFER ----------
    public void turnTransfer(boolean on) {
        if (on) {
            TransferLeft.setPower(1);
            TransferRight.setPower(-1);
        } else {
            TransferLeft.setPower(0);
            TransferRight.setPower(0);
        }
    }

    public void turnTransferReverse(boolean on) {
        if(on) {
            TransferLeft.setPower(-1);
            TransferRight.setPower(1);
        } else {
            TransferLeft.setPower(0);
            TransferRight.setPower(0);
        }
    }



    // ---------- SHOOTER ----------
    public void setMotorLeftShootPower(double power) {
        MotorLeftShoot.setPower(power);
    }

    public void setMotorRightShootPower(double power) {
        MotorRightShoot.setPower(power);
    }
}
