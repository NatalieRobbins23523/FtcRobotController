package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class DriveMain extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor elevator;
    private DcMotor Shooter;
    private CRServo Intake;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(CRServo.class, "Intake");

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setDirection(DcMotor.Direction.REVERSE);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int numClicks = 1;
        boolean isReversed = false;
        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;
        boolean isShooterShooting = false;
        boolean isShooterShootingReverse = false;
        boolean shooterTriggerPressed = false;
        boolean reverseShooterTriggerPressed = false;


        waitForStart();

        while (opModeIsActive()) {
            // Toggle driving reverse
            if (gamepad1.right_bumper) {
                isReversed = !isReversed;
                sleep(300);
            }

            // Drive forward/backward
            double tgtPowerY = gamepad1.left_stick_y;
            if (Math.abs(tgtPowerY) < 0.5) {
                tgtPowerY /= 2;
            }
            driveStraight(tgtPowerY, isReversed);

            // Turning
            double tgtPowerTurn = gamepad1.right_stick_x;
            turn(tgtPowerTurn);

            // Strafing
            double tgtPowerSide = gamepad1.left_stick_x;
            if (Math.abs(tgtPowerSide) < 0.5) {
                tgtPowerSide /= 2;
            }
            driveSideways(tgtPowerSide, isReversed);

            // Elevator Controls
            if (gamepad1.x) {
                releaseElevator();
                numClicks++;
                sleep(200);
            }

            if (gamepad1.y) {
                raiseElevator();
            }

            if (gamepad1.left_bumper) {
                setElevatorZero();
                sleep(200);
            }

            // Intake Controls
            if (gamepad2.b) {
                isIntakeTurning = !isIntakeTurning;
                sleep(200);
                changeIntake(isIntakeTurning);
            }

            if (gamepad2.dpad_left || gamepad2.left_bumper) {
                isIntakeTurningReverse = !isIntakeTurningReverse;
                sleep(200);
                reverseIntake(isIntakeTurningReverse);
            }

            // Elevator Fine Adjustments
            int targetPosition = elevator.getCurrentPosition();
            if (gamepad2.dpad_up) {
                targetPosition += 50;
                elevator.setTargetPosition(targetPosition);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
            } else if (gamepad2.dpad_down) {
                targetPosition -= 50;
                elevator.setTargetPosition(targetPosition);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(1);
            } else if (gamepad2.dpad_right) {
                elevator.setPower(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            }

            // Shooter forward
            // Shooter forward
            if (gamepad2.right_trigger > 0.5 && !shooterTriggerPressed) {
                if (!isShooterShootingReverse) { // Prevent conflict
                    isShooterShooting = !isShooterShooting;
                    shootShooter(isShooterShooting);
                }
                shooterTriggerPressed = true;
            } else if (gamepad2.right_trigger <= 0.5) {
                shooterTriggerPressed = false;
            }

            // Shooter reverse
            if (gamepad2.left_trigger > 0.5 && !reverseShooterTriggerPressed) {
                if (!isShooterShooting) { // Prevent conflict
                    isShooterShootingReverse = !isShooterShootingReverse;
                    reverseShooter(isShooterShootingReverse);
                }
                reverseShooterTriggerPressed = true;
            } else if (gamepad2.left_trigger <= 0.5) {
                reverseShooterTriggerPressed = false;
            }

        }
        }

    // Drive Functions
    public void driveStraight(double power, boolean reversed) {
        double pwr = reversed ? -power : power;
        frontLeft.setPower(pwr);
        backLeft.setPower(pwr);
        frontRight.setPower(pwr);
        backRight.setPower(pwr);
    }

    public void turn(double direction) {
        frontLeft.setPower(-direction);
        backLeft.setPower(-direction);
        frontRight.setPower(direction);
        backRight.setPower(direction);
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

    // Elevator Functions
    public void raiseElevator() {
        elevator.setPower(1);
        elevator.setTargetPosition(3000);
    }

    public void releaseElevator() {
        elevator.setPower(1);
        elevator.setTargetPosition(400);
    }

    public void setElevatorZero() {
        elevator.setPower(1);
        elevator.setTargetPosition(0);
    }

    public void setElevatorGrabbingPosition() {
        if (elevator.getCurrentPosition() != 450) {
            elevator.setPower(1);
            elevator.setTargetPosition(450);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    // Intake Functions
    public void changeIntake(boolean isIntakeTurning) {
        if (isIntakeTurning) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }
    }

    public void reverseIntake(boolean isIntakeTurning) {
        if (isIntakeTurning) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }
    }

    public void shootShooter(boolean isShooterShooting) {
        if (isShooterShooting) {
            Shooter.setPower(1);
        } else {
            Shooter.setPower(0);
        }
    }

    public void reverseShooter(boolean isShooterShootingReverse) {
        if (isShooterShootingReverse) {
            Shooter.setPower(-1);
        } else {
            Shooter.setPower(0);
        }
    }
}
