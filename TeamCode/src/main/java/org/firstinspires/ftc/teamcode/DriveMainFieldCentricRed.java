package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class DriveMainFieldCentricRed extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor bottomShooter;
    private DcMotor topShooter;
    private DcMotor transfer;
    private CRServo injector1;
    private CRServo injector2;
    private DcMotor intake;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer



    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        int numClicksX = 0;
        int numClicksY = 0;

        bottomShooter = hardwareMap.get(DcMotor.class, "bottomShooter");
        topShooter = hardwareMap.get(DcMotor.class, "topShooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        injector1 = hardwareMap.get(CRServo.class, "transfer2");
        injector2 = hardwareMap.get(CRServo.class, "transfer3");
        intake = hardwareMap.get(DcMotor.class, "intake");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        initializeOdometry(odo);

        // Toggle states

        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;
        boolean isTransferTurning = false;
        boolean isTransferTurningReverse = false;
        boolean areInjectorsTurning = false;
        boolean areInjectorsTurningReverse = false;
        boolean shooterForward = false;
        boolean xLast = false;
        boolean yLast = false;
        boolean reverseDrive = false;
        boolean lastRightBumper1 = false;

        initializeOdometry(odo);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.right_bumper && lastRightBumper1) {
                reverseDrive = !reverseDrive;
            }
            lastRightBumper1 = gamepad1.right_bumper;
            double reverse = reverseDrive ? -1.0 : 1.0;
            odo.update();
            // Start of basic driving
            double y = -gamepad1.left_stick_y * reverse; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * reverse;// * 1.1 // Counteract imperfect strafing
            double ftcTurn = -gamepad1.right_stick_x * reverse;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(ftcTurn), 1);
//            double frontLeftPower = (y + x + ftcTurn) / denominator;
//            double backLeftPower = (y - x + ftcTurn) / denominator;
//            double frontRightPower = (y - x - ftcTurn) / denominator;
//            double backRightPower = (y + x - ftcTurn) / denominator;
//
//            frontLeft.setPower(frontLeftPower);
//            backLeft.setPower(backLeftPower);
//            frontRight.setPower(frontRightPower);
//            backRight.setPower(backRightPower);

            double myHeading = odo.getHeading(AngleUnit.DEGREES);
            double yRobot = x*Math.cos(Math.toRadians(myHeading)) + y*Math.sin(Math.toRadians(myHeading));
            double xRobot = -x*Math.sin(Math.toRadians(myHeading)) + y*Math.cos(Math.toRadians(myHeading));

            moveRobot(xRobot, yRobot, ftcTurn);
            //end of basic driving

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
                if (shooterForward) {
                    turnShooter(0.75);               // forward
                } else {
                    turnShooter(0.0);               // stop
                }
            }
            xLast = gamepad1.x;

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
                areInjectorsTurning = !areInjectorsTurning;
                sleep(200);
                turnInjectors(areInjectorsTurning);
            }

            if (gamepad2.right_bumper) {
                areInjectorsTurningReverse = !areInjectorsTurningReverse;
                sleep(200);
                turnInjectorsReverse(areInjectorsTurningReverse);
            }

            // reset heading when robot is facing red goal side
            if (gamepad1.left_bumper) {
                odo.update();
                Pose2D newPose = new Pose2D(INCH, odo.getPosX(INCH), odo.getPosY(INCH), AngleUnit.DEGREES, 90);
                odo.setPosition(newPose);
                odo.update();
            }

        }
    }


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
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

    public void turnInjectors(boolean on) {
        if (on) {
            injector1.setPower(1);
            injector2.setPower(1);
        } else {
            injector1.setPower(0);
            injector2.setPower(0);
        }
    }

    public void turnInjectorsReverse(boolean on) {
        if (on) {
            injector1.setPower(-1);
            injector2.setPower(-1);
        } else {
            injector1.setPower(0);
            injector2.setPower(0);
        }
    }


    // ---------- SHOOTER ----------
    public void turnShooter (double power) {
        topShooter.setPower(-power);
        bottomShooter.setPower(power);
    }

    private void initializeOdometry(GoBildaPinpointDriver odometer) {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odometer.setOffsets(-2.1825, -7.125, INCH); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per unit of your odometry pod.
         */
        odometer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odometer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odometer.resetPosAndIMU();
        // allow 250 ms for calibration to complete
        sleep(300);

        Pose2D pose = new Pose2D(INCH, 0, 17, AngleUnit.DEGREES, 90); //usually 315 (angles are in odometry, ie. x axis = 0 degrees, so add 90 degrees)
        odo.setPosition(pose);
        odo.update();

        odometer.update();

        telemetry.addData("Init Odometry Function Status:", "Initialized");
        telemetry.addData("Odo Device Version Number:", odometer.getDeviceVersion());
        telemetry.addData("Odo X offset after initialization", odometer.getXOffset(INCH));
        telemetry.addData("Odo Y offset after initialization", odometer.getYOffset(INCH));
        telemetry.addData("Odo Heading after initialization", odometer.getYawScalar());
        telemetry.update();
    }

}
