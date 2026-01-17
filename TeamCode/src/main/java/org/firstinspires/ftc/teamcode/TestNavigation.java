package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import java.util.Locale;


@TeleOp
public class TestNavigation extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;
    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor motorIntake;
    private DcMotor motorShoot;
    private DcMotor motorTransfer;
    private ElapsedTime runtime = new ElapsedTime();

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.1;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 5 inch X error.
    final double STRAFE_GAIN =  0.125;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 50% power at a 4 inch Y error.
    final double TURN_GAIN   =  0.025;   //  Turn Control "Gain".  e.g. Ramp up to 50% power at a 20 degree error. (0.5 / 20)
    final double KD =  0.2;   //  PID Derivative Gain
    final double KI =  0.03;   //  PID Integral Gain
    final double RANGE_TOLERANCE = .5; // How close robot must be to target point to stop
    final double HEADING_TOLERANCE = 10; // How many degrees off of target heading robot can be to stop
    final double VELOCITY_TOLERANCE = 1; // inches/second
    final double MAX_AUTO_SPEED = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.6;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.4;   //  Clip the turn speed to this max value (adjust for your robot)

    final int RED_GOAL_ID = 24;
    final int BLUE_GOAL_ID = 20;
    // Define waypoints to guide navigation in AUTO
    final Pose2D ballsInfrontFar = new Pose2D(DistanceUnit.INCH, -12, 36, AngleUnit.DEGREES, 0);
    final Pose2D ballsInfrontMiddle = new Pose2D(DistanceUnit.INCH, 12, 36, AngleUnit.DEGREES, 0);
    final Pose2D ballsInfrontClose = new Pose2D(DistanceUnit.INCH, 36, 36, AngleUnit.DEGREES, 0);
    final Pose2D ballsAfterFar = new Pose2D(DistanceUnit.INCH, -12, 56, AngleUnit.DEGREES, 0);
    final Pose2D ballsAfterMiddle = new Pose2D(DistanceUnit.INCH, 12, 56, AngleUnit.DEGREES, 0);
    final Pose2D ballsAfterClose = new Pose2D(DistanceUnit.INCH, 36, 56, AngleUnit.DEGREES, 0);
    final Pose2D shooterPoint = new Pose2D(DistanceUnit.INCH, -0, 0, AngleUnit.DEGREES, 45);
    final Pose2D redAprilTagPosition = new Pose2D(DistanceUnit.INCH, -58.37, 55.64, AngleUnit.DEGREES, 46.4);
    final Pose2D blueAprilTagPosition = new Pose2D(DistanceUnit.INCH, -58.37, -55.64, AngleUnit.DEGREES, 133.6);
    final Pose2D initialBackupPoint = new Pose2D(DistanceUnit.INCH, -28.37, -25.64, AngleUnit.DEGREES,46.4);
    final Pose2D point_01 = new Pose2D(DistanceUnit.INCH, 0, 24, AngleUnit.DEGREES,0);
    final Pose2D point_m10 = new Pose2D(DistanceUnit.INCH, -24, 0, AngleUnit.DEGREES,90);
    final Pose2D point_0m1 = new Pose2D(DistanceUnit.INCH, 0, -24, AngleUnit.DEGREES,180);
    final Pose2D point_10 = new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES,270);
    final Pose2D point_00 = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES,270);

    @Override
    public void runOpMode() {

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        // frontRight.setDirection(DcMotor.Direction.REVERSE);
        // backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize odometry system
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        initializeOdometry(odo);

        // START DRIVING !!!
        waitForStart();

        // Print the current Position and Velocity of the robot.
        odo.update();
        Pose2D robotPos = odo.getPosition();
        String data = String.format(Locale.US, "{%.1f, %.1f, %.1f}", robotPos.getX(DistanceUnit.INCH), robotPos.getY(DistanceUnit.INCH), robotPos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Odometry position after INIT (XYH):", data);

        driveToPoint(point_01);
        driveToPoint(point_m10);
        driveToPoint(point_0m1);
        driveToPoint(point_10);
        driveToPoint((point_00));

        while (opModeIsActive()) {}

    }

    public void driveToPoint(Pose2D targetPoint) {
        // targetPoint is in FTC Field Coordinates

        final double ODO_TO_FTC_ROTATION = -90; // Odometry heading is FTC heading + 90 degrees
        final int MAX_LOOP_COUNT = 500;
        final double MAX_DRIVE_TIME_MS = 5500;

        double iXErr = 0; // integral of X error
        double iYErr = 0; // integral of Y error
        double iHErr = 0; // integral of Heading error
        double lastXErr = 0;
        double lastYErr = 0;
        double lastHErr = 0;
        double drive = 0;         // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;          // Desired turning power/speed (-1 to +1)

        double tgtX = targetPoint.getX(DistanceUnit.INCH);
        double tgtY = targetPoint.getY(DistanceUnit.INCH);
        double tgtHeading = targetPoint.getHeading(AngleUnit.DEGREES);
        telemetry.addData("DTP: ", "Tgt XYH (FTC) %.1f, %.1f, %.1f", tgtX, tgtY, tgtHeading);
        odo.update();
        telemetry.addData("DTP: ", "Odo XYH (ODO): %.1f, %.1f, %.1f \n",
                            odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.DEGREES));

        int driveLoopCount = 0;
        double startTime = runtime.milliseconds();
        ElapsedTime loopTimer = new ElapsedTime();

        // loop until target point is reached
        while (opModeIsActive() & ((runtime.milliseconds() - startTime) < MAX_DRIVE_TIME_MS)) {

            driveLoopCount += 1;

            odo.update();

            // calculate current position and orientation error relative to target
            Pose2D myPos = odo.getPosition();
            double myVelX = odo.getVelX(DistanceUnit.INCH);
            double myVelY = odo.getVelY(DistanceUnit.INCH);
            double myVel = Math.sqrt(myVelX*myVelX + myVelY*myVelY);
            double myX = myPos.getX(DistanceUnit.INCH);
            double myY = myPos.getY(DistanceUnit.INCH);
            double myHeading = myPos.getHeading(AngleUnit.DEGREES);
            double myHeadingFTC = myHeading + ODO_TO_FTC_ROTATION;

            telemetry.addData("DTP: ", "My XYH (ODO): %.1f, %.1f, %.1f", myX, myY, myHeading);

            double xErr = tgtX - myX;
            double yErr = tgtY - myY;
            double rangeError = Math.sqrt(xErr*xErr + yErr*yErr);
            double headingError = NormalizeAngle(tgtHeading - myHeadingFTC);
            telemetry.addData("DTP: ", "Error XYHRng (FTC) %.1f, %.1f, %.1f, %.1f", xErr, yErr, headingError, rangeError);

            // check if robot is close enough
            if (rangeError <= RANGE_TOLERANCE & Math.abs(headingError) <= HEADING_TOLERANCE & myVel <= VELOCITY_TOLERANCE) {
                moveRobot(0, 0, 0);
                telemetry.addData("DTP: ", "Within Tolerance ");
                telemetry.update();
                break;
            }

            // calculate PID variables
            double dTime = loopTimer.seconds();
            loopTimer.reset();
            double mXErr = (xErr - lastXErr) / dTime;
            double mYErr = (yErr - lastYErr) / dTime;
            double mHErr = (headingError - lastHErr) / dTime;
            iXErr += xErr * dTime;
            iYErr += yErr * dTime;
            iHErr += headingError * dTime;
            double pidXErr = xErr + KD * mXErr + KI * iXErr;
            double pidYErr = yErr + KD * mYErr + KI * iYErr;
            double pidHErr = headingError + KD * mHErr + KI * iHErr;

            // calculate direction to drive in robot body coordinates (straight ahead is +ve X, left is +ve Y)
            double xErrRobot = pidXErr * Math.cos(Math.toRadians(myHeading)) + pidYErr * Math.sin(Math.toRadians(myHeading));
            double yErrRobot = -pidXErr * Math.sin(Math.toRadians(myHeading)) + pidYErr * Math.cos(Math.toRadians(myHeading));
            telemetry.addData("DTP: ", "xErrRobot %.1f, yErrRobot %.1f", xErrRobot, yErrRobot);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(xErrRobot * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yErrRobot * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn = Range.clip(pidHErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            telemetry.addData("DTP: ", "Drive %5.2f, Strafe %5.2f, Turn %5.2f \n", drive, strafe, turn);
//            if (driveLoopCount > MAX_LOOP_COUNT) {
//                moveRobot(0, 0, 0);
//                telemetry.update();
//                break;
//            }
            moveRobot(drive, strafe, turn);
            sleep(10);

            // update PID variables
            lastXErr = xErr;
            lastYErr = yErr;
            lastHErr = headingError;
        }
        telemetry.update();
        sleep(500);
    } // end of driveToPoint function

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
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

    /**
     * Initialize the AprilTag processor.
     */

    private void initializeOdometry(GoBildaPinpointDriver odometer) {
        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        odometer.setOffsets(-2.1825, -7.125, DistanceUnit.INCH); //these are tuned for 3110-0002-0001 Product Insight #1

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

        odometer.update();

        telemetry.addData("Init Odometry Function Status:", "Initialized");
        telemetry.addData("Odo Device Version Number:", odometer.getDeviceVersion());
        telemetry.addData("Odo X offset after initialization", odometer.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Y offset after initialization", odometer.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Heading after initialization", odometer.getYawScalar());
        telemetry.update();
        sleep(2000);
    }

    private double NormalizeAngle(double angle) {
        while (angle > 180) { angle -= 360; }
        while (angle < -180) { angle += 360; }
        return angle;
    }
}

