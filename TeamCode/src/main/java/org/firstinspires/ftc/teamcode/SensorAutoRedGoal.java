package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SensorAutoRedGoal extends LinearOpMode {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;
    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor intake;
    private DcMotorEx topShooter;
    private DcMotorEx bottomShooter;
    private DcMotor transfer;
    private CRServo injector1;
    private CRServo injector2;

    private ElapsedTime runtime = new ElapsedTime();

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double DEFAULT_SPEED_GAIN  =  0.12  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double DEFAULT_STRAFE_GAIN =  0.12 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double DEFAULT_TURN_GAIN   =  0.05  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double DEFAULT_DRIVE_TIMEOUT = 3500; // milliseconds
    final double KD =  0.15;   //  PID Derivative Gain
    final double KI =  0.05;   //  PID Integral Gain
    final double DEFAULT_RANGE_TOLERANCE = 2; // How close robot must be to target point to stop
    final double DEFAULT_HEADING_TOLERANCE = 15; // How many degrees off of target heading robot can be to stop
    final double DEFAULT_VELOCITY_TOLERANCE = 100; // inches/second
    final double MAX_AUTO_SPEED = 1.0;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 1.0;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 1.0;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    final int RED_GOAL_ID = 24;
    final int BLUE_GOAL_ID = 20;
    // Define waypoints to guide navigation in AUTO
    final Pose2D ballsInfrontFar = new Pose2D(DistanceUnit.INCH, -12, 32, AngleUnit.DEGREES, 0);
    final Pose2D ballsInfrontMiddle = new Pose2D(DistanceUnit.INCH, 12, 34, AngleUnit.DEGREES, 0);
    final Pose2D ballsInfrontClose = new Pose2D(DistanceUnit.INCH, 36, 34, AngleUnit.DEGREES, 0);
    final Pose2D ballsHalfFar = new Pose2D(DistanceUnit.INCH, -12, 49.5, AngleUnit.DEGREES, 0);
    final Pose2D ballsHalfMiddle = new Pose2D(DistanceUnit.INCH, 12, 49.5, AngleUnit.DEGREES, 0);
    final Pose2D ballsHalfClose = new Pose2D(DistanceUnit.INCH, 36, 49.5, AngleUnit.DEGREES, 0);
    final Pose2D ballsAfterFar = new Pose2D(DistanceUnit.INCH, -12, 57.5, AngleUnit.DEGREES, 0);
    final Pose2D ballsAfterMiddle = new Pose2D(DistanceUnit.INCH, 12, 57.5, AngleUnit.DEGREES, 0);
    final Pose2D ballsAfterClose = new Pose2D(DistanceUnit.INCH, 33, 58.5, AngleUnit.DEGREES, 0);
    //    final Pose2D redAprilTagPosition = new Pose2D(DistanceUnit.INCH, -58.37, 55.64, AngleUnit.DEGREES, 46.4+90); // +90 because this is used to set odometry pose
//    final Pose2D blueAprilTagPosition = new Pose2D(DistanceUnit.INCH, -58.37, -55.64, AngleUnit.DEGREES, 133.6+90);
    final Pose2D shooterPoint = new Pose2D(DistanceUnit.INCH, -7.5, 14, AngleUnit.DEGREES, NormalizeAngle(234));
    //is in field coordinates
    final Pose2D gatePoint1 = new Pose2D(DistanceUnit.INCH, -5, -50, AngleUnit.DEGREES,90);
    final Pose2D gatePoint2 = new Pose2D(DistanceUnit.INCH, -5, -56.5, AngleUnit.DEGREES,90);
    final Pose2D finalParkingPoint = new Pose2D(DistanceUnit.INCH, 0, 24, AngleUnit.DEGREES,0); //usually 225

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            4.5, -8, 15.375, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0);

    @Override
    public void runOpMode() {

        // INITIALIZE HARDWARE
        // Note that the strings used here must correspond to the names assigned during
        // the robot configuration step on the DS or RC devices.

        // Initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize ball handling path
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        topShooter.setDirection(DcMotor.Direction.REVERSE);
        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        injector1 = hardwareMap.get(CRServo.class, "transfer2");
        injector2 = hardwareMap.get(CRServo.class, "transfer3");


        // Initialize odometry system
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        initializeOdometry(odo);

        // Initialize the Apriltag Detection process
        //initAprilTag();

        // START DRIVING !!!
        waitForStart();
        resetRuntime();

        // Toggle states
        boolean isIntakeTurning = false;
        boolean isTransferTurning = false;
        boolean isShooterTurning = false;

        /*
        This code prints the loop frequency of the REV Control Hub. This frequency is effected
        by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
        of time each cycle takes and finds the frequency (number of updates per second) from
        that cycle time.
         */
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

//        odo.setPosition(redAprilTagPosition);
        odo.update();
        telemetry.addData("Auto", "odo X,Y,H: %5.2f, %5.2f, %5.2f ",
                odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.DEGREES));
        turnBottomShooter(true, 0.6);
        sleep(50);
        turnTopShooter(true, -0.6);

        driveToPoint(shooterPoint, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                        DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        Shoot(1600, 1800);
        //syncOdoToFTC(4000);

        // Print the current Position and Velocity of the robot.
//        odo.update();
//        Pose2D pos = odo.getPosition();
//        telemetry.addData("Odometry position after initial backing up movement",  "");
//        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
//        telemetry.update();

        /*
        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
        READY: the device is working as normal
        CALIBRATING: the device is calibrating and outputs are put on hold
        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
        FAULT_BAD_READ - The firmware detected a bad I²C read, if a bad read is detected, the device status is updated and the previous position is reported
        */
//        telemetry.addData("Status", odo.getDeviceStatus());
//
//        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
//
//        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate

//        turnTopShooter(true, 0.8);
//        turnBottomShooter(true, 1);

        driveToPoint(ballsInfrontFar, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        telemetry.addData("RETURNED FROM DTP BALLSINFRONTFAR", "");
        telemetry.update();
        //driveToPoint((ballsHalfFar));
        driveToPoint(ballsAfterFar, 0.035, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, 5, DEFAULT_VELOCITY_TOLERANCE);
        turnTransfer(false);
        turnIntake(false);

        driveToPoint(shooterPoint, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        Shoot(1600, 1800);
        //turnTransfer2(false);
        //syncOdoToFTC(1.0);

        driveToPoint(ballsInfrontMiddle, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        //driveToPoint(ballsHalfMiddle);
        driveToPoint(ballsAfterMiddle, 0.035, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, 5, DEFAULT_VELOCITY_TOLERANCE);
        turnTransfer(false);
        turnIntake(false);
        driveToPoint(shooterPoint, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        Shoot(1600, 1800);
        //turnTransfer2(true);
        //sleep(1500);
        //turnTransfer2(false);
        //syncOdoToFTC(1.0);

        driveToPoint(ballsInfrontClose, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        //driveToPoint(ballsHalfClose);
        driveToPoint(ballsAfterClose, 0.035, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, 5, DEFAULT_VELOCITY_TOLERANCE);
        turnTransfer(false);
        turnIntake(false);
        driveToPoint(shooterPoint, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        Shoot(1600, 1800);
        //turnTransfer2(false);
        turnIntake(false);
        turnTopShooter(false, 0);
        turnBottomShooter(false, 0);
        turnTransfer(false);
        driveToPoint(finalParkingPoint, DEFAULT_SPEED_GAIN, DEFAULT_STRAFE_GAIN, DEFAULT_TURN_GAIN, DEFAULT_DRIVE_TIMEOUT,
                DEFAULT_HEADING_TOLERANCE, DEFAULT_RANGE_TOLERANCE, DEFAULT_VELOCITY_TOLERANCE);
        injector1.setPower(0);
        injector2.setPower(0);

        while (opModeIsActive()) {}

    }

    // intake
    public void turnIntake(boolean on) {
        if (on) { intake.setPower(-1); }
        else { intake.setPower(0); }
    }

    // shooter
    public void turnTopShooter(boolean on, double power) {
        if (on) { topShooter.setPower(-power); }
        else { topShooter.setPower(0); }
    }

    public void turnBottomShooter(boolean on, double power) {
        if (on) { bottomShooter.setPower(power); }
        else { bottomShooter.setPower(0); }
    }

    public void setMotorShootPower(double power) {
        topShooter.setPower(power);
    }


    // transfer
    public void turnTransfer(boolean on) {
        if (on) { transfer.setPower(-1); }
        else { transfer.setPower(0); }
    }

    public void turnInjector1 (boolean on) {
        if (on) { injector1.setPower(1); }
        else { injector1.setPower(0); }
    }

    public void turnInjector2 (boolean on) {
        if (on) { injector2.setPower(-1); }
        else { injector2.setPower(0); }
    }

    // Odometry Functions
    public void driveToPoint(Pose2D targetPoint, double SPEED_GAIN, double STRAFE_GAIN, double TURN_GAIN, double MAX_LOOP_TIME,
                             double HEADING_TOLERANCE, double RANGE_TOLERANCE, double VELOCITY_TOLERANCE) {
        // targetPoint is in FTC Field Coordinates

        final double ODO_TO_FTC_ROTATION = -90; // Odometry heading is FTC heading + 90 degrees
        final int MAX_LOOP_COUNT = 500;

        double iXErr = 0; // integral of X error
        double iYErr = 0; // integral of Y error
        double iHErr = 0; // integral of Heading error
        double lastXErr = 0;
        double lastYErr = 0;
        double lastHErr = 0;
        double  drive           = 0;          // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;          // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;          // Desired turning power/speed (-1 to +1)

        double tgtX = targetPoint.getX(DistanceUnit.INCH);
        double tgtY = targetPoint.getY(DistanceUnit.INCH);
        double tgtHeading = targetPoint.getHeading(AngleUnit.DEGREES);
        telemetry.addData("DTP: ", "Tgt XYH (FTC) %.1f, %.1f, %.1f", tgtX, tgtY, tgtHeading);
        odo.update();
        telemetry.addData("DTP: ", "Odo XYH (ODO): %.1f, %.1f, %.1f",
                odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.DEGREES));

        double startTime = runtime.milliseconds();
        ElapsedTime loopTimer = new ElapsedTime();

        // loop until target point is reached, but break loop if it takes too long
        while (opModeIsActive() && ((runtime.milliseconds() - startTime) < MAX_LOOP_TIME)) {
            // Calculate current position - assume odometry X and Y have been set to FTC coordinates
            // Note that FTC angle is 90 degrees 'behind' odometry angle (odometry angle = FTC angle + 90)
            odo.update();
            Pose2D myPos = odo.getPosition();
            double myX = myPos.getX(DistanceUnit.INCH);
            double myY = myPos.getY(DistanceUnit.INCH);
            double myVelX = odo.getVelX(DistanceUnit.INCH);
            double myVelY = odo.getVelY(DistanceUnit.INCH);
            double myVel = Math.sqrt(myVelX*myVelX + myVelY*myVelY);
            double myHeading = myPos.getHeading(AngleUnit.DEGREES);
            double myHeadingFTC = myHeading + ODO_TO_FTC_ROTATION;
            telemetry.addData("DTP: ", "My XYH (ODO): %.1f, %.1f, %.1f", myX, myY, myHeading);

            // calculate orientation error relative to target
            double xErr = tgtX - myX;
            double yErr = tgtY - myY;
            double rangeError = Math.sqrt(xErr*xErr + yErr*yErr);
            double headingError = tgtHeading - myHeadingFTC;
            while (headingError > 180) { headingError -= 360; }
            while (headingError < -180) { headingError += 360; }
            telemetry.addData("DTP: ", "Error XYH Range (FTC) %.1f, %.1f, %.1f, %.1f", xErr, yErr, headingError, rangeError);

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

            // Calculate direction to drive in robot body coordinates (straight ahead is +ve X, left is +ve Y)
            // Robot coordinate frame is rotated by 'myHeading' degrees relative to FTC coordinate frame
            double xErrRobot = pidXErr * Math.cos(Math.toRadians(myHeading)) + pidYErr * Math.sin(Math.toRadians(myHeading));
            double yErrRobot = -pidXErr * Math.sin(Math.toRadians(myHeading)) + pidYErr * Math.cos(Math.toRadians(myHeading));
            telemetry.addData("DTP: ", "xErrRobot %.1f, yErrRobot %.1f", xErrRobot, yErrRobot);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(xErrRobot * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(yErrRobot * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            turn = Range.clip(pidHErr * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            telemetry.addData("DTP: ", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

            moveRobot(drive, strafe, turn);
            sleep(10);

            lastXErr = xErr;
            lastYErr = yErr;
            lastHErr = headingError;
            telemetry.update();
        }
        // stop robot if it doesn't reach target point in time (or else it keeps moving)
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    } // end of driveToPoint function

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

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
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

        Pose2D pose = new Pose2D(DistanceUnit.INCH, -50.5, 41.75, AngleUnit.DEGREES, NormalizeAngle(-45));
        odo.setPosition(pose);
        odo.update();

        odometer.update();

        telemetry.addData("Init Odometry Function Status:", "Initialized");
        telemetry.addData("Odo Device Version Number:", odometer.getDeviceVersion());
        telemetry.addData("Odo X offset after initialization", odometer.getXOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Y offset after initialization", odometer.getYOffset(DistanceUnit.INCH));
        telemetry.addData("Odo Heading after initialization", odometer.getYawScalar());
        telemetry.update();
    }


    private void syncOdoToFTC(double maxWaitTime) {
        runtime.reset();
        //Initialize odometry position based on april tag
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        desiredTag  = null;
        while (!targetFound && opModeIsActive() && runtime.milliseconds() < maxWaitTime) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((detection.id == BLUE_GOAL_ID) || (detection.id == RED_GOAL_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            telemetry.addData("X coord based on AprilTag",  "%5.1f inches", desiredTag.ftcPose.x);
            telemetry.addData("Y coord based on AprilTag",  "%5.1f inches", desiredTag.ftcPose.y);
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                    desiredTag.robotPose.getPosition().x,
                    desiredTag.robotPose.getPosition().y,
                    desiredTag.robotPose.getPosition().z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                    desiredTag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    desiredTag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    desiredTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

            // Set odometry x,y to match FTC x,y from April Tag.
            // Set odometry heading to FTC heading + 90 degrees since odometry measures heading starting from x axis.
            Pose2D pose = new Pose2D(DistanceUnit.INCH, desiredTag.robotPose.getPosition().x, desiredTag.robotPose.getPosition().y,
                    AngleUnit.DEGREES, desiredTag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) + 90);
            odo.setPosition(pose);
            odo.update();
            telemetry.addData("Auto", "odo X,Y,H: %5.2f, %5.2f, %5.2f ",
                    odo.getPosX(DistanceUnit.INCH), odo.getPosY(DistanceUnit.INCH), odo.getHeading(AngleUnit.DEGREES));

        } else {
            telemetry.addData("\n>","Could not find valid target\n");
        }
        telemetry.update();

        // Sleep for debugging to see if april tag sync worked
        sleep(5000);

    }

    private double NormalizeAngle(double angle) {
        while (angle > 180) { angle -= 360; }
        while (angle < -180) { angle += 360; }
        return angle;
    }

    public void Shoot (double targetTopVelocity, double targetBottomVelocity) {

        final double SHOOT_SPEED_TOLERANCE = 50;
        final double SHOT_TIME_LIMIT = 2250;
        double Kp = 0.8*1/2800, Ki = 0.0, Kd = 0.00002; // Example coefficients
        double topIntegralSum = 0;
        double bottomIntegralSum = 0;
        double lastTopError = 0;
        double lastBottomError = 0;
        double currentTopPower = 0.6;
        double currentBottomPower = 0.6;
        boolean shotsStarted = false;
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime shotTimer = new ElapsedTime();

        while (shotTimer.milliseconds() < SHOT_TIME_LIMIT) {

            double currentTopVelocity = topShooter.getVelocity();
            double currentBottomVelocity = bottomShooter.getVelocity();

            // Calculate error
            double topError = targetTopVelocity - currentTopVelocity;
            double bottomError = targetBottomVelocity - currentBottomVelocity;

            if (topError <= SHOOT_SPEED_TOLERANCE && bottomError <= SHOOT_SPEED_TOLERANCE && !shotsStarted) {
                injector1.setPower(1);
                injector2.setPower(1);
                transfer.setPower(-1);
                intake.setPower(-1);
                shotsStarted = true;
                shotTimer.reset();
            }

            // Integral sum (with simple anti-windup)
            double dt = loopTimer.seconds();
            loopTimer.reset();
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
        }
        injector1.setPower(-0.2);
        injector2.setPower(-0.2);
        //transfer.setPower(0);
        //intake.setPower(0);
        //topShooter.setPower(0);
        //bottomShooter.setPower(0);
    }
}


