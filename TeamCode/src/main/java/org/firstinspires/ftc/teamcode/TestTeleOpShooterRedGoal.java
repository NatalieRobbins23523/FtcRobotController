package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
public class TestTeleOpShooterRedGoal extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotorEx bottomShooter;
    private DcMotorEx topShooter;
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

        bottomShooter = hardwareMap.get(DcMotorEx.class, "bottomShooter");
        topShooter = hardwareMap.get(DcMotorEx.class, "topShooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        injector1 = hardwareMap.get(CRServo.class, "transfer2");
        injector2 = hardwareMap.get(CRServo.class, "transfer3");
        intake = hardwareMap.get(DcMotor.class, "intake");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        initializeOdometry(odo);

        // Toggle states
        boolean isReversed = false;
        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;
        boolean isTransferTurning = false;
        boolean isTransferTurningReverse = false;
        boolean areInjectorsTurning = false;
        boolean areInjectorsTurningReverse = false;
        boolean xLast = false;
        boolean yLast = false;

        boolean shooterForward = false;
        boolean shooterReverse = false;

        initializeOdometry(odo);

        waitForStart();

        while (opModeIsActive()) {

            // Toggle driving reverse
            if (gamepad1.right_bumper) {
                isReversed = !isReversed;
                sleep(250);
            }

            // Drive forward/backward
            double tgtPowerY = -gamepad1.left_stick_y; // forward y on gamepad returns negative value
            if (Math.abs(tgtPowerY) < 0.5) tgtPowerY /= 2;
            driveStraight(tgtPowerY, isReversed);

            // Turning
            turn(gamepad1.right_stick_x);

            // Strafing
            double tgtPowerSide = -gamepad1.left_stick_x; // gamepad returns positive value when pushed right but positive strafe is left
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

            //interpolateStuff(calcDistFromGoal());
            //Shoot(interpolateStuff(calcDistFromGoal()));

            if (gamepad1.x && !xLast) {             // runs ONLY on button press edge
                shooterForward = !shooterForward;   // toggle
                if (shooterForward) {
                    turnShooter(1);               // forward
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

        Pose2D pose = new Pose2D(INCH, -6.5, 36, AngleUnit.DEGREES, 0);
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

    private double NormalizeAngle(double angle) {
        while (angle > 180) { angle -= 360; }
        while (angle < -180) { angle += 360; }
        return angle;
    }

    public void Shoot (double targetTopVelocity, double targetBottomVelocity) {

        final double SHOOT_SPEED_TOLERANCE = 30;
        final double SHOT_TIME_LIMIT = 2000;
        double Kp = 0.8*1/2800, Ki = 0.0, Kd = 0.00006; // Example coefficients
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
                injector2.setPower(-1);
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
        injector1.setPower(0);
        injector2.setPower(0);
        topShooter.setPower(0);
        bottomShooter.setPower(0);
    }

    public double[] interpolateStuff(double distance) {
        double[] distanceArray = {46, 58, 70}; // X
        double[] topMotorSpeedArray = {1250, 1350, 1500}; // Y top
        double[] bottomMotorSpeedArray = {1300, 1350, 1500}; // Y bottom
        // TODO - test more distances

        for ( int i = 0; i < distanceArray.length; i++ ) {
            if (distanceArray[i] > distance){
                double distanceA = distanceArray[i - 1];
                double distanceB = distanceArray[i];

                double y1Top = topMotorSpeedArray[i - 1];
                double y2Top = topMotorSpeedArray[i];
                double topMotorSpeed = y1Top + (distance - distanceA) * (y2Top - y1Top) / (distanceB - distanceA);

                double y1Bottom = bottomMotorSpeedArray[i - 1];
                double y2Bottom = bottomMotorSpeedArray [i];
                double bottomMotorSpeed = y1Bottom + (distance - distanceA) * (y2Bottom - y1Bottom) / (distanceB - distanceA);

                return new double[] { topMotorSpeed, bottomMotorSpeed };
            }
        }
        return distanceArray; // Not sure why it made me put this here but I don't think it will mess anything up
    }

    public void calcShooterSpeed(double distance) {

    }

    public double calcDistFromGoal() {
        odo.update();
        Pose2D pos = odo.getPosition(); // measured from front middle of intake
        double myX = pos.getX(DistanceUnit.INCH);
        double myY = pos.getY(DistanceUnit.INCH);
        double myHeading = pos.getHeading(AngleUnit.DEGREES);

        final Pose2D goalPos = new Pose2D(DistanceUnit.INCH, -60, 60, AngleUnit.DEGREES, NormalizeAngle(-45)); // TODO - get actual goal coordinates
        double goalX = goalPos.getX(DistanceUnit.INCH);
        double goalY = goalPos.getY(DistanceUnit.INCH);
        double goalHeading = goalPos.getHeading((AngleUnit.DEGREES));

        double XErr = myX - goalX;
        double YErr = myY - goalY;

        double distance = Math.sqrt((XErr * XErr) + (YErr * YErr));
        telemetry.addData("Dist from goal", distance);
        telemetry.update();
        return distance;
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
            injector2.setPower(-1);
        } else {
            injector1.setPower(0);
            injector1.setPower(0);
        }
    }

    public void turnInjectorsReverse(boolean on) {
        if (on) {
            injector1.setPower(-1);
            injector2.setPower(1);
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

}

