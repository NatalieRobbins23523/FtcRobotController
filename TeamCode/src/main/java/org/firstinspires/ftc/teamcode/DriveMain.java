package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp
public class DriveMain extends LinearOpMode {


    //declares motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor elevator;
    private Servo elevatorPincher;
    private DcMotor arm;
    private CRServo intakeSpinner;
    //private CRServo armTelescope;




    public void runOpMode() {
        //defines motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        elevatorPincher = hardwareMap.get(Servo.class, "elevatorPincher");
        intakeSpinner = hardwareMap.get(CRServo.class, "intakeSpinner");
        intakeSpinner.setDirection(DcMotorSimple.Direction.REVERSE);


        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armTelescope = hardwareMap.get(CRServo.class, "armTelescope");


        elevator = hardwareMap.get(DcMotor.class, "elevator");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        // elevator.setDirection(DcMotor.Direction.REVERSE);
        elevator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        elevator.setDirection(DcMotor.Direction.REVERSE);
        int numClicks = 1;
        elevatorPincher.setPosition(.3);
        waitForStart();




        double tgtPowerY = 0.0;
        double tgtPowerTurn = 0.0;
        double tgtPowerSide = 0.0;
        boolean isReversed = false;
        boolean isIntakeTurning = false;
        boolean isIntakeTurningReverse = false;


        while (opModeIsActive()){
            if(gamepad1.right_bumper){
                isReversed = !isReversed;
                sleep(300); //makes sure reverse isn't toggled multiple times
            }
            //driving
            tgtPowerY = gamepad1.left_stick_y;


            // If the power on the left joystick is less than 0.5, halve the power
            if (Math.abs(tgtPowerY) < 0.5) {
                tgtPowerY /= 2;
            }




            driveStraight(tgtPowerY, isReversed);




            //turning
            tgtPowerTurn = gamepad1.right_stick_x;
            turn(tgtPowerTurn);


            //sideways
            tgtPowerSide = gamepad1.left_stick_x;


            if(Math.abs(tgtPowerSide) < 0.5) {
                tgtPowerSide /= 2;
                // tgtPowerSide = tgtPowerSide/2;
            }


            driveSideways(tgtPowerSide, isReversed);


            if(gamepad1.x){
                releaseElevator();
                numClicks++;
                sleep(200);
            }


            if (gamepad1.y) {
                raiseElevator();
            }


            if (gamepad1.a) {
                setElevatorGrabbingPosition();
            }


            if (gamepad1.b) {
                numClicks++;
                sleep(300);
                changePincher(numClicks);
            }


            if (gamepad1.left_bumper) {
                setElevatorZero();
                sleep(200);
            }
            if(gamepad2.b){
                isIntakeTurning = !isIntakeTurning;
                sleep(200);
                changeIntake(isIntakeTurning);
            }
            if(gamepad2.dpad_left){
                isIntakeTurningReverse = !isIntakeTurningReverse;
                sleep(200);
                changeIntake(isIntakeTurningReverse);
            }


            if(gamepad2.a){
                armDown();
            }
            if(gamepad2.x){
                armOut();
            }
            if(gamepad2.y){
                armUp();
            }
            if(gamepad2.left_bumper){
                isIntakeTurningReverse=!isIntakeTurningReverse;
                sleep(200);
                reverseIntake(isIntakeTurningReverse);
            }
            //armTelescope.setPower(gamepad2.left_stick_y);


            //lets us manually control elevator position
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
            } else if (gamepad2.dpad_right){
                elevator.setPower(0);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


        }


    }


    public void driveStraight(double pwrX, boolean isReversed){
        if(isReversed){
            frontLeft.setPower(-pwrX);
            backLeft.setPower(-pwrX);
            frontRight.setPower(-pwrX);
            backRight.setPower(-pwrX);
        }
        else{
            frontLeft.setPower(pwrX);
            backLeft.setPower(pwrX);
            frontRight.setPower(pwrX);
            backRight.setPower(pwrX);
        }


    }


    public void turn(double direction){
        frontLeft.setPower(-direction);
        backLeft.setPower(-direction);
        frontRight.setPower(direction);
        backRight.setPower(direction);
    }


    public void driveSideways(double speed, boolean isReversed){
        if(isReversed){
            frontLeft.setPower(speed);
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
            backRight.setPower(speed);
        }
        else{
            frontLeft.setPower(-speed);
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backRight.setPower(-speed);
        }
    }


    public void setElevatorGrabbingPosition(){ //sets elevator to grab from wall
        if (elevator.getCurrentPosition() != 450) { // Assuming 200 is the target
            elevator.setPower(1);
            elevator.setTargetPosition(450);
            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void raiseElevator(){ //sets elevator to place on bar
        elevator.setPower(1);
        elevator.setTargetPosition(3000);
    }
    public void releaseElevator(){ //lowers elevator to put block on bar
        elevator.setPower(1);
        elevator.setTargetPosition(400);
        sleep(400);
        elevatorPincher.setPosition(0.8);
        sleep(200);
        setElevatorGrabbingPosition();
    }
    public void setElevatorZero(){
        elevator.setPower(1);
        elevator.setTargetPosition(0);
    }
    public void changePincher(int numClicks){
        if(numClicks%2 ==0){
            elevatorPincher.setPosition(.8);
        }
        else{
            elevatorPincher.setPosition(.3);
        }
    }


    public void changeIntake(boolean isIntakeTurning){
        if(isIntakeTurning){
            intakeSpinner.setPower(1);
        }
        else{
            intakeSpinner.setPower(0);
        }
    }


    public void reverseIntake(boolean isIntakeTurning){
        if(isIntakeTurning){
            intakeSpinner.setPower(-1);
        }
        else{
            intakeSpinner.setPower(0);
        }
    }


    public void armUp(){
        arm.setPower(0.5);
        arm.setTargetPosition(2300);
    }


    public void armDown(){
        arm.setPower(0.5);
        arm.setTargetPosition(0);
    }


    public void armOut(){
        arm.setPower(0.5);
        arm.setTargetPosition(500);
    }


    public void armInBucket(){
        arm.setPower(0.5);
        arm.setTargetPosition(3400);
    }


}
