package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class DudAuto extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private DcMotor motorIntake;

    private DcMotor motorTopShoot;

    private DcMotor motorBottomShoot;


    private DcMotor motorTransfer;
    private CRServo motorTransfer2;


    public void runOpMode(){
        //defines motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);

        motorTopShoot = hardwareMap.get(DcMotor.class, "topShooter");

        motorBottomShoot = hardwareMap.get(DcMotor.class, "bottomShooter");


        motorIntake = hardwareMap.get(DcMotor.class, "intake");

        motorTransfer = hardwareMap.get(DcMotor.class, "transfer");

        motorTransfer2 = hardwareMap.get(CRServo.class, "transfer2");



        //MotorLeftShoot = hardwareMap.get(DcMotor.class, "MotorLeftShoot");
        //MotorRightShoot = hardwareMap.get(DcMotor.class, "MotorRightShoot");

        int loopCount = 0;
        int MAX_LOOP_COUNT = 1;
        int loopCount2 = 0;
        waitForStart();
        while(opModeIsActive()){


//            Shooter();
//            sleep(3000);
//            stopShooter();

            if (loopCount < MAX_LOOP_COUNT) {
//                driveStraight(0.5, false);
//                sleep(2000);
//                driveStraight(0, false);
//                sleep(10);
//                turn(-1);
//                sleep(275);
//                turn(0.0);
//                driveStraight(0.5, false);
                loopCount = loopCount + 1;
            }

            turnShooter(true, 0.9);
            turnIntake(true);
            sleep(1000);
            turnTransfer(true);
            turnTransfer2(true);
            sleep(800);
            turnTransfer(true);
            turnTransfer2(false);
            sleep(800);
            turnTransfer(true);
            turnTransfer2(true);
            sleep(800);
            turnTransfer(true);
            turnTransfer2(false);
            sleep(800);
            turnTransfer(true);
            turnTransfer2(true);
            sleep(800);
            turnTransfer2(false);
            sleep(1000);
            //turnIntake(true);
            //sleep(1000);
            //turnTransfer2(true);
            //sleep(150);
            //turnTransfer2(false);
            //sleep(500);
            //turnTransfer2(true);
            //sleep(150);
            //turnTransfer2(false);
            //sleep(500);
            //turnTransfer2(true);
            //sleep(150);
            turnTransfer2(false);
            turnTransfer(false);
            turnShooter(false, 0);
            sleep(500);

//            if (loopCount2 < MAX_LOOP_COUNT) {
//                driveStraight(-0.5, false);
//                sleep(750);
//                driveStraight(0.0, false);
//                //turn(0.5);
//                //sleep(150);
//                loopCount2 = loopCount2 + 1;
//            }




//            turn(0.5);
//            sleep(1000);
//            stopMotors();
//
//
//            driveStraight(0.5, true);
//            sleep(2000);
//            stopMotors();
//
//
//            driveStraight(0.5, true);
//            Intake();
//            sleep(1000);
//            stopIntake();
//            stopMotors();
//
//
//            driveStraight(0.5,false);
//            sleep(3000);
//            stopMotors();
//
//
//            turn(-0.5);
//            sleep(1000);
//            stopMotors();
//
//
//            driveStraight(0.5, true);
//            sleep(500);
//            stopMotors();


        }
    }

    public void driveStraight(double power, boolean reversed) {
        double p = reversed ? -power : power;
        frontLeft.setPower(p);
        backLeft.setPower(p);
        frontRight.setPower(p);
        backRight.setPower(p);
    }
//    public void driveStraight(double pwrX, boolean isReversed){
//        if(isReversed){
//            frontLeft.setPower(-pwrX);
//            backLeft.setPower(-pwrX);
//            frontRight.setPower(-pwrX);
//            backRight.setPower(-pwrX);
//        }
//        else{
//            frontLeft.setPower(pwrX);
//            backLeft.setPower(pwrX);
//            frontRight.setPower(pwrX);
//            backRight.setPower(pwrX);
//        }


    //}


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


    public void driveDiagonally(double tgtX, double tgtY) {
        frontLeft.setPower(tgtY - tgtX);
        frontRight.setPower(tgtY + tgtX);
        backLeft.setPower(tgtY + tgtX);
        backRight.setPower(tgtY - tgtX);


    }


    public void stopMotors(){
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }




    public void turnIntake(boolean on) {
        if (on) {
            motorIntake.setPower(1);
        } else {
            motorIntake.setPower(0);
        }
    }

    public void turnShooter(boolean on, double power) {
        if (on) {
            motorTopShoot.setPower(-power);
            motorBottomShoot.setPower(power);

        } else {
            motorTopShoot.setPower(0);
            motorBottomShoot.setPower(0);

        }
    }

    public void turnTransfer(boolean on) {
        if (on) {
            motorTransfer.setPower(1);
        } else {
            motorTransfer.setPower(0);
        }
    }

    public void turnTransfer2(boolean on) {
        if (on) {
            motorTransfer2.setPower(-1);
        }
        else {
            motorTransfer.setPower(0);
        }
    }
}



