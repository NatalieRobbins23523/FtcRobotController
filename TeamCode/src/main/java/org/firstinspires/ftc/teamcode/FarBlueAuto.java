package org.firstinspires.ftc.teamcode;




import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class FarBlueAuto extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private CRServo servo1;
    private DcMotor MotorLeftShoot;
    private DcMotor MotorRightShoot;


    public void runOpMode(){
        //defines motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        MotorLeftShoot = hardwareMap.get(DcMotor.class, "MotorLeftShoot");
        MotorRightShoot = hardwareMap.get(DcMotor.class, "MotorRightShoot");


        waitForStart();
        while(opModeIsActive()){


            Shooter();
            sleep(6000);
            stopShooter();


            driveStraight(0.5, false);
            sleep(2000);
            stopMotors();


            driveSideways(0.5, false);
            sleep(4000);
            stopMotors();


            driveStraight(0.5, true);
            sleep(5000);
            stopMotors();


            driveSideways(0.5, true);
            sleep(4000);
            stopMotors();


            turn(0.5);
            sleep(500);
            stopMotors();


            Shooter();
            sleep(6000);
            stopShooter();
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


    public void Intake() {
        servo1.setPower(1);
    }


    public void stopIntake(){
        servo1.setPower(0);
    }


    public void Shooter() {
        MotorRightShoot.setPower(1.0);
        MotorLeftShoot.setPower(-1.0);
    }


    public void stopShooter(){
        MotorRightShoot.setPower(0);
        MotorLeftShoot.setPower(0);
    }
}
