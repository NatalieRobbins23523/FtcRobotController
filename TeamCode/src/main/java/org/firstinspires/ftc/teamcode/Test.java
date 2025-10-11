package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Test extends LinearOpMode {
    private DcMotor motor1; //a semi-colon means done w line//declaring a motor
    private Servo servo1;
    private CRServo servo2;//continuous
    public void runOpMode(){//put parenthesis after every function (abv)
        servo1 = hardwareMap.get(Servo.class, "servo1");//always put device name in configuration
        motor1=hardwareMap.get(DcMotor.class,"motor1");//defining a motor
        motor1.setDirection(DcMotor.Direction.REVERSE);//motor reversed
        servo2 = hardwareMap.get(CRServo.class, "servo2");




        waitForStart(); //makes it wait until start button is pressed
        while(opModeIsActive() ){
            if(gamepad1.a){//whenever you press a it sets the speed to 1
                setServo2(1);
            }
            if(gamepad1.b){
                setServo2(0);
            }
            setMotor1(.8);//between -1 and 1;


        }
    }


    //this is a function rawr
    public void setMotor1(double speed) {//whenever the function is called, give it a number with decimals (double)
        motor1.setPower(speed);//sets motor1 to the number you gave it
    }


    public void setServo1(double position) {
        servo1.setPosition(position);//servos don't use power or speed bc they are weird
    }


    public void setServo2(double speed){
        servo2.setPower(speed);//continuous rotation
    }
}
//always between the first bracket and this one^
