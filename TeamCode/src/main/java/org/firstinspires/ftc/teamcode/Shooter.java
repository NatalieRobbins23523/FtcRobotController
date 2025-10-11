package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Shooter")
public class Shooter extends LinearOpMode {  // Removed 'abstract'


    private DcMotor MotorLeftShoot;
    private DcMotor MotorRightShoot;


    @Override
    public void runOpMode() {
        MotorLeftShoot = hardwareMap.get(DcMotor.class, "MotorLeftShoot");


        waitForStart();


        while (opModeIsActive()) {
            if (gamepad1.x) {
                setMotorLeftShootPower(-1.0);
                setMotorRightShootPower(1.0);// Run forward
            }
            if(gamepad1.y){
                setMotorLeftShootPower(0.0);
                setMotorRightShootPower(0.0);// Stop when y is pressed
            }
        }
    }


    public void setMotorLeftShootPower(double power) {
        MotorLeftShoot.setPower(power);  // Use setPower for CRServo
    }
    public void setMotorRightShootPower(double power) {
        MotorRightShoot.setPower(power);
    }
}

