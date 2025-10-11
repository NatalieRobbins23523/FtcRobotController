package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Intake", group = "TeleOp")
public class Intake extends LinearOpMode {

    private CRServo servo1;

    @Override
    public void runOpMode() {
        // Initialize the servo
        servo1 = hardwareMap.get(CRServo.class, "servo1");

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Check if 'A' button is pressed
            if (gamepad1.a) {
                servo1.setPower(1.0);  // Run intake forward
            } else if (gamepad1.b) {
                servo1.setPower(-1.0); // Run intake in reverse
            } else {
                servo1.setPower(0.0);  // Stop intake
            }
        }
    }
}

