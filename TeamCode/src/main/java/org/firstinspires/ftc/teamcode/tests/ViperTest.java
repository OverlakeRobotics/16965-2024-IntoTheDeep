package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Set to Zero")
public class ViperTest extends LinearOpMode {
    private Servo myServo;
    private Servo myServo2;

    @Override
    public void runOpMode() {
        // Initialize the servo
        myServo = hardwareMap.get(Servo.class, "SERVO1");
        myServo2 = hardwareMap.get(Servo.class, "SERVO2");

        // Wait for the start button to be pressed
        waitForStart();

        // Set the servo position to 0
        myServo.setPosition(0.0);
        myServo2.setPosition(0.0);

        if (gamepad1.a) {
            myServo.setPosition(1.0);
            myServo2.setPosition(1.0);
        }


        // Keep the op mode running
        while (opModeIsActive()) {
            telemetry.addData("Servo Position", myServo.getPosition());
            telemetry.addData("Servo Position 2", myServo2.getPosition());
            telemetry.update();
        }
    }
}