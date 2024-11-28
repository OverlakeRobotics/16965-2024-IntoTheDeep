package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Opposite Servos Control", group="TeleOp")
public class SecondHang extends LinearOpMode {

    // Declare the servos
    private Servo servo1;
    private Servo servo2;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables
        servo1 = hardwareMap.get(Servo.class, "hangServo1");
        servo2 = hardwareMap.get(Servo.class, "hangServo2");

        servo1.setDirection(Servo.Direction.REVERSE);
        servo2.setDirection(Servo.Direction.FORWARD);



        // Set both servos to position 0 in init
        servo1.setPosition(0);
        servo2.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                servo1.setPosition(0);
                servo2.setPosition(0);
            } else if (gamepad1.b) {
                servo1.setPosition(1);
                servo2.setPosition(1);
            }

            // Telemetry data for debugging
            telemetry.addData("Servo1 Position", servo1.getPosition());
            telemetry.addData("Servo2 Position", servo2.getPosition());
            telemetry.update();
        }
    }
}