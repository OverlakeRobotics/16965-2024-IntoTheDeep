package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ClawTest", group="TeleOp")
public class ClawTest extends LinearOpMode {
    private Servo clawLeft = null;
    private Servo clawRight = null;
    @Override
    public void runOpMode() {
        clawLeft = hardwareMap.get(Servo.class, "CLAWLEFT");
        clawRight = hardwareMap.get(Servo.class, "CLAWRIGHT");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x) {
                openClaw();
            }
            else if (gamepad1.b) {
                closeClaw();
            }
            telemetry.addData("Claw Left Position", clawLeft.getPosition());
            telemetry.addData("Claw Right Position", clawRight.getPosition());
            telemetry.update();
        }
    }
    private void openClaw() {
        clawLeft.setPosition(0.3);   // Adjust these values based on your servo configuration
        clawRight.setPosition(0.7);
    }
    private void closeClaw() {
        clawLeft.setPosition(0.5);   // Adjust these values based on your servo configuration
        clawRight.setPosition(0.5);
    }
}