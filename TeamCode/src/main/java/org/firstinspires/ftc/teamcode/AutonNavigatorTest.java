package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// Autonomous OpMode
// Autonomous OpMode
@Autonomous(name = "Autonomous Navigator Comprehensive Test", group = "Concept")
public class AutonNavigatorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Starting position (Assuming starting at (0,0) with heading 0°)
        double initialX = 0.0;        // in feet
        double initialY = 0.0;        // in feet
        double initialHeading = 0.0;  // in degrees

        // Initialize Autonomous Navigator with initial position
        AutonomousNavigator navigator = new AutonomousNavigator(this, telemetry, initialX, initialY, initialHeading);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Movement 1: Move forward 2 feet
            telemetry.addData("Test 1", "Moving forward 2 feet");
            telemetry.update();
            navigator.moveToRelativePosition(2.0, 0.0, 0.0);

            // Movement 2: Move right 2 feet
            telemetry.addData("Test 2", "Moving right 2 feet");
            telemetry.update();
            navigator.moveToRelativePosition(0.0, 2.0, 0.0);

            // Movement 3: Rotate 90 degrees
            telemetry.addData("Test 3", "Rotating 90 degrees");
            telemetry.update();
            navigator.moveToRelativePosition(0.0, 0.0, 90.0);

            // Movement 4: Move forward 2 feet after rotation
            telemetry.addData("Test 4", "Moving forward 2 feet after rotation");
            telemetry.update();
            navigator.moveToRelativePosition(2.0, 0.0, 0.0);

            // Movement 5: Move backward 1 foot
            telemetry.addData("Test 5", "Moving backward 1 foot");
            telemetry.update();
            navigator.moveToRelativePosition(-1.0, 0.0, 0.0);

            // Movement 6: Rotate -45 degrees
            telemetry.addData("Test 6", "Rotating -45 degrees");
            telemetry.update();
            navigator.moveToRelativePosition(0.0, 0.0, -45.0);

            // Movement 7: Move diagonally (forward and right)
            telemetry.addData("Test 7", "Moving diagonally forward-right 1.5 feet");
            telemetry.update();
            navigator.moveToRelativePosition(1.5, 1.5, 0.0);

            // Stop the navigator
            navigator.stop();

            telemetry.addData("Test Complete", "All movements executed");
            telemetry.update();
        }
    }
}
