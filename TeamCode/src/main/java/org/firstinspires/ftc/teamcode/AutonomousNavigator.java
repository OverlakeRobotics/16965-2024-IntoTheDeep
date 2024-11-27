package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AutonomousNavigator {

    // Hardware variables
    private DcMotor leftFrontDrive   = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor leftBackDrive    = null;
    private DcMotor rightBackDrive   = null;
    private IMU imu;

    // Localization
    private AprilTagLocalization aprilTagLocalization;

    private double startPosX;
    private double startPosY;
    private double startHeading;

    // Position tracking
    private double posX = 0.0;     // in feet
    private double posY = 0.0;     // in feet
    private double heading = 0.0;  // in degrees

    // PID Controllers
    private PIDController distancePID;
    private PIDController headingPID;

    // OpMode reference
    private LinearOpMode opMode;
    private Telemetry telemetry;


    public AutonomousNavigator(LinearOpMode opMode, Telemetry telemetry) {
        this(opMode, telemetry, 0.0, 0.0, 0.0);
    }

    public AutonomousNavigator(LinearOpMode opMode, Telemetry telemetry, double initialX, double initialY, double initialHeading) {
        this.opMode = opMode;
        this.telemetry = telemetry;

        // Initialize hardware
        initHardware();

        // Initialize AprilTag Localization
        aprilTagLocalization = new AprilTagLocalization(opMode.hardwareMap);

        // Initialize PID Controllers
        distancePID = new PIDController(0.5, 0.0, 0.05);
        headingPID = new PIDController(0.05, 0.0, 0.01);

        // Set output limits for PID controllers
        distancePID.setOutputLimits(-1.0, 1.0);
        headingPID.setOutputLimits(-1.0, 1.0);

        // Initialize IMU heading
        resetHeading();

        // Set the initial position and heading
        this.startPosX = initialX;
        this.startPosY = initialY;
        this.startHeading = initialHeading;

        // Initialize position tracking variables
        this.posX = initialX;
        this.posY = initialY;
        this.heading = initialHeading;
    }

    private void initHardware() {
        // Initialize motors
        leftFrontDrive  = opMode.hardwareMap.get(DcMotor.class, "FRONTLEFT");
        rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, "FRONTRIGHT");
        leftBackDrive   = opMode.hardwareMap.get(DcMotor.class, "BACKLEFT");
        rightBackDrive  = opMode.hardwareMap.get(DcMotor.class, "BACKRIGHT");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to run without encoders
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,    // Direction the REV Robotics logo is facing
                RevHubOrientationOnRobot.UsbFacingDirection.UP // Direction the USB ports are facing
        );
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
    }

    private void resetHeading() {
        imu.resetYaw();
        heading = 0.0;
    }
    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    // Main method to move to a target position
    public void moveToPosition(double targetX, double targetY, double targetHeading) {
        while (opMode.opModeIsActive()) {
            // Update robot's position
            updatePosition();

            // If position is invalid, cannot proceed
            if (Double.isNaN(posX) || Double.isNaN(posY)) {
                telemetry.addData("Position", "Invalid position, cannot navigate");
                telemetry.update();
                opMode.sleep(50);
                continue; // Skip to next iteration
            }

            // Calculate errors
            double deltaX = targetX - posX;
            double deltaY = targetY - posY;
            double distanceError = Math.hypot(deltaX, deltaY);
            double angleToTarget = Math.toDegrees(Math.atan2(deltaY, deltaX));

            double headingError = angleToTarget - heading;

            // Normalize heading error to [-180, 180]
            headingError = (headingError + 540) % 360 - 180;

            // Update PID controllers with current errors
            distancePID.setSetpoint(0.0); // We aim for zero distance error
            double distanceOutput = distancePID.calculate(distanceError);

            headingPID.setSetpoint(angleToTarget);
            double headingOutput = headingPID.calculate(heading);

            // Determine motor powers
            double drive = distanceOutput * Math.cos(Math.toRadians(headingError));
            double strafe = distanceOutput * Math.sin(Math.toRadians(headingError));
            double turn = headingOutput;

            // Apply motor powers
            moveRobot(drive, strafe, turn);

            // Telemetry for debugging
            telemetry.addData("Target Position", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("Current Position", "(%.2f, %.2f)", posX, posY);
            telemetry.addData("Distance Error", "%.2f", distanceError);
            telemetry.addData("Angle to Target", "%.2f", angleToTarget);
            telemetry.addData("Heading", "%.2f", heading);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Distance PID Output", "%.2f", distanceOutput);
            telemetry.addData("Heading PID Output", "%.2f", headingOutput);
            telemetry.addData("Drive Power", "%.2f", drive);
            telemetry.addData("Strafe Power", "%.2f", strafe);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.update();

            // Check if we have reached the target position
            if (distanceError < 0.1 && Math.abs(headingError) < 2) {
                break; // Exit loop when close enough to target
            }

            opMode.sleep(20); // Small delay to prevent CPU overload
        }

        // Stop the robot
        moveRobot(0, 0, 0);
    }

    public void moveToRelativePosition(double deltaX, double deltaY, double deltaHeading) {
        // Convert deltaX and deltaY from the robot's starting coordinate frame to the field coordinate frame
        double startHeadingRadians = Math.toRadians(startHeading);

        double deltaXField = deltaX * Math.cos(startHeadingRadians) - deltaY * Math.sin(startHeadingRadians);
        double deltaYField = deltaX * Math.sin(startHeadingRadians) + deltaY * Math.cos(startHeadingRadians);

        // Compute the target position in field coordinates
        double targetX = startPosX + deltaXField;
        double targetY = startPosY + deltaYField;
        double targetHeading = startHeading + deltaHeading;

        // Normalize targetHeading to be within -180 to 180 degrees
        targetHeading = normalizeAngle(targetHeading);

        // Use the existing moveToPosition method to move to the target position
        moveToPosition(targetX, targetY, targetHeading);
    }

    private void updatePosition() {
        // Get position from AprilTagLocalization
        aprilTagLocalization.updateRobotPosition();
        double tagX = aprilTagLocalization.getRobotX();
        double tagY = aprilTagLocalization.getRobotY();

        if (!Double.isNaN(tagX) && !Double.isNaN(tagY)) {
            // Update position from AprilTag
            posX = tagX;
            posY = tagY;

            // Update heading from IMU
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addData("Position Update", "Using AprilTag Localization");
            telemetry.addData("AprilTag Position", "(%.2f, %.2f)", posX, posY);
            telemetry.addData("IMU Heading", "%.2f", heading);

        } else {
            // AprilTagLocalization failed
            telemetry.addData("Position Update", "AprilTag Localization failed");
        }
    }

    private void moveRobot(double drive, double strafe, double turn) {
        // Calculate wheel powers
        double leftFrontPower  = drive + strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double leftBackPower   = drive - strafe + turn;
        double rightBackPower  = drive + strafe - turn;

        // Normalize motor powers if any value is greater than 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
        );
        if (maxPower > 1.0) {
            leftFrontPower  /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower   /= maxPower;
            rightBackPower  /= maxPower;
        }

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // Method to stop and release resources
    public void stop() {
        // Stop the robot
        moveRobot(0, 0, 0);

        // Close AprilTag Localization
        if (aprilTagLocalization != null) {
            aprilTagLocalization.close();
        }
    }
}