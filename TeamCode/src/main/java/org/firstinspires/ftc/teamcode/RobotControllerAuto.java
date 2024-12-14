package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class RobotControllerAuto {

    private static final double MIN_DIST_TO_STOP = 0.5;
    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;
    private double xPos;
    private double yPos;
    private double hPos;
    private int currentAprilTagID;
    private final LinearOpMode robot;
    private final AprilTagProcessor aprilTag;

    private double  headingError  = 0;

    // Telemetry targets
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  strafeSpeed   = 0;
    private double  turnSpeed     = 0;
    static final double FORWARD_COUNTS_PER_INCH = 32.64;
    static final double STRAFE_COUNTS_PER_INCH = 38.89;
    static final double COUNTS_LEFT_TO_SLOW_DOWN = 1500;

    static final double     DEFAULT_DRIVE_SPEED             = 0.8;
    static final double     DEFAULT_TURN_SPEED              = 0.5;
    static final double     HEADING_THRESHOLD       = 1.0 ;

    static final double     P_TURN_GAIN             = 0.02;
    static final double     P_DRIVE_GAIN            = 0.03;
    private int lastBackLeftEncoderCount;
    private int lastBackRightEncoderCount;
    private int lastFrontLeftEncoderCount;
    private int lastFrontRightEncoderCount;

    public RobotControllerAuto(DcMotorEx frontLeftDrive, DcMotorEx frontRightDrive,
                               DcMotorEx backLeftDrive, DcMotorEx backRightDrive,
                               IMU imu, WebcamName camera, double startX,
                               double startY, double startH, Position cameraOffset,
                               YawPitchRollAngles cameraAngles, LinearOpMode robot) {

        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.backLeftDrive = backLeftDrive;
        this.backRightDrive = backRightDrive;
        this.frontLeftDrive = frontLeftDrive;
        this.frontRightDrive = frontRightDrive;

        this.imu = imu;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraOffset, cameraAngles)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.addProcessor(aprilTag);

        this.robot = robot;

        this.hPos = startH;
        this.xPos = startX;
        this.yPos = startY;
    }


    /**
     * Drive a given distance in a specified field-oriented direction (moveAngle),
     * while maintaining a specified heading.
     *
     * @param maxDriveSpeed Max speed (0 to 1)
     * @param distance Distance in inches
     * @param moveAngle Field-oriented direction to move (in degrees, 0 = forward on field)
     * @param heading Desired robot heading to maintain
     */
    public void driveStraight(double distance, double moveAngle, double heading, double maxDriveSpeed) {
        if (!robot.opModeIsActive()) return;

        // Convert angles to radians
        double angleDiff = Math.toRadians(moveAngle - heading);

        // Compute local dx, dy relative to robot heading
        double dx = distance * Math.sin(angleDiff) * STRAFE_COUNTS_PER_INCH;
        double dy = distance * Math.cos(angleDiff) * FORWARD_COUNTS_PER_INCH;

        // Compute target increments for each wheel
        int flTarget = frontLeftDrive.getCurrentPosition()  + (int)(dy + dx);
        int frTarget = frontRightDrive.getCurrentPosition() + (int)(dy - dx);
        int blTarget = backLeftDrive.getCurrentPosition()   + (int)(dy - dx);
        int brTarget = backRightDrive.getCurrentPosition()  + (int)(dy + dx);

        // Set targets
        frontLeftDrive.setTargetPosition(flTarget);
        frontRightDrive.setTargetPosition(frTarget);
        backLeftDrive.setTargetPosition(blTarget);
        backRightDrive.setTargetPosition(brTarget);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Base drive and strafe speeds (normalized)
        // We'll start with desired vector at full maxDriveSpeed in that direction


        double rawDrive = Math.cos(angleDiff) * maxDriveSpeed;
        double rawStrafe = Math.sin(angleDiff) * maxDriveSpeed;

        // Loop while motors are running
        while (robot.opModeIsActive() &&
                frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                backLeftDrive.isBusy() && backRightDrive.isBusy()) {

            double distanceToDestination = (Math.abs(blTarget - backLeftDrive.getCurrentPosition()) +
                    Math.abs(brTarget - backRightDrive.getCurrentPosition()) +
                    Math.abs(flTarget - frontLeftDrive.getCurrentPosition()) +
                    Math.abs(frTarget - frontRightDrive.getCurrentPosition())) / 4.0;

            robot.telemetry.addData("distance in", distanceToDestination);

            double scaleFactor = Math.min(1.0, 0.2 + (distanceToDestination / COUNTS_LEFT_TO_SLOW_DOWN));
            double drive = rawDrive * scaleFactor;
            double strafe = rawStrafe * scaleFactor;


            // Apply heading correction
            double turnCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // Combine drive/strafe/turn
            // Might need to ensure max speed not exceeded
            double maxMag = Math.max(Math.abs(drive) , Math.abs(strafe));
            maxMag = Math.max(maxMag, Math.abs(turnCorrection));
            if (maxMag > 1.0) {
                drive /= maxMag;
                strafe /= maxMag;
                turnCorrection /= maxMag;
            }

            moveRobot(drive, strafe, turnCorrection);
            sendTelemetry(true);
        }

        // Stop motors and revert to RUN_USING_ENCODER
        moveRobot(0, 0, 0);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveStraight(double distance, double moveAngle, double heading) {
        driveStraight(distance, moveAngle, heading, DEFAULT_DRIVE_SPEED);
    }

    public void updateRobotPosition() {
        int backLeftEncoderCount = backLeftDrive.getCurrentPosition();
        int backRightEncoderCount = backRightDrive.getCurrentPosition();
        int frontLeftEncoderCount = frontLeftDrive.getCurrentPosition();
        int frontRightEncoderCount = frontRightDrive.getCurrentPosition();

        int deltaBackLeft = backLeftEncoderCount - lastBackLeftEncoderCount;
        int deltaBackRight = backRightEncoderCount - lastBackRightEncoderCount;
        int deltaFrontLeft = frontLeftEncoderCount - lastFrontLeftEncoderCount;
        int deltaFrontRight = frontRightEncoderCount - lastFrontRightEncoderCount;

        lastBackLeftEncoderCount = backLeftEncoderCount;
        lastBackRightEncoderCount = backRightEncoderCount;
        lastFrontLeftEncoderCount = frontLeftEncoderCount;
        lastFrontRightEncoderCount = frontRightEncoderCount;

        // If anything goes wrong, this is the most likely area to be wrong, check this
        double deltaForwardCounts = (deltaFrontLeft + deltaFrontRight + deltaBackLeft + deltaBackRight) / 4.0;
        double deltaStrafeCounts = (-deltaFrontLeft + deltaFrontRight + deltaBackLeft - deltaBackRight) / 4.0;

        double deltaForward = deltaForwardCounts / FORWARD_COUNTS_PER_INCH;
        double deltaStrafe = deltaStrafeCounts / STRAFE_COUNTS_PER_INCH;

        hPos = getHeading();

        double sinH = Math.sin(Math.toRadians(hPos));
        double cosH = Math.cos(Math.toRadians(hPos));

        // Here, something could go wrong too, make sure signs and such are correct.
        double deltaX = deltaStrafe * cosH - deltaForward * sinH;
        double deltaY = deltaStrafe * sinH + deltaForward * cosH;

        xPos += deltaX;
        yPos -= deltaY;

        updateRobotPositionWithApril();
    }

    public void updateRobotPositionWithApril() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Process detections that have a valid robotPose
        for (AprilTagDetection detection : detections) {
            if (detection.robotPose != null) {
                // Get robot's position from robotPose
                xPos = detection.robotPose.getPosition().x;
                yPos = detection.robotPose.getPosition().y;
                hPos = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                currentAprilTagID = detection.id;
                break;
            }
        }
    }

    public int getCurrentAprilTagID() {
        return currentAprilTagID;
    }

    public void aprilTagDrive(double wantedX, double wantedY, double wantedH, double speed) throws RuntimeException {
        double distance = Math.sqrt(Math.pow(wantedX - xPos, 2) + Math.pow(wantedY - yPos, 2));
        while ((distance > MIN_DIST_TO_STOP || Math.abs(getHeading() - wantedH) > 3.5) && robot.opModeIsActive()) {

            double dy = wantedY - yPos;
            double dx = wantedX - xPos;

            robot.telemetry.addData("dx", dx);
            robot.telemetry.addData("dy", dy);

            // Calculate move direction
            double moveDirection = Math.atan2(dx, dy);

            double forward = -speed * Math.sqrt(distance) * Math.cos((moveDirection - Math.toRadians(hPos))) / 2;
            double strafe = speed * Math.sqrt(distance) * Math.sin((moveDirection - Math.toRadians(hPos))) / 2;
            moveRobot(forward, strafe, getSteeringCorrection(wantedH, P_DRIVE_GAIN));
            distance = Math.sqrt(Math.pow(wantedX - xPos, 2) + Math.pow(wantedY - yPos, 2));
        }
        moveRobot(0, 0, 0);
    }

    public void turnToHeading(double heading, double maxTurnSpeed) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (robot.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0, 0);
    }
    public void turnToHeading(double heading) {
        turnToHeading(heading, DEFAULT_TURN_SPEED);
    }


    public void holdHeading(double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (robot.opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -DEFAULT_TURN_SPEED, DEFAULT_TURN_SPEED);
            moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;
        headingError = targetHeading - getHeading();

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return -Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Move the robot in mecanum style: drive (forward), strafe (right), turn (CW)
     * Positive drive = forward
     * Positive strafe = right
     * Positive turn = rotate CW
     */
    public void moveRobot(double drive, double strafe, double turn) {
        driveSpeed = drive;
        strafeSpeed = strafe;
        turnSpeed  = turn;

        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        // Normalize speeds if any exceed 1.0
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);

        updateRobotPosition();
    }

    private void sendTelemetry(boolean straight) {
        if (straight) {
            robot.telemetry.addData("Motion", "Drive w/ Strafe");
            robot.telemetry.addData("FL Pos/Tgt", "%d / %d", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition());
            robot.telemetry.addData("FR Pos/Tgt", "%d / %d", frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition());
        } else {
            robot.telemetry.addData("Motion", "Turning");
        }

        robot.telemetry.addData("Heading (Target:Current)", "%5.2f : %5.0f", targetHeading, getHeading());
        robot.telemetry.addData("Error : Turn Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        robot.telemetry.addData("Drive:Strafe:Turn", "%5.2f : %5.2f : %5.2f", driveSpeed, strafeSpeed, turnSpeed);
        robot.telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}