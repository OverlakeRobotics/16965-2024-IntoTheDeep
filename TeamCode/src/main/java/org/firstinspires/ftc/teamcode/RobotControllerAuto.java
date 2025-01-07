package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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

@Config
public class RobotControllerAuto {

    private static final double MIN_DIST_TO_STOP = 3.0;
    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final IMU imu;
    private double xPos;
    private double yPos;
    private double hPos;
    private double errorX;
    private double errorY;
    private int currentAprilTagID;
    private final LinearOpMode robot;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final PIDController aprilTagPIDStrafe;
    private final PIDController aprilTagPIDForward;

    private double  headingError  = 0;

    // Telemetry targets
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  strafeSpeed   = 0;
    private double  turnSpeed     = 0;
    static final double FORWARD_COUNTS_PER_INCH = 32.64;
    static final double STRAFE_COUNTS_PER_INCH = 38.89;
    static final int COUNTS_LEFT_TO_SLOW_DOWN = 500;

    static final double     DEFAULT_DRIVE_SPEED             = 0.8;
    static final double     DEFAULT_TURN_SPEED              = 0.5;
    static final double     HEADING_THRESHOLD       = 1.0;

    static final double     P_TURN_GAIN             = 0.02;
    static final double     P_DRIVE_GAIN            = 0.03;
    public static double Kp_APRIL_FORWARD = 0.04; // 0.034;
    public static double Kd_APRIL_FORWARD= 0.000839;
    public static double Ki_APRIL_FORWARD = 0; // 0.09;

    public static double Kp_APRIL_STRAFE = 0.05; // 0.04;
    public static double Kd_APRIL_STRAFE= 0.001;
    public static double Ki_APRIL_STRAFE = 0; // 0.14;
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

        this.aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraOffset, cameraAngles)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.addProcessor(this.aprilTag);

        visionPortal = builder.build();

        aprilTagPIDStrafe = new PIDController(Kp_APRIL_STRAFE, Ki_APRIL_STRAFE, Kd_APRIL_STRAFE);
        aprilTagPIDForward = new PIDController(Kp_APRIL_FORWARD, Ki_APRIL_FORWARD, Kd_APRIL_FORWARD);

        this.robot = robot;

        this.hPos = startH;
        this.xPos = startX;
        this.yPos = startY;
    }


    /**
     * Drive a given distance in a specified field-oriented direction (moveAngle)
     * while maintaining a specified heading.
     *
     * @param maxDriveSpeed Max speed (0 to 1)
     * @param distance Distance in inches
     * @param moveAngle Field-oriented direction to move (in degrees, 0 = forward on field)
     * @param heading Desired robot heading to maintain
     */
    public void driveStraight(double distance, double moveAngle, double maxDriveSpeed, double heading, boolean doSlowDown) {
        if (!robot.opModeIsActive()) return;

        // Uses errorX and errorY to calculate a new distance and moveAngle, adjusting the endpoint by errorX and errorY.
        // Check to make sure this is right if something goes wrong.
        if (errorX != 0 || errorY != 0) {
            // Convert angle to radians
            double angleRad = Math.toRadians(moveAngle);
            // Calculate original endpoint
            double x = distance * Math.cos(angleRad);
            double y = distance * Math.sin(angleRad);
            // Shift the point by dx and dy
            double xNew = x + errorX;
            double yNew = y + errorY;
            // Calculate new distance and angle
            distance = Math.sqrt(xNew * xNew + yNew * yNew);
            moveAngle = Math.atan2(yNew, xNew);
            moveAngle = Math.toDegrees(moveAngle);
            errorX = 0;
            errorY = 0;
        }

        moveAngle = normalize(-moveAngle);
        // Convert angles to radians
        double angleDiff = Math.toRadians(moveAngle - heading);

        // Compute local dx, dy relative to robot heading

        // Possibly test using this to see if it works better?
        // double strafeInches = dx * Math.cos(angleDiff) + dy * Math.sin(angleDiff)
        // double forwardInches = -dx * Math.sin(angleDiff) + dy * Math.cos(angleDiff)
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
        do {
            double distanceToDestination = (Math.abs(blTarget - backLeftDrive.getCurrentPosition()) +
                    Math.abs(brTarget - backRightDrive.getCurrentPosition()) +
                    Math.abs(flTarget - frontLeftDrive.getCurrentPosition()) +
                    Math.abs(frTarget - frontRightDrive.getCurrentPosition())) / 4.0;

            robot.telemetry.addData("distance in", distanceToDestination);
            double scaleFactor = Math.min(1.0, 0.2 + (distanceToDestination / COUNTS_LEFT_TO_SLOW_DOWN));
            if (!doSlowDown) scaleFactor = 1;
            double drive = rawDrive * scaleFactor;
            double strafe = rawStrafe * scaleFactor;


            // Apply heading correction
            double turnCorrection = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // Realized we don't need this as it already scales it in the moveRobot function.
            // Combine drive/strafe/turn
            // Might need to ensure max speed not exceeded
//            double maxMag = Math.max(Math.abs(drive) , Math.abs(strafe));
//            maxMag = Math.max(maxMag, Math.abs(turnCorrection));
//            if (maxMag > 1.0) {
//                drive /= maxMag;
//                strafe /= maxMag;
//                turnCorrection /= maxMag;
//            }

            // Why are we driving with any strafe power? Is this not meant to just power the robot? Wouldn't driving with strafe mess it up?
            moveRobot(drive, strafe, turnCorrection);
            sendTelemetry(true);

            if (distanceToDestination < 60) {
                break;
            }
        } while (robot.opModeIsActive() &&
                (frontLeftDrive.isBusy() || frontRightDrive.isBusy() ||
                backLeftDrive.isBusy() || backRightDrive.isBusy()));

        // Stop motors and revert to RUN_USING_ENCODER
        if (doSlowDown) {
            moveRobot(0, 0, 0);
        }
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveStraight(double distance, double moveAngle, double heading) {
        driveStraight(distance, moveAngle, DEFAULT_DRIVE_SPEED, heading, true);
    }

    public void driveStraight(double distance, double moveAngle, double maxDriveSpeed, double heading) {
        driveStraight(distance, moveAngle, maxDriveSpeed, heading, true);
    }

    // Implementation of what david was talking about. Instead of lining up with a point, it tracks
    // how far off it was at the end of the drive, so that during the next drive, it can correct for
    // the error using the errorX and errorY fields. This should be faster.
    //
    // One note is that for this to work the distances given by the april tag have to be correct,
    // so if it is not working by way overshooting or undershooting the target, verify that if you
    // move the robot 1 inch, there is a 1 inch change in its position according to the april tag.
    public void aprilDriveErrorTracking(double wantedX, double wantedY, double wantedH, double speed) {
        updateRobotPositionWithApril();
        double dy = wantedY - yPos; // neg
        double dx = wantedX - xPos; // neg
        double moveDirection = Math.toDegrees(Math.atan2(-dx, -dy));
        // start temp block
        // This is just to see values, remove this for actual program
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//        moveRobot(0, 0, 0);
//        while (robot.opModeIsActive() && (holdTimer.seconds() < 5)) {
//            robot.telemetry.addData("dx", dx);
//            robot.telemetry.addData("xPos", xPos);
//            robot.telemetry.addData("dy", dy);
//            robot.telemetry.addData("yPos", yPos);
//            robot.telemetry.addData("moveDirec", moveDirection);
//            robot.telemetry.update();
//        }
        // end temp block
        double distance = Math.hypot(dx, dy);
        driveStraight(distance, moveDirection, speed, wantedH);
        dy = wantedY - yPos; // neg
        dx = wantedX - xPos; // neg
        moveDirection = Math.toDegrees(Math.atan2(-dx, -dy));
        distance = Math.hypot(dx, dy);
        driveStraight(distance, moveDirection, speed, wantedH);
        errorX = wantedX - xPos;
        errorY = wantedY - yPos;
    }

    private static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
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
                hPos = getHeading();
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
        int consecutiveCorrectIterations = 0;
        aprilTagPIDForward.reset();
        aprilTagPIDStrafe.reset();

        while (consecutiveCorrectIterations < 3 && robot.opModeIsActive()) {
            if (distance > MIN_DIST_TO_STOP || Math.abs(getHeading() - wantedH) > HEADING_THRESHOLD) {
                consecutiveCorrectIterations = 0;
            } else {
                consecutiveCorrectIterations++;
            }
            double dy = wantedY - yPos;
            double dx = wantedX - xPos;

            robot.telemetry.addData("dx", dx);
            robot.telemetry.addData("dy", dy);
            robot.telemetry.update();

            // Calculate move direction
            double moveDirection = Math.atan2(dx, dy);

            double forward = -speed * aprilTagPIDForward.calculate(yPos, wantedY);
            double strafe = (STRAFE_COUNTS_PER_INCH / FORWARD_COUNTS_PER_INCH) * speed * aprilTagPIDStrafe.calculate(xPos, wantedX);
            Log.d("April Tag", "forward: " + forward);
            Log.d("April Tag", "strafe: " + strafe);
            Log.d("April Tag", "strafe pid: " + aprilTagPIDStrafe.calculate(xPos, wantedX));
            Log.d("April Tag", "forward pid: " + aprilTagPIDForward.calculate(yPos, wantedY));
            Log.d("April Tag", "dx: " + dx + ", dy: " + dy);
            moveRobot(forward, strafe, getSteeringCorrection(wantedH, P_DRIVE_GAIN));
            distance = Math.sqrt(Math.pow(wantedX - xPos, 2) + Math.pow(wantedY - yPos, 2));
        }
        moveRobot(0, 0, 0);
        errorX = wantedX - xPos;
        errorY = wantedY - yPos;
    }

    public void aprilTagDriveEncoders(double wantedX, double wantedY, double wantedH, double speed) throws RuntimeException {
        double dy = wantedY - yPos;
        double dx = wantedX - xPos;

        double distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        double currentHeading = getHeading();
        double moveAngle = Math.toDegrees(Math.atan2(dx, dy));

        // I think angleDiff is off and that might be why this function does not work
        double angleDiff = Math.toRadians(normalize(moveAngle - currentHeading));

        // If this doesn't work go back to what is used in the driveStraight method.
        // Maybe also mess around with negatives or swapping sines and cosines.
        // Compute local dx, dy relative to robot heading
        double strafeCounts = dx * Math.sin(angleDiff) * STRAFE_COUNTS_PER_INCH;
        double forwardCounts = dy * Math.cos(angleDiff) * FORWARD_COUNTS_PER_INCH;

        // Compute target increments for each wheel
        int flTarget = frontLeftDrive.getCurrentPosition()  + (int)(forwardCounts + strafeCounts);
        int frTarget = frontRightDrive.getCurrentPosition() + (int)(forwardCounts - strafeCounts);
        int blTarget = backLeftDrive.getCurrentPosition()   + (int)(forwardCounts - strafeCounts);
        int brTarget = backRightDrive.getCurrentPosition()  + (int)(forwardCounts + strafeCounts);

        frontLeftDrive.setTargetPosition(flTarget);
        frontRightDrive.setTargetPosition(frTarget);
        backLeftDrive.setTargetPosition(blTarget);
        backRightDrive.setTargetPosition(brTarget);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int consecutiveCorrectIterations = 0;
        while (consecutiveCorrectIterations < 3 && robot.opModeIsActive()) {
            if (distance > MIN_DIST_TO_STOP || Math.abs(normalize(currentHeading - wantedH)) > HEADING_THRESHOLD) {
                consecutiveCorrectIterations = 0;
            } else {
                consecutiveCorrectIterations++;
            }
            dy = wantedY - yPos;
            dx = wantedX - xPos;

            currentHeading = getHeading();
            moveAngle = Math.toDegrees(Math.atan2(dx, dy));

            angleDiff = Math.toRadians(moveAngle - currentHeading);

            strafeCounts = dx * Math.sin(angleDiff) * STRAFE_COUNTS_PER_INCH;
            forwardCounts = dy * Math.cos(angleDiff) * FORWARD_COUNTS_PER_INCH;

            // Compute target increments for each wheel
            flTarget = frontLeftDrive.getCurrentPosition()  + (int)(forwardCounts + strafeCounts);
            frTarget = frontRightDrive.getCurrentPosition() + (int)(forwardCounts - strafeCounts);
            blTarget = backLeftDrive.getCurrentPosition()   + (int)(forwardCounts - strafeCounts);
            brTarget = backRightDrive.getCurrentPosition()  + (int)(forwardCounts + strafeCounts);

            frontLeftDrive.setTargetPosition(flTarget);
            frontRightDrive.setTargetPosition(frTarget);
            backLeftDrive.setTargetPosition(blTarget);
            backRightDrive.setTargetPosition(brTarget);

            moveRobot(speed, 0, getSteeringCorrection(wantedH, P_DRIVE_GAIN));

            distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
        }

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

        while (robot.opModeIsActive() && (holdTimer.seconds() < holdTime)) {
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

//        updateRobotPositionWithApril();
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