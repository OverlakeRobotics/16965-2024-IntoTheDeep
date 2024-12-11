package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

public class RobotControllerNew {
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final IMU gyro;
    private final VisionPortal visionPortal;
    private final Position cameraPosition;
    private final YawPitchRollAngles cameraOrientation;
    private final ElapsedTime runtime = new ElapsedTime();
    private final AprilTagProcessor aprilTag;
    private final PIDFController headingController;

    // Constants
    private static final double FORWARD_COUNTS_PER_INCH = 43.80;
    private static final double STRAFE_COUNTS_PER_INCH = 50.58;
    private static final double MAX_CORRECTION_ERROR = 1.0;
    private static final double MIN_DIST_TO_STOP = 0.5;
    private static final double POSITION_TOLERANCE = 0.75;
    private static final double HEADING_PID_KP = 0.05;
    private static final double HEADING_PID_KI = 0.0007;
    private static final double HEADING_PID_KD = 0.009;

    // State tracking
    private DriveState currentState = DriveState.IDLE;
    private double targetX, targetY, targetHeading;
    private double moveSpeed;
    private int currentAprilTagID;
    private final int[] motorTargets = new int[4];
    private boolean isFieldCentric = true;

    private enum DriveState {
        IDLE,
        DISTANCE_DRIVE,
        TURNING,
        APRIL_TAG_DRIVE,
        CONTINUOUS_DRIVE
    }

    public RobotControllerNew(HardwareMap hardwareMap, Position cameraPosition, YawPitchRollAngles cameraOrientation) {
        this.backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        this.backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");;
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");;
        this.frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");;
        this.gyro = hardwareMap.get(IMU.class, "imu2");;
        this.cameraPosition = cameraPosition;
        this.cameraOrientation = cameraOrientation;

        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        gyro.resetYaw();

        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTag)
                .build();

        headingController = new PIDFController(HEADING_PID_KP, HEADING_PID_KI, HEADING_PID_KD, 0);
        headingController.setTolerance(1.0);

    }

    public void setFieldCentric(boolean isFieldCentric) {
        this.isFieldCentric = isFieldCentric;
    }

    public void startContinuousDrive(double forward, double strafe, double turn) {
        currentState = DriveState.CONTINUOUS_DRIVE;

        if (isFieldCentric) {
            double heading = getHeading();
            double adjustedForward = forward * Math.cos(Math.toRadians(heading)) +
                    strafe * Math.sin(Math.toRadians(heading));
            double adjustedStrafe = -forward * Math.sin(Math.toRadians(heading)) +
                    strafe * Math.cos(Math.toRadians(heading));
            setMotorPowers(adjustedForward, adjustedStrafe, turn);
        } else {
            setMotorPowers(forward, strafe, turn);
        }
    }

    public void startDistanceDrive(double distance, double direction, double speed, double holdHeading) {
        currentState = DriveState.DISTANCE_DRIVE;
        moveSpeed = speed;
        headingController.setSetPoint(holdHeading);
        headingController.reset();

        double forward = Math.cos(Math.toRadians(direction - holdHeading));
        double strafe = Math.sin(Math.toRadians(direction - holdHeading));

        double moveCountMult = Math.sqrt(Math.pow(Math.cos(direction * (Math.PI / 180)) * (1.0 / FORWARD_COUNTS_PER_INCH), 2) +
                Math.pow(Math.sin(direction * (Math.PI / 180)) * (1.0 / STRAFE_COUNTS_PER_INCH), 2));

        int forwardCounts = (int)(forward * distance / moveCountMult);
        int strafeCounts = (int)(strafe * distance / moveCountMult);

        motorTargets[0] = backLeft.getCurrentPosition() - forwardCounts + strafeCounts;
        motorTargets[1] = backRight.getCurrentPosition() - forwardCounts - strafeCounts;
        motorTargets[2] = frontLeft.getCurrentPosition() - forwardCounts - strafeCounts;
        motorTargets[3] = frontRight.getCurrentPosition() - forwardCounts + strafeCounts;

        setTargetPositions(motorTargets);
        setDrivePower(speed);
    }

    public void startTurnTo(double angle, double speed) {
        currentState = DriveState.TURNING;
        targetHeading = normalize(angle);
        moveSpeed = speed;
    }

    public void startAprilTagDrive(double x, double y, double heading, double speed) {
        currentState = DriveState.APRIL_TAG_DRIVE;
        targetX = x;
        targetY = y;
        targetHeading = heading;
        moveSpeed = speed;
        updateRobotPosition();
    }

    public void update() {
        switch (currentState) {
            case DISTANCE_DRIVE:
                if (isAtTargetPosition()) {
                    stop();
                } else {
                    double correction = headingController.calculate(normalize(getHeading()));
                    setMotorPowers(moveSpeed, 0, correction);
                }
                break;

            case TURNING:
                double angleError = normalize(targetHeading - getHeading());
                if (Math.abs(angleError) > MAX_CORRECTION_ERROR) {
                    double correction = moveSpeed * headingController.calculate(normalize(getHeading()));
                    setMotorPowers(0, 0, correction);
                } else {
                    stop();
                }
                break;

            case APRIL_TAG_DRIVE:
                updateRobotPosition();
                double distance = Math.hypot(targetX - getX(), targetY - getY());

                if (distance > MIN_DIST_TO_STOP) {
                    double dx = targetX - getX();
                    double dy = targetY - getY();
                    double moveDirection = Math.atan2(dx, dy);

                    double forward = -moveSpeed * Math.sqrt(distance) *
                            Math.cos(moveDirection - Math.toRadians(getHeading())) / 2;
                    double strafe = moveSpeed * Math.sqrt(distance) *
                            Math.sin(moveDirection - Math.toRadians(getHeading())) / 2;

                    setMotorPowers(forward, strafe, 0);
                } else {
                    stop();
                }
                break;
            case IDLE:
                double correction = moveSpeed * headingController.calculate(normalize(targetHeading - getHeading()));
                setMotorPowers(0, 0, correction);
                break;
            case CONTINUOUS_DRIVE:
                // Motor powers already set in startContinuousDrive
                break;
        }
    }

    private void updateRobotPosition() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == currentAprilTagID && detection.ftcPose != null) {
                targetX = detection.ftcPose.x;
                targetY = detection.ftcPose.y;
                targetHeading = detection.ftcPose.yaw;
            }
        }
    }

    public void stop() {
        currentState = DriveState.IDLE;
        setMotorPowers(0, 0, 0);
    }

    private void setMotorPowers(double forward, double strafe, double turn) {
        backLeft.setPower((forward + strafe - turn) / 3);
        backRight.setPower((forward - strafe + turn) / 3);
        frontLeft.setPower((forward - strafe - turn) / 3);
        frontRight.setPower((forward + strafe + turn) / 3);
    }

    private void setTargetPositions(int[] targets) {
        backLeft.setTargetPosition(targets[0]);
        backRight.setTargetPosition(targets[1]);
        frontLeft.setTargetPosition(targets[2]);
        frontRight.setTargetPosition(targets[3]);
    }

    private void setDrivePower(double power) {
        backLeft.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power);
        frontRight.setPower(power);
    }

    private static double normalize(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees <= -180) degrees += 360;
        return degrees;
    }

    public boolean isAtTargetPosition() {
        return Math.abs(backLeft.getCurrentPosition() - motorTargets[0]) < POSITION_TOLERANCE &&
                Math.abs(backRight.getCurrentPosition() - motorTargets[1]) < POSITION_TOLERANCE &&
                Math.abs(frontLeft.getCurrentPosition() - motorTargets[2]) < POSITION_TOLERANCE &&
                Math.abs(frontRight.getCurrentPosition() - motorTargets[3]) < POSITION_TOLERANCE;
    }

    public boolean isBusy() {
        return currentState != DriveState.IDLE;
    }

    public double getHeading() {
        return normalize(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    public double getX() {
        return targetX;
    }

    public double getY() {
        return targetY;
    }

    public int getCurrentAprilTagID() {
        return currentAprilTagID;
    }
}