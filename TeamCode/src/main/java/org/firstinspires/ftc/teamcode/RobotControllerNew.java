package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="Robot: Auto Drive By Gyro (Mecanum with Strafe)", group="Robot")
public class RobotControllerNew extends LinearOpMode {

    private DcMotor frontLeftDrive   = null;
    private DcMotor frontRightDrive  = null;
    private DcMotor backLeftDrive    = null;
    private DcMotor backRightDrive   = null;
    private IMU     imu             = null;

    private double  headingError  = 0;

    // Telemetry targets
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  strafeSpeed   = 0;
    private double  turnSpeed     = 0;

    // Approx counts per revolution for GoBILDA 435RPM Yellow Jacket
    static final double     COUNTS_PER_MOTOR_REV    = 383.6;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //        (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double FORWARD_COUNTS_PER_INCH = 32.64;
    static final double STRAFE_COUNTS_PER_INCH = 38.89;
    static final double COUNTS_LEFT_TO_SLOW_DOWN = 1500;

    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.2;
    static final double     HEADING_THRESHOLD       = 1.0 ;

    static final double     P_TURN_GAIN             = 0.02;
    static final double     P_DRIVE_GAIN            = 0.03;

    @Override
    public void runOpMode() {

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        frontRightDrive = hardwareMap.get(DcMotor.class, "FRONTRIGHT");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "BACKLEFT");
        backRightDrive  = hardwareMap.get(DcMotor.class, "BACKRIGHT");

        // Set directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu2");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Reset encoders
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for start
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu.resetYaw();

        // Example movement:
        // Move 72 inches at moveAngle = 45 degrees while holding heading = 0
        //driveStraight(DRIVE_SPEED, 96, 0.0, 0.0);

        // Turn to heading 180 degrees
        //turnToHeading(TURN_SPEED, 180);

        // Move 72 inches at moveAngle = 135 degrees while holding heading = 180
        driveStraight(DRIVE_SPEED, 24, -90, 0.0);
        //driveStraight(DRIVE_SPEED, 96, 180, 0.0);
        //driveStraight(DRIVE_SPEED, 96, 90, 0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
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
    public void driveStraight(double maxDriveSpeed, double distance, double moveAngle, double heading) {
        if (!opModeIsActive()) return;

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
        while (opModeIsActive() &&
                frontLeftDrive.isBusy() && frontRightDrive.isBusy() &&
                backLeftDrive.isBusy() && backRightDrive.isBusy()) {

            double distanceToDestination = (Math.abs(blTarget - backLeftDrive.getCurrentPosition()) +
                    Math.abs(brTarget - backRightDrive.getCurrentPosition()) +
                    Math.abs(flTarget - frontLeftDrive.getCurrentPosition()) +
                    Math.abs(frTarget - frontRightDrive.getCurrentPosition())) / 4.0;

            telemetry.addData("distance in", distanceToDestination);

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
            //sendTelemetry(true);
        }

        // Stop motors and revert to RUN_USING_ENCODER
        moveRobot(0, 0, 0);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            moveRobot(0, 0, turnSpeed);
            sendTelemetry(false);
        }

        moveRobot(0, 0, 0);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
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
    }

    private void sendTelemetry(boolean straight) {
        if (straight) {
            telemetry.addData("Motion", "Drive w/ Strafe");
            telemetry.addData("FL Pos/Tgt", "%d / %d", frontLeftDrive.getCurrentPosition(), frontLeftDrive.getTargetPosition());
            telemetry.addData("FR Pos/Tgt", "%d / %d", frontRightDrive.getCurrentPosition(), frontRightDrive.getTargetPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading (Target:Current)", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error : Turn Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Drive:Strafe:Turn", "%5.2f : %5.2f : %5.2f", driveSpeed, strafeSpeed, turnSpeed);
        telemetry.update();
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}