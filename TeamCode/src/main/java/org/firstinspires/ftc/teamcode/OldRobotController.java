// This is the robot controller class. This class will contain all methods needed to control the robot
// so you can just instantiate this and use it in other files. Some examples include driving,
// getting the current heading, and moving any other motors on the robot.

// Remember to comment with B.E.R.P:
//      B: Behavior, what the method does.
//      E: Exceptions, what exceptions the method throws.
//      R: Return, what the method returns.
//      P: Parameters, the list of parameters and explanations of them.

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class OldRobotController {
    public static final boolean DEFAULT_FIELD_CENTRIC = true;
    public static final boolean DEFAULT_SEND_TELEMETRY = true;
    public static final double FORWARD_COUNTS_PER_INCH = 43.80;
    public static final double STRAFE_COUNTS_PER_INCH = 50.58;
    public static final double DEFAULT_HEADING_CORRECTION_POWER = 2.0;
    public static final double MAX_CORRECTION_ERROR = 1.0;
    public static final double TURN_SPEED_RAMP = 4.0;
    public static final double MIN_VELOCITY_TO_SMOOTH_TURN = 115;
    public static final double INCHES_LEFT_TO_SLOW_DOWN = 8;
    public static final double INCHES_LEFT_TO_SPEED_UP = 5;
    public static final double TURN_DRIFT_TIME = 0;
    public static final double MIN_DIST_TO_STOP = 0.1;
    public static double Kp = 0.05;
    public static double Kd = 0.009;
    public static double Ki = 0.0007;
    public static final double ANGULAR_SCALAR = 0.9954681619;
    public static double LINEAR_SCALAR = 1.127;

    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final IMU gyro;
    private final LinearOpMode robot;
    private final ElapsedTime runtime;
    private final ElapsedTime PIDTimer;
    private final ElapsedTime angularVelocityTimer;

    private double wantedHeading;
    private double currentForward;
    private double currentStrafe;
    private double currentTurn;
    private double currentAngularVelocity;
    private double lastHeading;
    private double integralSum;
    private double lastError;
    private double turnStartedTime;
    private double turnStoppedTime;
    private double headingOffset;
    private int kTuner;


    // Create the controller with all the motors needed to control the robot. If another motor,
    // servo, or sensor is added, put that in here so the class can access it.
    public OldRobotController(DcMotorEx backLeft, DcMotorEx backRight,
                              DcMotorEx frontLeft, DcMotorEx frontRight,
                              IMU gyro, LinearOpMode robot) {

        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.backLeft = backLeft;
        this.backRight = backRight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;

        this.gyro = gyro;
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        gyro.initialize(params);
        gyro.resetYaw();
//        this.headingOffset = -gyro.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
        this.robot = robot;
        this.wantedHeading = getAngleImuDegrees();

        this.runtime = new ElapsedTime();
        this.runtime.reset();

        this.PIDTimer = new ElapsedTime();
        this.PIDTimer.reset();

        this.angularVelocityTimer = new ElapsedTime();
        this.angularVelocityTimer.reset();
    }

    // Overloaded constructor to create the robot controller without a LinearOpMode.
    // You cannot use distanceDrive or turnTo without a LinearOpMode.
    public OldRobotController(DcMotorEx backLeft, DcMotorEx backRight,
                              DcMotorEx frontLeft, DcMotorEx frontRight,
                              IMU gyro) {
        this(backLeft, backRight, frontLeft, frontRight, gyro, null);
    }

    // TODO: Tune PID values + other constants like TURN_DRIFT_TIME.
    // Behavior: Moves the robot using a given forward, strafe, and turn power.
    // Params:
    //      - double forward: The forward power for the robot.
    //      - double strafe: The strafe power for the robot.
    //      - double turn: The turn power fo the robot.
    //      - double headingCorrectionPower: The speed of heading correction.
    //      - boolean isTelemetry: Whether or not this method will send telemetry.
    private void move(double forward, double strafe, double turn, double headingCorrectionPower,
                      boolean isTelemetry) {
        double currentHeading = getAngleImuDegrees();

        // Calculate angular velocity in degrees per second.
        currentAngularVelocity = Math.abs(normalize(currentHeading - lastHeading) / angularVelocityTimer.seconds());
        angularVelocityTimer.reset();

        // Make it so that at the start of the turn its slower.
        if (turn != 0) {
            if (turnStartedTime == 0) {
                turnStartedTime = runtime.seconds();
            }
            turnStoppedTime = 0;
        } else {
            if (turnStoppedTime == 0) {
                turnStoppedTime = runtime.seconds();
            }
            turnStartedTime = 0;
        }
        double turnTime = runtime.seconds() - turnStartedTime;
        if (turn != 0 && TURN_SPEED_RAMP * turnTime < Math.PI/2) {
            turn *= Math.sin(TURN_SPEED_RAMP * turnTime);
        }

        // The wanted heading is not only set when the controller turn input is put in, but also
        // when the robot turns fast. This ensures that the robot doesn't set back as much after
        // stopping, and if it gets hit by something super fast, it doesn't try to correct its
        // heading back into that object.
        if ((robot == null && currentAngularVelocity > MIN_VELOCITY_TO_SMOOTH_TURN) || turn != 0 || runtime.seconds() - turnStoppedTime < TURN_DRIFT_TIME) {
            wantedHeading = currentHeading;
        } else {
            double error = normalize(wantedHeading - currentHeading);
            double derivative = (error - lastError) / PIDTimer.seconds();
            integralSum += error * PIDTimer.seconds();

            double headingCorrection = -((Kp * error) + (Kd * derivative) + (Ki * integralSum));
            headingCorrection = normalize(headingCorrection);
            lastError = error;
            PIDTimer.reset();

            turn = headingCorrectionPower * (Double.isNaN(headingCorrection) ? 0.0 : headingCorrection);
        }

        // Set fields so the robot knows what its current forward, strafe, and turn is in other methods.
        currentForward = forward;
        currentStrafe = strafe;
        currentTurn = turn;
        lastHeading = currentHeading;

        if (isTelemetry && robot != null) {
            sendTelemetry();
        }

        double backLeftPower = Range.clip((forward + strafe - turn) / 3, -1.0, 1.0);
        double backRightPower = Range.clip((forward - strafe + turn) / 3, -1.0, 1.0);
        double frontLeftPower = Range.clip((forward - strafe - turn) / 3, -1.0, 1.0);
        double frontRightPower = Range.clip((forward + strafe + turn) / 3, -1.0, 1.0);

        // * 2000 to convert from power to velocity
        backLeft.setVelocity(2000 * backLeftPower);
        backRight.setVelocity(2000 * backRightPower);
        frontLeft.setVelocity(2000 * frontLeftPower);
        frontRight.setVelocity(2000 * frontRightPower);
    }

    // Behavior: Overloaded method of move. This sets the default of isTelemetry.
    // Params:
    //      - double forward: The forward power for the robot.
    //      - double strafe: The strafe power for the robot.
    //      - double turn: The turn power fo the robot.
    //      - double headingCorrectionPower: The speed of heading correction.
    private void move(double forward, double strafe, double turn, double headingCorrectionPower) {
        move(forward, strafe, turn, headingCorrectionPower, DEFAULT_SEND_TELEMETRY);
    }

    // TODO: Find exact values for distance and implement it in COUNTS_PER_INCH to make this method precise.
    // TODO: Figure out error where when first starting, it drives straight then strafes.
    // Behavior: Drives the robot a given distance in a given direction without turning it.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    // Params:
    //      - double distance: The distance to drive the robot in inches.
    //      - double direction: The direction, in degrees, that the robot will drive in. This is
    //                          based on the direction the robot was initialized in.
    //      - double speed: The speed at which the robot will move.
    //      - boolean isFieldCentric: Determines whether the direction is based from the robot
    //                                or the field. If its field centric, the robot will always
    //                                move the same direction for the same inputted direction,
    //                                no matter what direction the robot is facing.
    public void distanceDrive(double distance, double direction, double speed, double holdHeading, boolean isFieldCentric)
            throws RuntimeException {
        if (robot == null) {
            throw new RuntimeException("Tried to run distanceDrive but LinearOpMode object not given!");
        }
        holdHeading = normalize(holdHeading);
        double currentHeading = getAngleImuDegrees();
        double startingHeading = holdHeading;
        // This still needs testing.
        double forward;
        double strafe;
        if (isFieldCentric) {
            forward = Math.cos((direction - currentHeading) * (Math.PI / 180));
            strafe = Math.sin((direction - currentHeading) * (Math.PI / 180));
        } else {
            forward = Math.cos(direction * (Math.PI / 180));
            strafe = Math.sin(direction * (Math.PI / 180));
        }

        double moveCountMult = Math.sqrt(Math.pow(Math.cos(direction * (Math.PI / 180)) * (1.0 / FORWARD_COUNTS_PER_INCH), 2) +
                Math.pow(Math.sin(direction * (Math.PI / 180)) * (1.0 / STRAFE_COUNTS_PER_INCH), 2));

        int forwardCounts = (int)(forward * distance / moveCountMult);
        int strafeCounts = (int)(strafe * distance / moveCountMult);


        int backLeftTarget = backLeft.getCurrentPosition() - forwardCounts + strafeCounts;
        int backRightTarget = backRight.getCurrentPosition() - forwardCounts - strafeCounts;
        int frontLeftTarget = frontLeft.getCurrentPosition() - forwardCounts - strafeCounts;
        int frontRightTarget = frontRight.getCurrentPosition() - forwardCounts + strafeCounts;

        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);
        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);

        backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while ((backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy())
                && robot.opModeIsActive()) {
            wantedHeading = holdHeading;
            double distanceToDestination = (Math.abs(backLeftTarget - backLeft.getCurrentPosition()) +
                    Math.abs(backRightTarget - backRight.getCurrentPosition()) +
                    Math.abs(frontLeftTarget - frontLeft.getCurrentPosition()) +
                    Math.abs(frontRightTarget - frontRight.getCurrentPosition())) / 4.0;
            double distanceToDestinationInches = 2 * Math.sqrt(Math.pow(Math.cos(direction) *
                    (distanceToDestination / FORWARD_COUNTS_PER_INCH), 2) + Math.pow(Math.sin(direction) *
                    (distanceToDestination / STRAFE_COUNTS_PER_INCH), 2));
            robot.telemetry.addData("Current Action", "Distance Driving");
            robot.telemetry.addData("Distance To Target", distanceToDestination * moveCountMult);
            robot.telemetry.addData("", "");
            double headingCorrectionPower = 0.9 * speed;
            if (speed < 2.0) {
                headingCorrectionPower = 0;
            }
            if (distanceToDestinationInches <= INCHES_LEFT_TO_SLOW_DOWN) {
                move(0.01 + speed * Math.sin(distanceToDestinationInches / INCHES_LEFT_TO_SLOW_DOWN * (Math.PI / 2)), 0.0, 0.0, headingCorrectionPower);
            } else {
                move(speed, 0.0, 0.0, headingCorrectionPower);
            }

            if (distanceToDestinationInches < 1) {
                break;
            }
        }

        // Stop movement and switch modes
        move(0, 0, 0, 0);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // Behavior: Overloaded method of distanceDrive. This sets the default of isFieldCentric.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    // Params:
    //      - double distance: The distance to drive the robot in inches.
    //      - double direction: The direction, in degrees, that the robot will drive in. This is
    //                          based on the direction the robot was initialized in.
    //      - double speed: The speed at which the robot will move.
    public void distanceDrive(double distance, double direction, double speed, double holdHeading) throws RuntimeException {
        distanceDrive(distance, direction, speed, holdHeading, DEFAULT_FIELD_CENTRIC);
    }

    public void distanceDrive(double distance, double direction, double speed) throws RuntimeException {
        distanceDrive(distance, direction, speed, 0, DEFAULT_FIELD_CENTRIC);
    }

    // Behavior: Drives the robot to a given position on the field relative to the starting position.
    //           The position is given in inches. It will also turn the robot to the desired heading
    //           given in the position parameter.
    // Params:
    //      - SparkFunOTOS.Pose2D position: The position to drive the robot to.
    //      - double speed: The speed at which the robot will move.
//    public void positionDrive(SparkFunOTOS.Pose2D wantedPosition, double speed) {
//        SparkFunOTOS.Pose2D currentPosition = getPosition();
//        double positionMultiplier = (54 - (0.005 * speed * 2000 / 3)) / 50;
////        double positionMultiplier = 1;
//        double distance = Math.sqrt(Math.pow(currentPosition.x * positionMultiplier - wantedPosition.x, 2) + Math.pow(currentPosition.y * positionMultiplier - wantedPosition.y, 2));
//        while (distance > MIN_DIST_TO_STOP && robot.opModeIsActive()) {
//            wantedHeading = wantedPosition.h;
//            currentPosition = getPosition();
//            double dy = wantedPosition.y - currentPosition.y * positionMultiplier;
//            double dx = wantedPosition.x - currentPosition.x * positionMultiplier;
//            double moveDirection = Math.atan2(dx, dy);
//            double forward = -speed * Math.cos((moveDirection - currentPosition.h));
//            double strafe = speed * Math.sin((moveDirection - currentPosition.h));
//            robot.telemetry.addData("Distance", distance);
//            robot.telemetry.addData("dx", dx);
//            robot.telemetry.addData("dy", dy);
//            robot.telemetry.addData("Move direction", moveDirection);
//            move(forward, 0.0, 0.0, DEFAULT_HEADING_CORRECTION_POWER);
////            distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
//            distance = dy;
//        }
//        // Stop the robot
//        move(0, 0, 0, 0);
////        currentPosition = getPosition();
////        photoSensor.setPosition(new SparkFunOTOS.Pose2D(currentPosition.x * positionMultiplier, currentPosition.y * positionMultiplier, currentPosition.h));
//    }

    // Behavior: Drives the robot continuously based on forward, strafe, and turn power.
    // Params:
    //      - double forwardPower: The power at which the robot will move forward.
    //      - double strafePower: The power at which the robot will strafe.
    //      - double turn: The power at which the robot will turn.
    //      - boolean isFieldCentric: Determines whether the forward direction is based on the
    //                                direction of the robot, or the direction the robot was
    //                                initialized in.
    public void continuousDrive(double forwardPower, double strafePower, double turn, boolean isFieldCentric) {
        double currentHeading = getAngleImuDegrees();
        double forward;
        double strafe;
        if (isFieldCentric) {
            forward = ((forwardPower * Math.cos(currentHeading * (Math.PI / 180))) +
                    (strafePower * Math.sin(currentHeading * (Math.PI / 180))));
            strafe = -((forwardPower * Math.sin(currentHeading * (Math.PI / 180))) -
                    (strafePower * Math.cos(currentHeading * (Math.PI / 180))));
        } else {
            forward = forwardPower;
            strafe = strafePower;
        }

        move(forward, strafe, turn, 0.0);
    }

    // Behavior: Allows driver to tune heading correction using the controller.
    // Params:
    //      - Gamepad gamepad1: The first gamepad from driver control
    //      - Telemetry telemetry: Telemetry object, so telemetry can be sent to Driver Hub
    public void tuneHeadingCorrection(Gamepad gamepad1, Telemetry telemetry) {
        if (gamepad1.a) {
            kTuner = (kTuner == 2) ? 0 : kTuner + 1;
        }
        if (kTuner == 0) {
            if (gamepad1.dpad_down) {
                Kp -= 0.001;
            } else if (gamepad1.dpad_up) {
                Kp += 0.001;
            }
            telemetry.addData("Tuning", "proportional");
        } else if (kTuner == 1) {
            if (gamepad1.dpad_down) {
                Ki -= 0.000001;
            } else if (gamepad1.dpad_up) {
                Ki += 0.000001;
            }
            telemetry.addData("Tuning", "integral");
        } else {
            if (gamepad1.dpad_down) {
                Kd -= 0.00001;
            } else if (gamepad1.dpad_up) {
                Kd += 0.00001;
            }
            telemetry.addData("Tuning", "derivative");
        }
        telemetry.addData("Kp", Kp);
        telemetry.addData("Ki", Ki);
        telemetry.addData("Kd", Kd);
        telemetry.addData("", "");
    }

    // Behavior: Tests the current Kp, Kd, and Ki values by turning the robot, stopping, and seeing
    //           how fast it stops.
    // Returns: A double representing how good the heading correction is. The lower, the better.
    // Parameters:
    //      - double testTime: The amount of time in seconds that the robot should test for.
    public double testHeadingCorrection(double testTime) {
        double startTime = runtime.seconds();
        double totalError = 0;
        double lastTime = runtime.seconds();
        while (startTime + testTime > runtime.seconds()) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastTime;
            if ((int)currentTime % 2 == 0) {
                move(0, 0, 0.5, DEFAULT_HEADING_CORRECTION_POWER);
            } else {
                move(0, 0, 0, DEFAULT_HEADING_CORRECTION_POWER);
                totalError += deltaTime * currentAngularVelocity;
            }
            lastTime = currentTime;
        }
        return totalError;
    }

    // Behavior: Overloaded method of continuousDrive. This sets the default of isFieldCentric.
    // Params:
    //      - double forwardPower: The power at which the robot will move forward.
    //      - double strafePower: The power at which the robot will strafe.
    //      - double turn: The power at which the robot will turn.
    public void continuousDrive(double forwardPower, double strafePower, double turn) {
        continuousDrive(forwardPower, strafePower, turn, DEFAULT_FIELD_CENTRIC);
    }

    // Behavior: Turns the robot to a given angle.
    // Params:
    //      - double degrees: The angle to turn the robot to in degrees.
    //      - double speed: The speed at which the robot should turn.
    public void turnTo(double angle, double speed) {
        wantedHeading = normalize(angle);
        while (Math.abs(normalize(getAngleImuDegrees() - wantedHeading)) > MAX_CORRECTION_ERROR && robot.opModeIsActive()) {
            robot.telemetry.addData("Current Action", "Turning To Angle");
            robot.telemetry.addData("Degrees to destination", wantedHeading - getAngleImuDegrees());
            robot.telemetry.addData("", "");
            wantedHeading = normalize(angle);
            robot.telemetry.addData("Wanted Angle Turn To", angle);
            move(0, 0, 0, speed);
        }
        // Stop the robot
        move(0, 0, 0, 0);
    }

    // Behavior: Normalizes a given degree value to the range (-180, 180]
    // Returns: The normalized degrees as a double.
    // Params:
    //      - double degrees: The degrees to be normalized.
    private static double normalize(double degrees) {
        double normalizedAngle = degrees;
        while (normalizedAngle > 180) normalizedAngle -= 360;
        while (normalizedAngle <= -180) normalizedAngle += 360;
        return normalizedAngle;
    }

    // Behavior: Get the current IMU heading in degrees.
    // Returns: The current robots angle in degrees on the range (-180, 180]
    private double getAngleImuDegrees() {
        return normalize(//photoSensor.getPosition().h);
                gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    }

    // Behavior: Sends various information to the telemetry to be read. The information sent is
    //           the strafe movement of the robot, the forward movement of the robot, the turn
    //           movement of the robot, the wanted heading of the robot, the current heading
    //           of the robot, and the current runtime.
    // Params:
    //      - Telemetry telemetry: The telemetry to send the information to.
    public void sendTelemetry(Telemetry telemetry) {
        telemetry.addData("Forward", currentForward);
        telemetry.addData("Strafe", currentStrafe);
        telemetry.addData("Turn", currentTurn);
        telemetry.addData("Angular Velocity", currentAngularVelocity);
        telemetry.addData("", "");
        telemetry.addData("Current Heading", getAngleImuDegrees());
        telemetry.addData("Wanted Heading", wantedHeading);
        telemetry.addData("", "");
        telemetry.addData("Runtime", runtime.seconds());

        telemetry.update();
    }

    // Behavior: Overloaded method of sendTelemetry() that sends the information to the provided
    //           LinearOpMode's telemetry.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    public void sendTelemetry() throws RuntimeException {
        if (robot == null) {
            throw new RuntimeException(
                    "Tried to run sendTelemetry (no parameters) but LinearOpMode object not given!");
        }

        sendTelemetry(robot.telemetry);
    }

    // Behavior: Stops all movement except heading correction for the given duration of time.
    // Exceptions:
    //      - Throws RuntimeException when a LinearOpMode object was not provided in the constructor.
    // Params:
    //      - double time: The amount of time the robot should stop for in seconds.
    public void sleep(double time) throws RuntimeException {
        if (robot == null) {
            throw new RuntimeException("Tried to run sleep but LinearOpMode object not given!");
        }

        double startTime = runtime.seconds();
        while (robot.opModeIsActive() && (runtime.seconds() - startTime) < time) {
            move(0, 0, 0, 0);
        }

        // Stop the robot
        move(0, 0, 0, 0);
    }
}