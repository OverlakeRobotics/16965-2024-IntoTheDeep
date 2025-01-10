package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@Autonomous(name="PID Tuner", group="Robot")
public class PIDTuner extends LinearOpMode {
    public static double DRIVE_SPEED = 0.8; // 2.7 + 0.75;
    public static double TURN_SPEED = 2.0;
    private RobotControllerAuto robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
    public final boolean isFieldCentric = false;
    public static double driveDistance = 72;
    public static double driveDirection = 0.0;
    public static double driveDistance2 = 72;
    public static double driveDirection2 = -90;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        robotController.holdHeading(0,2);
//        robotController.turnTo(90, TURN_SPEED);
//        robotController.driveStraight(driveDistance, driveDirection, DRIVE_SPEED, 0);
//        robotController.driveStraight(driveDistance2, driveDirection2, 0);
        robotController.driveStraight(24, 0.0, DRIVE_SPEED, 0);
        robotController.driveStraight(10.5, 90.0, DRIVE_SPEED, 0);
        robotController.driveStraight(35.0, 180.0, DRIVE_SPEED, 0);
        robotController.driveStraight(35.0, 0.0, DRIVE_SPEED, 0);
        robotController.driveStraight(10.5, 90.0, DRIVE_SPEED, 0);
        robotController.driveStraight(35.0, 180.0, DRIVE_SPEED, 0);
        robotController.holdHeading(0, 100);
//        robotController.sleep(100);
    }

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
//        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
//        claw.setDirection(Servo.Direction.REVERSE);
//        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");

        IMU gyro = hardwareMap.get(IMU.class, "imu2");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT"),
                false
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT"),
                0,
                false
        );
        intake = new Intake(hardwareMap);
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        Position cameraPosition = new Position(DistanceUnit.INCH,
                -7, 5.6, 7, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                -90, -82, 14, 0);
        robotController = new RobotControllerAuto(backLeft, backRight, frontLeft, frontRight, gyro, camera, 0, 0, -180, cameraPosition, cameraOrientation, this);

        telemetry.addData("Status", "Initialized");
    }
}