package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
@Autonomous(name="PID Tuner", group="Robot")
public class PIDTuner extends LinearOpMode {
    public static double DRIVE_SPEED = 0.85; // 2.7 + 0.75;
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
        initialize();
        waitForStart();
        intake.close();

        // place first specimen
        pivot.setTargetPosition(pivot.getCurrentPosition()); //1277

        robotController.driveStraight(31.0, 0.0, DRIVE_SPEED - 0.2, 0, true, true);
        robotController.driveStraight(4.0, 180.0, DRIVE_SPEED, 0);
        robotController.driveStraight(5.0, 180.0, DRIVE_SPEED, 0);
        robotController.driveStraight(24.5, 90.0, DRIVE_SPEED, 0);
//        if (robotController.getCurrentAprilTagID() == 11) {
//            robotController.aprilTagDrive(-43.4, 51.8, 0, DRIVE_SPEED);
//        } else {
//            robotController.aprilTagDrive(43.4, -51.8, 0, DRIVE_SPEED);
//        }
        robotController.driveStraight(28, 0.0, DRIVE_SPEED, 0);
        robotController.driveStraight(10.5, 90.0, DRIVE_SPEED, 0);
        robotController.driveStraight(39.0, 180.0, DRIVE_SPEED, 0);
        robotController.driveStraight(39.0, 0.0, DRIVE_SPEED, 0);
        robotController.driveStraight(10.5, 90.0, DRIVE_SPEED, 0);
        robotController.driveStraight(34.0, 180.0, DRIVE_SPEED, 0);
        robotController.driveStraight(11.0, 180.0, DRIVE_SPEED - 0.3, 0);
//        robotController.driveStraight(8, -45.0, DRIVE_SPEED, 0);
//        robotController.driveStraight(10, -180, DRIVE_SPEED - 0.3, 0.0);
        robotController.driveStraight(52, -70.6992, DRIVE_SPEED, 0);
        robotController.driveStraight(14.0, -0.0, DRIVE_SPEED - 0.2, 0, true, true);
        robotController.driveStraight(3.0, -180, DRIVE_SPEED, 0);
        robotController.driveStraight(35.0, 100.0, DRIVE_SPEED, 0, false);
        if (robotController.getCurrentAprilTagID() == 11) {
            robotController.aprilTagDrive(-56.4, 57.4, 0, DRIVE_SPEED, false);
        } else {
            robotController.aprilTagDrive(56.4, -57.4, 0, DRIVE_SPEED, true);
        }
        robotController.driveStraight(12, -180, DRIVE_SPEED - 0.3, 0.0);
        robotController.driveStraight(45.6946, -71.8014, DRIVE_SPEED, 0);
        robotController.driveStraight(15.0, -0.0, DRIVE_SPEED - 0.2, 0, true, true);
        robotController.driveStraight(3.0, -180, DRIVE_SPEED, 0);
        robotController.driveStraight(41.0, 110.0, DRIVE_SPEED, 0);
//        if (robotController.getCurrentAprilTagID() == 11) {
//            robotController.aprilTagDrive(-56.4, 57.4, 0, DRIVE_SPEED);
//        } else {
//            robotController.aprilTagDrive(56.4, -57.4, 0, DRIVE_SPEED);
//        }
        robotController.driveStraight(11, -180, DRIVE_SPEED - 0.3, 0.0);
        robotController.driveStraight(43.6946, -71.8014, DRIVE_SPEED, 0);
        robotController.driveStraight(15.0, -0.0, DRIVE_SPEED - 0.2, 0, true, true);

        robotController.driveStraight(10, -180, DRIVE_SPEED, 0, false);
        robotController.driveStraight(50, 110, 1.0, 0);

        robotController.holdHeading(0,100);
    }

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");

        IMU gyro = hardwareMap.get(IMU.class, "imu2");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT"),
                520
        );
        intake = new Intake(hardwareMap);
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "LIMITSWITCH");
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");
        Position cameraPosition = new Position(DistanceUnit.INCH,
                -7, 5.6, 7, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                -90, -82, 14, 0);
        robotController = new RobotControllerAuto(backLeft, backRight, frontLeft, frontRight, gyro, limitSwitch, camera, 0, 0, -180, cameraPosition, cameraOrientation, this);

        telemetry.addData("Status", "Initialized");
    }
}