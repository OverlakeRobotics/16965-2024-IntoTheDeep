package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="4 Specimen Auto Observation", group="Robot")
public class AutoObservation4Specimen extends LinearOpMode {
    public static final double DRIVE_SPEED = 0.85;
    public static final double TURN_SPEED = 0.75;
    private RobotControllerAuto robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize
        initialize();
        waitForStart();
        intake.close();

        // place first specimen
        pivot.setTargetPosition(-1277); //1277
        viperSlide.setTargetPosition(417);
        intake.hingeToDegree(120); //112
        robotController.driveStraight(31.0, 0.0, DRIVE_SPEED - 0.2, 0, true, true);
        robotController.holdHeading(0, 0.05);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.driveStraight(4.0, 180.0, DRIVE_SPEED, 0);
        intake.largeOpen();
        robotController.holdHeading(0, 0.1);

        // push 2 samples into observation zone
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
        // prep for wall pickup
        pivot.setAngleDegrees(212);
        intake.setWristDegree(0);
        intake.largeOpen();
        intake.hingeToDegree(75);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION + 20);
        robotController.driveStraight(39.0, 0.0, DRIVE_SPEED, 0);
        robotController.driveStraight(10.5, 90.0, DRIVE_SPEED, 0);
        robotController.driveStraight(34.0, 180.0, DRIVE_SPEED, 0);
        robotController.driveStraight(11.0, 180.0, DRIVE_SPEED - 0.3, 0);

//        robotController.distanceDrive(47.0, -0.0, DRIVE_SPEED, 0);
//        robotController.distanceDrive(10.5, 90.0, DRIVE_SPEED, 0);
//        robotController.distanceDrive(47.0, 180.0, DRIVE_SPEED, 0);

        // prep for picking up second specimen
//        robotController.distanceDrive(22.7386, 14.0362, DRIVE_SPEED, 0);
//        robotController.turnTo(180.0, TURN_SPEED);

//        if (robotController.getCurrentAprilTagID() == 11) {
//            robotController.aprilTagDrive(-56.4, 57.4, 0, 1.0);
//        } else {
//            robotController.aprilTagDrive(56.4, -57.4, 0, 1.0);
//        }

        // pick up the second specimen
//        intake.setWristDegree(0);
//        intake.largeOpen();
//        pivot.setTargetPosition(207);
//        intake.hingeToDegree(157);
//        viperSlide.setTargetPosition(242);
//        robotController.sleep(3.0);
//        robotController.distanceDrive(17.0, 180.0, DRIVE_SPEED - 2, 180);
//        intake.close();
//        robotController.sleep(0.5);

//        robotController.driveStraight(12, -45.0, DRIVE_SPEED, 0);
//        robotController.driveStraight(10, -180, DRIVE_SPEED - 0.3, 0.0);
        robotController.holdHeading(0,0.2);
        intake.close();
        robotController.holdHeading(0,0.2);


        // drive to submersible for second specimen placing
//        intake.hingeToDegree(99);
//        intake.setWristDegree(-81);
//        pivot.setTargetPosition(-1277);
//        robotController.distanceDrive(55.4105, -74.7449, DRIVE_SPEED, 180);
//        viperSlide.setTargetPosition(417);
//        robotController.turnTo(0.0, TURN_SPEED);
//        robotController.distanceDrive(10.0, -0.0, DRIVE_SPEED, 0);
        pivot.setAngleDegrees(106);
        intake.hingeToDegree(-30);
        robotController.holdHeading(0,0.2);
        intake.tinyOpen();
        viperSlide.setTargetPosition(1120);
        robotController.driveStraight(52, -70.6992, DRIVE_SPEED, 0); // 48.2945
        intake.close();
        robotController.driveStraight(14.0, -0.0, DRIVE_SPEED - 0.2, 0, true, true);
        viperSlide.setTargetPosition(1695);
        viperSlide.waitForFinish();
        intake.open();
        robotController.holdHeading(0,0.2);
        robotController.driveStraight(3.0, -180, DRIVE_SPEED, 0, false);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION + 20);
        robotController.holdHeading(0,0.2);
        pivot.setAngleDegrees(212);
        intake.setWristDegree(0);
        intake.largeOpen();
        intake.hingeToDegree(75);
        robotController.driveStraight(35.0, 100.0, DRIVE_SPEED, 0);
        if (robotController.getCurrentAprilTagID() == 11) {
            robotController.aprilTagDrive(-56.4, 57.4, 0, DRIVE_SPEED, false);
        } else {
            robotController.aprilTagDrive(56.4, -57.4, 0, DRIVE_SPEED, true);
        }

        robotController.driveStraight(13, -180, DRIVE_SPEED - 0.3, 0.0);
        robotController.holdHeading(0,0.2);
        intake.close();
        robotController.holdHeading(0,0.1);

        pivot.setAngleDegrees(106);
        intake.hingeToDegree(-30);
        robotController.holdHeading(0,0.2);
        intake.tinyOpen();
        viperSlide.setTargetPosition(1120);

        robotController.driveStraight(45.6946, -71.8014, DRIVE_SPEED, 0);
        intake.close();
        robotController.driveStraight(15.0, -0.0, DRIVE_SPEED - 0.2, 0, true, true);

        viperSlide.setTargetPosition(1695);
        viperSlide.waitForFinish();
        intake.open();

        robotController.holdHeading(0,0.2);
        robotController.driveStraight(3.0, -180, DRIVE_SPEED, 0, false);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION + 20);
        robotController.holdHeading(0,0.2);
        pivot.setAngleDegrees(212);
        intake.setWristDegree(0);
        intake.largeOpen();
        intake.hingeToDegree(75);
        robotController.driveStraight(41.0, 110.0, DRIVE_SPEED, 0);
//        if (robotController.getCurrentAprilTagID() == 11) {
//            robotController.aprilTagDrive(-56.4, 57.4, 0, DRIVE_SPEED);
//        } else {
//            robotController.aprilTagDrive(56.4, -57.4, 0, DRIVE_SPEED);
//        }

        robotController.driveStraight(11, -180, DRIVE_SPEED - 0.3, 0.0);
        robotController.holdHeading(0,0.2);
        intake.close();
        robotController.holdHeading(0,0.1);

        pivot.setAngleDegrees(106);
        intake.hingeToDegree(-30);
        robotController.holdHeading(0,0.2);
        intake.tinyOpen();
        viperSlide.setTargetPosition(1120);

        robotController.driveStraight(43.6946, -71.8014, DRIVE_SPEED, 0);
        intake.close();
        robotController.driveStraight(15.0, -0.0, DRIVE_SPEED - 0.2, 0, true, true);

        viperSlide.setTargetPosition(1695);
        viperSlide.waitForFinish();
        intake.open();

        robotController.holdHeading(0,0.2);
        intake.hingeToDegree(50);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION + 20);
        robotController.driveStraight(10, -180, DRIVE_SPEED, 0, false);
        robotController.driveStraight(50, 110, 1.0, 0);

        robotController.holdHeading(0,100);

        // place second specimen
//        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
//        robotController.distanceDrive(3.0, 180.0, DRIVE_SPEED, 0);
//        robotController.sleep(0.1);
//        intake.largeOpen();
//        robotController.sleep(0.1);
//
//        // park
////        pivot.setAngleDegrees(100);
////        intake.hingeToDegree(90);
//        robotController.distanceDrive(55.2495, 100, DRIVE_SPEED, 0);
//        robotController.turnTo(90, TURN_SPEED);
//        robotController.distanceDrive(10, 180, DRIVE_SPEED, 90);
//        intake.setWristDegree(0);
//        intake.largeOpen();
//        pivot.setTargetPosition(207);
//        intake.hingeToDegree(157);
//        viperSlide.setTargetPosition(242);
//        robotController.distanceDrive(5, 90, DRIVE_SPEED, 90);
//        robotController.sleep(0.1);
//        intake.close();
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