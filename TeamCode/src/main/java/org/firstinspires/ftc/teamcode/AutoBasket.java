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
@Config
@Autonomous(name="Auto Basket", group="Robot")
public class AutoBasket extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.7 + 0.75;
    public static final double TURN_SPEED = 1.5;
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
    public static double moveSpeed = 1.5;
    public static double distance = 48;
    public static double sleepTime = 100;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        intake.close();
//
//        // place preloaded specimen
//        pivot.setTargetPosition(-1277);
//        viperSlide.setTargetPosition(417);
//        intake.hingeToDegree(99);
//        robotController.distanceDrive(29.0, -0.0, DRIVE_SPEED, 0);
//        robotController.sleep(0.5);
//        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
//        robotController.distanceDrive(3.0, 180.0, DRIVE_SPEED, 0);
//        robotController.sleep(0.1);
//        intake.largeOpen();
//        robotController.sleep(0.1);
//
//        // get out of the way
//        robotController.distanceDrive(10.0, 180.0, DRIVE_SPEED, 0);
//        pivot.setAngleDegrees(100);
//        intake.hingeToDegree(90);
//        robotController.distanceDrive(15.0, -90.0, DRIVE_SPEED, 0);
//        robotController.sleep(20);
        // place the preloaded specimen
        pivot.setTargetPosition(-1277);
        viperSlide.setTargetPosition(417);
        intake.hingeToDegree(99);
        robotController.distanceDrive(29.0, -0.0, DRIVE_SPEED, 0);
        robotController.sleep(0.5);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.distanceDrive(3.0, 180.0, DRIVE_SPEED, 0);
        robotController.sleep(0.1);
        intake.largeOpen();
        robotController.sleep(0.1);

        // pick up first neutral sample
        robotController.distanceDrive(12.0, 180.0, DRIVE_SPEED, 0);
        robotController.distanceDrive(38.5, -90.0, DRIVE_SPEED, 0);

        intake.setWristDegree(0);
        intake.largeOpen();
        pivot.setTargetPosition(207);
        intake.hingeToDegree(157);
        viperSlide.setTargetPosition(242);
        robotController.sleep(1.0);
        robotController.distanceDrive(15.0, -0.0, DRIVE_SPEED - 2.5, 0);
        robotController.sleep(0.7);
        intake.close();
        robotController.sleep(0.4);

        // place first neutral sample in basket
        pivot.setAngleDegrees(100);
        robotController.distanceDrive(23.632, -159.444, DRIVE_SPEED, 0);
        intake.hingeToDegree(90);
        robotController.turnTo(315.0, TURN_SPEED);
        viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
        intake.setWristDegree(0);
        robotController.sleep(3);
        intake.hingeToDegree(187);
        robotController.distanceDrive(7, -180.0, DRIVE_SPEED - 2, -45);
        intake.largeOpen();

        intake.hingeToDegree(90);
        robotController.sleep(0.2);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(2);
        robotController.distanceDrive(7, -45, DRIVE_SPEED, -45);
        robotController.turnTo(0, TURN_SPEED);
        robotController.distanceDrive(5, -90.0, DRIVE_SPEED, 0);

        // pick up second neutral sample
        intake.setWristDegree(0);
        intake.largeOpen();
        intake.hingeToDegree(157);
        viperSlide.setTargetPosition(242);
        pivot.setTargetPosition(207);
        robotController.sleep(2.0);
        robotController.distanceDrive(18.0, -0.0, DRIVE_SPEED - 2.5, 0);
        intake.close();
        robotController.sleep(0.1);

        robotController.sleep(100);
//
//        // place second neutral sample in basket
//        robotController.distanceDrive(15.0, 143.1301, DRIVE_SPEED, 0);
//        pivot.setAngleDegrees(80);
//        viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
//        intake.hingeToDegree(80);
//        intake.setWristDegree(0);
//        robotController.turnTo(135.0, TURN_SPEED);
//        robotController.distanceDrive(12.7279, -135.0, DRIVE_SPEED, 135);
//        intake.largeOpen();
//
//        // pick up third neutral sample
//        intake.hingeToDegree(100);
//        robotController.sleep(0.1);
//        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
//        intake.setWristDegree(-45);
//        robotController.sleep(1.0);
//        robotController.distanceDrive(27.1662, -6.3402, DRIVE_SPEED, 135);
//        robotController.turnTo(45.0, TURN_SPEED);
//        intake.hingeToDegree(0);
//        pivot.setAngleDegrees(-10);
//        robotController.sleep(0.5);
//        intake.close();
//        pivot.setAngleDegrees(0);
//        robotController.sleep(0.1);
//
//        // place third neutral sample in basket
//        robotController.distanceDrive(21.6333, 146.3099, DRIVE_SPEED, 45);
//        pivot.setAngleDegrees(80);
//        viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
//        intake.hingeToDegree(80);
//        intake.setWristDegree(0);
//        robotController.turnTo(135.0, TURN_SPEED);
//        robotController.distanceDrive(12.7279, -135.0, DRIVE_SPEED, 135);
//        intake.largeOpen();
//
//        // park
//        intake.hingeToDegree(100);
//        robotController.sleep(0.1);
//        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
//        robotController.distanceDrive(65.7951, 24.2277, DRIVE_SPEED, 135);
//        pivot.setAngleDegrees(90);
//        robotController.turnTo(270.0, TURN_SPEED);
//        robotController.distanceDrive(9.0, 90.0, DRIVE_SPEED, 270);
    }

    public void initialize() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
//        claw.setDirection(Servo.Direction.REVERSE);
//        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");

        AutonomousNavigator navigator = new AutonomousNavigator(this, telemetry, 0, 0, 0);
        IMU gyro = hardwareMap.get(IMU.class, "imu2");

        SparkFunOTOS photoSensor = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT"),
                650
        );
        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, photoSensor,this);

        telemetry.addData("Status", "Initialized");
    }
}
