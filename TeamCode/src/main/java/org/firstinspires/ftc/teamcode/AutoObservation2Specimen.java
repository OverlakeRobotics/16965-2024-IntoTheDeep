package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="2 Specimen Auto Observation", group="Robot")
public class AutoObservation2Specimen extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.7 + 0.75;
    public static final double TURN_SPEED = 1.5;
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_POWER = 0.75;
    private final static double PIVOT_POWER = 0.35;
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

        // place first specimen
        pivot.setTargetPosition(-1277);
        viperSlide.setTargetPosition(417);
        intake.hingeToDegree(99);
        robotController.distanceDrive(29.0, -0.0, DRIVE_SPEED);
        robotController.sleep(0.5);
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.distanceDrive(3.0, 180.0, DRIVE_SPEED);
        robotController.sleep(0.1);
        intake.largeOpen();
        robotController.sleep(0.1);

        // push 2 samples into observation zone
        robotController.distanceDrive(9.0, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(30.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(33.0, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(10.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(47.0, 180.0, DRIVE_SPEED);
        robotController.distanceDrive(47.0, -0.0, DRIVE_SPEED);
        robotController.distanceDrive(12.0, 90.0, DRIVE_SPEED);
        robotController.distanceDrive(47.0, 180.0, DRIVE_SPEED);

        // prep for picking up second specimen
        robotController.distanceDrive(22.7386, 14.0362, DRIVE_SPEED);
        robotController.turnTo(180.0, TURN_SPEED);

        // pick up the second specimen
        intake.setWristDegree(0);
        intake.largeOpen();
        pivot.setTargetPosition(207);
        intake.hingeToDegree(157);
        viperSlide.setTargetPosition(292);
        robotController.sleep(0.1);
        robotController.distanceDrive(21.0, 180.0, DRIVE_SPEED);
        intake.close();
        robotController.sleep(0.1);

        // drive to submersible for second specimen placing
        robotController.distanceDrive(63.4105, -74.7449, DRIVE_SPEED);
        pivot.setTargetPosition(-1277);
        viperSlide.setTargetPosition(417);
        intake.hingeToDegree(99);
        intake.setWristDegree(-81);
        robotController.turnTo(0.0, TURN_SPEED);
        robotController.distanceDrive(8.0, -0.0, DRIVE_SPEED);

        // place second specimen
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.distanceDrive(3.0, 180.0, DRIVE_SPEED);
        robotController.sleep(0.1);
        intake.largeOpen();
        robotController.sleep(0.1);

        // park
        robotController.distanceDrive(55.2495, 124.5085, DRIVE_SPEED);
        robotController.sleep(10.0);
    }

    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");

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
