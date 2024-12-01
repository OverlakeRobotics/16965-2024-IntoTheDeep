package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="3 Specimen Auto Observation", group="Robot")
public class AutoObservation3Specimen extends LinearOpMode {
    public static final double DRIVE_SPEED = 2.7;
    public static final double TURN_SPEED = 0.8;
    private RobotController robotController;
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

        // first specimen
        pivot.setTargetPosition(-945);
        intake.hingeToDegree(0);
        viperSlide.setTargetPosition(1230);
        robotController.distanceDrive(26.0, -0.0, DRIVE_SPEED + 0.75);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(500);
        robotController.sleep(0.75);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);

        // push two samples into observation zone
        robotController.distanceDrive(6.0, 180.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(27.0, 90.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(30.0, -0.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(12.0, 90.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(42.0, 180.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(42.0, -0.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(12.0, 90.0, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(42.0, 180.0, DRIVE_SPEED + 0.75);

        // prep for grabbing second specimen
        intake.hingeToDegree(80);
        pivot.setTargetPosition(-1890);
        robotController.distanceDrive(13.4164, -63.4349, DRIVE_SPEED + 0.75);
        robotController.sleep(1.5);

        // grab the second specimen
        robotController.distanceDrive(12.0, 180.0, DRIVE_SPEED);
        intake.close();
        robotController.sleep(0.1);
        intake.hingeToDegree(0);
        pivot.setTargetPosition(-945);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(1320);

        // place the second specimen
        robotController.distanceDrive(44.5982, -70.3462, DRIVE_SPEED + 0.75);
        robotController.distanceDrive(11.0, -0.0, DRIVE_SPEED + 0.75);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(500);
        robotController.sleep(0.75);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(0.5);

        // prep to grab third specimen
        intake.hingeToDegree(80);
        pivot.setTargetPosition(-1890);
        robotController.distanceDrive(43.6807, 105.9454, DRIVE_SPEED + 0.75);
        robotController.sleep(1.5);

        // grab the third specimen
        robotController.distanceDrive(12.0, 180.0, DRIVE_SPEED);
        intake.close();
        robotController.sleep(0.1);
        intake.hingeToDegree(0);
        pivot.setTargetPosition(-945);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(1320);

        // place the third specimen
        robotController.distanceDrive(50.2892, -72.646, DRIVE_SPEED);
        robotController.distanceDrive(11.0, -0.0, DRIVE_SPEED);
        robotController.sleep(0.1);
        viperSlide.setTargetPosition(500);
        robotController.sleep(0.75);
        intake.largeOpen();
        viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        robotController.sleep(10);
    }

    public void initialize() {
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
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT")
        );
        intake = new Intake(hardwareMap);
        robotController = new RobotController(backLeft, backRight, frontLeft, frontRight, gyro, photoSensor,this);

        telemetry.addData("Status", "Initialized");
    }
}
