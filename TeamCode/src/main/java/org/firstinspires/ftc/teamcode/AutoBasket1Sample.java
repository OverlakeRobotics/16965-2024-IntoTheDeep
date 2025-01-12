package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name="Auto Basket 1 Sample", group="Robot")
public class AutoBasket1Sample extends LinearOpMode {
    public static final double DRIVE_SPEED = 0.7;
    public static final double TURN_SPEED = 1.5;
    private RobotControllerAuto robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private final static double VIPER_MAX_POWER = 0.75;
    private final static double VIPER_START_POWER = 0.1;
    private final static double PIVOT_POWER = 0.35;
    public static double moveSpeed = 1.5;
    public static double distance = 48;
    public static double sleepTime = 100;
    public final boolean isFieldCentric = false;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    private ExponentialRamp ramp;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();
        waitForStart();
        intake.close();
        pivot.setAngleDegrees(90);
        intake.hingeToDegree(45);
        rampViper(0.2, ViperSlide.MAX_POSITION);
        viperSlide.waitForFinish();
        pivot.waitForFinish();
        robotController.driveStraight(24, -0.0, DRIVE_SPEED, 0);
        intake.open();
        robotController.holdHeading(0, 0.2);
        robotController.driveStraight(10, -180.0, DRIVE_SPEED, 0);
        rampViper(0.2, ViperSlide.MIN_POSITION);

        robotController.holdHeading(0, 100);
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

    public void rampViper(double rampTime, int viperTarget) {
        ramp = new ExponentialRamp(new Point(runtime.seconds(), Math.min(VIPER_START_POWER, VIPER_MAX_POWER)), new Point(runtime.seconds() + rampTime, VIPER_MAX_POWER));
        while (opModeIsActive() && ramp.scaleX(runtime.seconds()) < VIPER_MAX_POWER) {
            viperSlide.setTargetPosition(viperTarget, ramp.scaleX(runtime.seconds()));
            telemetry.addData("viper power", viperSlide.getPower());
            telemetry.addData("wanted power", ramp.scaleX(runtime.seconds()));
            telemetry.addData("viper position", viperSlide.getCurrentPosition());
            telemetry.update();
        }
    }
}