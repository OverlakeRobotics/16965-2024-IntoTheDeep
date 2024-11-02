package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="Mecanum Auto Front Left", group="Robot")
public class MecanumAutoFrontLeft extends LinearOpMode {
    public static final double DRIVE_SPEED = 1.0;
    public static final double TURN_SPEED = 0.5;
    private MecanumRobotController robotController;
    @Override
    public void runOpMode() {
        // Initialize
        initialize();

        robotController.distanceDrive(10, 0, DRIVE_SPEED);
        robotController.turnTo(-90, TURN_SPEED);
        robotController.distanceDrive(10, -90, DRIVE_SPEED);
        robotController.turnTo(-180, TURN_SPEED);
        robotController.distanceDrive(10, -180, DRIVE_SPEED);
        robotController.turnTo(90, TURN_SPEED);
        robotController.distanceDrive(10, 90, DRIVE_SPEED);
        robotController.turnTo(0, TURN_SPEED);

    }

    public void initialize() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        IMU gyro = hardwareMap.get(IMU.class, "imu");
        gyro.resetYaw();
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, this);
    }
}
