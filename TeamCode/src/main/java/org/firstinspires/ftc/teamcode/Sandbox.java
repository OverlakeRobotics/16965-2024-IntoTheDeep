package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name="Sandbox", group="Robot")
public class Sandbox extends OpMode {
    private Intake intake;
    public static int wristDegree = 0;
    public static int hingeDegree = 0;
    @Override
    public void init() {
        intake = new Intake(hardwareMap);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        intake.close();

        // first specimen
        intake.setWristDegree(wristDegree);
        intake.hingeToDegree(hingeDegree);
    }

}
