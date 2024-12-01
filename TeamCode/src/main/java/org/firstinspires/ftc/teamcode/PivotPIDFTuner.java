package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="Pivot PIDF Tuner", group="TeleOp")
public class PivotPIDFTuner extends OpMode {
    public static int target;
    public static double kp;
    public static double ki;
    public static double kd;
    public static double kf;
    private PIDController controller;
    private DcMotorEx left;
    private DcMotorEx right;
    @Override
    public void init() {
        controller = new PIDController(kp, ki, kd);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        left = hardwareMap.get(DcMotorEx.class, "PIVOTLEFT");
        right = hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT");
        left.setDirection(DcMotorEx.Direction.REVERSE);
        right.setDirection(DcMotorEx.Direction.FORWARD);
        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        controller.setPID(kp, ki, kd);
        int currPos = (left.getCurrentPosition() + right.getCurrentPosition()) / 2;
        double pid = controller.calculate(currPos, target);
        double feedForward = Math.cos(Math.toRadians(target / Pivot.TICKS_PER_DEGREE)) * kf;

        double power = pid + feedForward;

        left.setPower(power);
        right.setPower(power);

        telemetry.addData("pos", currPos);
        telemetry.addData("target", target);
    }
}
