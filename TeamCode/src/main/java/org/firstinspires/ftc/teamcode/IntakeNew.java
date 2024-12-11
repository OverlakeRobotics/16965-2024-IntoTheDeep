package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakeNew {
    private final Servo clawLeft;
    private final Servo clawRight;
    private final Servo hingeLeft;
    private final Servo hingeRight;
    private final Servo wrist;
    private boolean isOpen;

    // Constants from original Intake
    public static double LARGE_OPEN_POSITION = 0;
    public static double NORMAL_OPEN_POSITION = 0.1;
    public static double CLOSED_POSITION = 0.21;
    public static final double HINGE_MAX_POSITION = 1.0;
    public static final double WRIST_MIN_POSITION = 0;
    public static final double WRIST_MAX_POSITION = 1.0;
    public static final double WRIST_NEUTRAL_POSITION = 0.3;

    public IntakeNew(HardwareMap hardwareMap) {
        clawLeft = hardwareMap.get(Servo.class, "CLAWLEFT");
        clawRight = hardwareMap.get(Servo.class, "CLAWRIGHT");
        hingeLeft = hardwareMap.get(Servo.class, "HINGELEFT");
        hingeRight = hardwareMap.get(Servo.class, "HINGERIGHT");
        wrist = hardwareMap.get(Servo.class, "WRIST");

        clawLeft.setDirection(Servo.Direction.REVERSE);
        clawRight.setDirection(Servo.Direction.FORWARD);
        hingeLeft.setDirection(Servo.Direction.FORWARD);
        hingeRight.setDirection(Servo.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.FORWARD);
    }

    public void open() {
        clawLeft.setPosition(NORMAL_OPEN_POSITION);
        clawRight.setPosition(NORMAL_OPEN_POSITION);
        isOpen = true;
    }

    public void largeOpen() {
        clawLeft.setPosition(LARGE_OPEN_POSITION);
        clawRight.setPosition(LARGE_OPEN_POSITION);
        isOpen = true;
    }

    public void close() {
        clawLeft.setPosition(CLOSED_POSITION);
        clawRight.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    public void hingeTo(double position) {
        position = Math.min(Math.max(0, position), HINGE_MAX_POSITION);
        hingeLeft.setPosition(position);
        hingeRight.setPosition(position);
    }

    public void hingeToDegree(int degree) {
        double position = HINGE_MAX_POSITION - (HINGE_MAX_POSITION / 187.5) * (degree + 30);
        position = Math.min(Math.max(0, position), HINGE_MAX_POSITION);
        hingeLeft.setPosition(position);
        hingeRight.setPosition(position);
    }

    public void setWristPosition(double position) {
        position = Math.min(Math.max(WRIST_MIN_POSITION, position), WRIST_MAX_POSITION);
        wrist.setPosition(position);
    }

    public void setWristDegree(int degree) {
        double position = WRIST_NEUTRAL_POSITION + (WRIST_MAX_POSITION / 270) * degree;
        position = Math.min(Math.max(WRIST_MIN_POSITION, position), WRIST_MAX_POSITION);
        wrist.setPosition(position);
    }

    public boolean isOpen() {
        return isOpen;
    }

    public boolean isClosed() {
        return !isOpen;
    }

    public double getLeftClawPosition() {
        return clawLeft.getPosition();
    }

    public double getRightClawPosition() {
        return clawRight.getPosition();
    }

    public double getHingePosition() {
        return hingeLeft.getPosition();
    }

    public int getHingePositionDegrees() {
        return (int) (187.5 - (hingeLeft.getPosition() / HINGE_MAX_POSITION * 187.5)) - 30;
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }

    public int getWristPositionDegrees() {
        return (int) ((wrist.getPosition() - WRIST_NEUTRAL_POSITION) / WRIST_MAX_POSITION * 270);
    }

    public void setLeftClawPosition(double position) {
        clawLeft.setPosition(position);
    }

    public void setRightClawPosition(double position) {
        clawRight.setPosition(position);
    }

    // No update() method needed since servos are inherently non-blocking
    // They automatically move to their set positions
}