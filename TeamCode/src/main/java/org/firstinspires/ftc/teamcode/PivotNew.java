package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PivotNew {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private static final double DEFAULT_POWER = 0.6;
    private static final int POSITION_TOLERANCE = 10;
    public static final int MIN_POSITION = -1750;
    public static final int MAX_POSITION = 200;
    public static final double TICKS_PER_DEGREE = 945 / 90d;
    private int targetPosition = 0;
    private final int offset;
    private PivotState currentState = PivotState.STOPPED;

    public enum PivotState {
        MOVING,
        MOVING_TO_POSITION,
        HOLDING,
        STOPPED
    }

    public PivotNew(DcMotorEx leftMotor, DcMotorEx rightMotor, int offset, boolean doResetEncoders) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.offset = offset;

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (doResetEncoders) {
            resetEncoders();
        } else {
            leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public PivotNew(DcMotorEx leftMotor, DcMotorEx rightMotor, int offset) {
        this(leftMotor, rightMotor, offset, true);
    }

    public PivotNew(DcMotorEx leftMotor, DcMotorEx rightMotor) {
        this(leftMotor, rightMotor, 0);
    }

    public void startMove(double power) {
        currentState = PivotState.MOVING;

        if (leftMotor.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void startMoveToPosition(int position) {
        targetPosition = position + offset;
        currentState = PivotState.MOVING_TO_POSITION;

        leftMotor.setTargetPosition(targetPosition);
        rightMotor.setTargetPosition(targetPosition);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(DEFAULT_POWER);
        rightMotor.setPower(DEFAULT_POWER);
    }

    public void startSetAngleDegrees(double degrees) {
        startMoveToPosition((int)(-((10.5 * degrees) + 430)));
    }

    public void hold() {
        currentState = PivotState.HOLDING;
        int currentPos = getCurrentPosition();
        leftMotor.setTargetPosition(currentPos + offset);
        rightMotor.setTargetPosition(currentPos + offset);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(DEFAULT_POWER);
        rightMotor.setPower(DEFAULT_POWER);
    }

    public void stop() {
        currentState = PivotState.STOPPED;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void update() {
        switch (currentState) {
            case MOVING_TO_POSITION:
                if (isAtTargetPosition()) {
                    hold();
                }
                break;

            case MOVING:
                // Power already set in startMove
                break;

            case HOLDING:
                // Already in position hold mode
                break;

            case STOPPED:
                // Motors already stopped
                break;
        }
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isAtTargetPosition() {
        int leftError = Math.abs(leftMotor.getCurrentPosition() - targetPosition);
        int rightError = Math.abs(rightMotor.getCurrentPosition() - targetPosition);
        return leftError < POSITION_TOLERANCE && rightError < POSITION_TOLERANCE;
    }

    public boolean isDrifting() {
        int positionDifference = Math.abs(leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition());
        return positionDifference > POSITION_TOLERANCE;
    }

    public double getAngleDegrees() {
        return -(getCurrentPosition() + 430) / 10.5;
    }

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition() - offset;
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition() - offset;
    }

    public int getCurrentPosition() {
        return ((leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2) - offset;
    }

    public PivotState getCurrentState() {
        return currentState;
    }
}