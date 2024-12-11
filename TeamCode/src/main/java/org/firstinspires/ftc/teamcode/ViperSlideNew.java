package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ViperSlideNew {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private final ElapsedTime runtime = new ElapsedTime();

    // Constants from original ViperSlide
    public static final double DEFAULT_POWER = 0.65;
    public static final int POSITION_TOLERANCE = 20;
    public static final int MOVE_COUNTS_PER_INCH = 111;
    public static final double BASE_ARM_LENGTH = 15;
    public static final double CHASSIS_TO_PIVOT_LENGTH = 9;
    public static final int MIN_POSITION = 35;
    public static final int MAX_POSITION = 3000;
    public static final int MAX_POSITION_HANG = 1320;
    public static final double RAMP_TIME = 0.2; // Time in seconds to ramp up
    public static final double START_POWER = 0.1; // Starting power for ramping

    // State tracking
    private double targetPower;
    private int targetPosition;
    private double moveStartTime;
    private boolean isRamping = false;
    private ViperState currentState = ViperState.IDLE;

    public enum ViperState {
        IDLE,
        MOVING_TO_POSITION,
        MOVING_WITH_POWER,
        HOLDING
    }

    public ViperSlideNew(DcMotorEx leftMotor, DcMotorEx rightMotor, boolean doResetEncoders) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (doResetEncoders) {
            resetEncoders();
        } else {
            leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        runtime.reset();
    }

    public ViperSlideNew(DcMotorEx leftMotor, DcMotorEx rightMotor) {
        this(leftMotor, rightMotor, true);
    }

    public void startMoveToPosition(int position) {
        if (position < MIN_POSITION || position > MAX_POSITION) {
            return;
        }

        targetPosition = position;
        moveStartTime = runtime.seconds();
        isRamping = true;
        currentState = ViperState.MOVING_TO_POSITION;

        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void startMove(double power) {
        int currentPosition = getCurrentPosition();
        if ((power > 0 && currentPosition >= MAX_POSITION) ||
                (power < 0 && currentPosition <= MIN_POSITION)) {
            hold();
            return;
        }

        currentState = ViperState.MOVING_WITH_POWER;
        targetPower = power;
        moveStartTime = runtime.seconds();
        isRamping = true;

        if (leftMotor.getMode() != DcMotorEx.RunMode.RUN_USING_ENCODER) {
            leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void hold() {
        currentState = ViperState.HOLDING;
        int currentPos = getCurrentPosition();
        leftMotor.setTargetPosition(currentPos);
        rightMotor.setTargetPosition(currentPos);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setPower(DEFAULT_POWER);
        rightMotor.setPower(DEFAULT_POWER);
    }

    private double calculateRampPower(double targetPower) {
        if (!isRamping) return targetPower;

        double elapsed = runtime.seconds() - moveStartTime;
        if (elapsed >= RAMP_TIME) {
            isRamping = false;
            return targetPower;
        }

        double rampProgress = elapsed / RAMP_TIME;
        return START_POWER + (Math.abs(targetPower) - START_POWER) * rampProgress * Math.signum(targetPower);
    }

    public void update() {
        switch (currentState) {
            case MOVING_TO_POSITION:
                double positionPower = calculateRampPower(DEFAULT_POWER);
                leftMotor.setPower(positionPower);
                rightMotor.setPower(positionPower);

                if (isAtTargetPosition()) {
                    hold();
                }
                break;

            case MOVING_WITH_POWER:
                double movePower = calculateRampPower(targetPower);
                leftMotor.setPower(movePower);
                rightMotor.setPower(movePower);
                break;

            case HOLDING:
                // Motors already in position hold mode from hold()
                break;

            case IDLE:
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
        }
    }

    public void resetEncoders() {
        leftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public boolean isAtTargetPosition() {
        int leftError = Math.abs(leftMotor.getCurrentPosition() - targetPosition);
        int rightError = Math.abs(rightMotor.getCurrentPosition() - targetPosition);
        return leftError < POSITION_TOLERANCE && rightError < POSITION_TOLERANCE;
    }

    public double getExtensionBeyondChassis(double angle) {
        double extension = Math.max(Math.abs(Math.cos(Math.toRadians(angle)) *
                (((double) getCurrentPosition() / MOVE_COUNTS_PER_INCH) + BASE_ARM_LENGTH)) -
                CHASSIS_TO_PIVOT_LENGTH, 0);
        if (angle > 90) {
            extension *= -1;
        }
        return extension;
    }

    public boolean isDrifting() {
        int positionDifference = Math.abs(leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition());
        return positionDifference > POSITION_TOLERANCE;
    }

    public int getLeftPosition() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightMotor.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
    }

    public double getCurrentPositionInches() {
        return (double) getCurrentPosition() / MOVE_COUNTS_PER_INCH;
    }

    public boolean isAtBottom() {
        return getCurrentPosition() <= MIN_POSITION;
    }

    public boolean isAtTop() {
        return getCurrentPosition() >= MAX_POSITION;
    }

    public ViperState getCurrentState() {
        return currentState;
    }

    public double getPower() {
        return (leftMotor.getPower() + rightMotor.getPower()) / 2;
    }
}