package org.firstinspires.ftc.teamcode;

public class PIDController {
    private double kP, kI, kD;
    private double setpoint = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    // Constructor
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Set the desired setpoint
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        integralSum = 0;
        lastError = 0;
    }

    // Calculate the output of the PID controller
    public double calculate(double measurement) {
        double error = setpoint - measurement;
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

        // Clamp output to min and max
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

    // Optional: Set output limits
    public void setOutputLimits(double min, double max) {
        outputMin = min;
        outputMax = max;
    }

    // Optional: Reset the controller
    public void reset() {
        integralSum = 0;
        lastError = 0;
    }
}
