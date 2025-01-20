package org.firstinspires.ftc.teamcode;

public class PIController {
    private double kP;
    private double kI;

    private double totalError;

    private double setpoint;

    public PIController(double kP, double kI) {
        this(kP, kI, 0);
    }


    public PIController(double kP, double kI, double setpoint) {
        this.kP = kP;
        this.kI = kI;

        this.setpoint = setpoint;
        this.totalError = 0;
    }

    public double calculate(double measure) {
        double error = setpoint - measure;

        if (kI != 0) {
            totalError = clamp(totalError + error, -1 / kI, 1 / kI);
        }

        return kP * error + kI * totalError;
    }

    /**
     * Returns value clamped between low and high boundaries.
     *
     * @param value Value to clamp.
     * @param low The lower boundary to which to clamp value.
     * @param high The higher boundary to which to clamp value.
     * @return The clamped value.
     */
    private double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}
