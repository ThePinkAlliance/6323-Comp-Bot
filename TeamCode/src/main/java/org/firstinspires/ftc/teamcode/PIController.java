package org.firstinspires.ftc.teamcode;

public class PIController {
    private double kP;
    private double kI;

    private double totalError;
    private double error;

    private double setpoint;

    public PIController(double kP, double kI) {
        this(kP, kI, 0);
    }


    public PIController(double kP, double kI, double setpoint) {
        this.kP = kP;
        this.kI = kI;

        this.setpoint = setpoint;
        this.totalError = 0;
        this.error = 0;
    }

    public double calculate(double measure) {
        error = setpoint - measure;

        if (kI != 0) {
            totalError = MathUtils.Helpers.clamp(totalError + error, -1 / kI, 1 / kI);
        }

        return kP * error + kI * totalError;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError() {
        return error;
    }
}
