package org.firstinspires.ftc.teamcode;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class PIController {
    private double kP;
    private double kI;

    private double totalError;
    private Supplier<Double> error;

    private double setpoint;

    public PIController(double kP, double kI) {
        this(kP, kI, 0);
    }


    public PIController(double kP, double kI, double setpoint) {
        this.kP = kP;
        this.kI = kI;

        this.setpoint = setpoint;
        this.totalError = 0;
        this.error = () -> 0.0;
    }

    public double calculate(double measure) {
        error = () -> (setpoint - measure);

        if (kI != 0) {
            totalError = MathUtils.Helpers.clamp(totalError + error.get(), -1 / kI, 1 / kI);
        }

        return kP * error.get() + kI * totalError;
    }

    public double getSetpoint() {
        return this.setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public Supplier<Double> getError() {
        return error;
    }
}
