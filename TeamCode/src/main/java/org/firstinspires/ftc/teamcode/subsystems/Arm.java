package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private HardwareMap map;
    private DcMotorEx pivotMotor;
    private DcMotorEx extendMotorOne, extendMotorTwo;

    public Arm(HardwareMap map) {
        this.map = map;

        this.pivotMotor = map.get(DcMotorEx.class, "pivotMotor");
        this.extendMotorOne = map.get(DcMotorEx.class, "extendOne");
        this.extendMotorTwo = map.get(DcMotorEx.class, "extendTwo");

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotorTwo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Action extendTo(double pos) {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    // Init
                }

                return true;
            }
        };
    }

    public Action pivotTo(double pos) {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    // Init
                    double targetPosition = 5281.1 * (pos/(2*Math.PI));
                }

                return true;
            }
        };
    }
}
