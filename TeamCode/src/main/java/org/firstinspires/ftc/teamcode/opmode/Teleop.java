package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");

        waitForStart();

        while (opModeIsActive()) {
            double currentPosition = (pivotMotor.getCurrentPosition() * 0.000189354) * 2 * Math.PI;
            double currentExtend1 = extendOne.getCurrentPosition() * 0.001859773;
            double currentExtend2 = extendTwo.getCurrentPosition() * 0.001859773;

            telemetry.addData("pivot rotations", currentPosition);
            telemetry.addData("extend one", currentExtend1);
            telemetry.addData("extend two", currentExtend2);
            telemetry.update();
        }
    }
}
