package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.PIController;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Servo port 0-1 are the vex servos
 * Servo port 2 is the gobuilda spin servo
 *
 * currently extend pid controller is within 0.06 error
 */
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    private final double MAX_EXTEND_ROTATIONS = 6.539;

    @Override
    public void runOpMode() throws InterruptedException {
        PIController pivotController = new PIController(0.5, 0);
        PIController extendController = new PIController(0.5, 0, 1);
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");
        Drive drive = new Drive(hardwareMap);

        extendOne.setDirection(DcMotorSimple.Direction.REVERSE);
        extendTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double currentPosition = (pivotMotor.getCurrentPosition() * ((1/5281.1) * 2 * Math.PI));
            double currentExtend1 = extendOne.getCurrentPosition() * -0.001859773;
            double extendPower = gamepad1.right_stick_y;

            double controlEffort = pivotController.calculate(currentPosition) + Math.cos(currentPosition) * 0.05;
            controlEffort = MathUtils.Helpers.clamp(controlEffort, -0.5, 0.5);

            double extendEffort = MathUtils.Helpers.clamp(extendController.calculate(currentExtend1), -0.5, 0.5);

            extendTwo.setPower(extendEffort);
            extendOne.setPower(extendEffort);

//            drive.setDriveCommand(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("control effort", controlEffort);
            telemetry.addData("pivot rotations", currentPosition);
            telemetry.addData("extend", currentExtend1);
            telemetry.addData("extend effort", extendEffort);
            telemetry.addData("extend error", extendController.getError());
            telemetry.update();
        }
    }
}
