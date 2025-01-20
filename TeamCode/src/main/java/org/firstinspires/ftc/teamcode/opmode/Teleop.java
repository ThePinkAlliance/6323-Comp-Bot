package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.PIController;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Servo port 0-1 are the vex servos
 * Servo port 2 is the gobuilda spin servo
 */
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PIController controller = new PIController(0.5, 0);
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");
        Drive drive = new Drive(hardwareMap);

        extendOne.setDirection(DcMotorSimple.Direction.REVERSE);
        extendTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            double currentPosition = (pivotMotor.getCurrentPosition() * 0.000189354);
            double currentExtend1 = extendOne.getCurrentPosition() * 0.001859773;
            double extendPower = gamepad1.right_stick_y;

//            double controlEffort = controller.calculate(currentPosition) + ();

            extendTwo.setPower(extendPower);
            extendOne.setPower(extendPower);

            drive.setDriveCommand(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

//            pivotMotor.setPower();

            telemetry.addData("pivot rotations", currentPosition);
            telemetry.addData("extend", currentExtend1);
            telemetry.update();
        }
    }
}
