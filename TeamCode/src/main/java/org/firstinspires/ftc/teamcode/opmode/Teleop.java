package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.PIController;
import org.firstinspires.ftc.teamcode.lib.ActionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Servo port 0-1 are the vex servos
 * Servo port 2 is the gobuilda spin servo
 *
 * currently extend pid controller is within 0.06 error
 */
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    // On arm extend subtracted 0.5 from max extend length and it reached max
    private final double MAX_EXTEND_ROTATIONS = 6.539 - 0.5;
    private final double PIVOT_ZERO_OFFSET_RAD = 0.6258;

    // Amount of encoder ticks per rotation
    private final double PIVOT_GEAR_RATIO = (1/5281.1);
    private PIController pivotController = new PIController(0.95, 0.002);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");

        ActionExecutor executor = new ActionExecutor();
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Drive drive = new Drive(hardwareMap);

        extendOne.setDirection(DcMotorSimple.Direction.FORWARD);
        extendTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double motorRotationsRad = (pivotMotor.getCurrentPosition() * PIVOT_GEAR_RATIO * 2 * Math.PI) - PIVOT_ZERO_OFFSET_RAD;
            double pivotEffort = pivotController.calculate(motorRotationsRad);
            pivotEffort = MathUtils.Helpers.clamp(pivotEffort, -1, 1);

            if (gamepad1.a) {
                executor.run(new SequentialAction(rotateArm(1.67), extendArm((MAX_EXTEND_ROTATIONS) / 2, extendOne, extendTwo)));
            }

            if (gamepad1.y) {
                executor.run(new SequentialAction(rotateArm(1.67), extendArm((MAX_EXTEND_ROTATIONS), extendOne, extendTwo)));
            }

            if (gamepad1.b) {
                // -0.15 was estimated to close to specified encoder counts when at rest but I didn't get to test before leaving.
                executor.run(new SequentialAction(extendArm(0, extendOne, extendTwo), rotateArm(-0.15)));
            }

            telemetry.addData("control effort", pivotEffort);
            telemetry.addData("pivot rotations", motorRotationsRad);
            telemetry.addData("pivot error", pivotController.getError());

            pivotMotor.setPower(pivotEffort);
            drive.setDriveCommand(gamepad1.left_stick_x * -1, gamepad1.left_stick_y, gamepad1.right_stick_x * -1);

            telemetry.addData("back right vel", drive.getRightBackVel());
            telemetry.addData("back left vel", drive.getLeftBackVel());
            telemetry.addData("front left vel", drive.getLeftFrontVel());
            telemetry.addData("front right vel", drive.getRightFrontVel());

            telemetry.update();
            executor.execute();
        }
    }

    /**
     * @param distance in rotations
     * @param extendOne left extend motor
     * @param extendTwo right extend motor (without encoder cable)
     * @return
     */
    public Action extendArm(double distance, DcMotorEx extendOne, DcMotorEx extendTwo) {
        return new Action() {
            PIController extendController = new PIController(0.5, 0);

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                extendController.setSetpoint(distance);

                double currentExtend1 = extendOne.getCurrentPosition() * -0.001859773;
                double extendEffort = MathUtils.Helpers.clamp(extendController.calculate(currentExtend1), -0.5, 0.5);

                if (Math.abs(extendController.getError()) <= 0.02) {
                    extendTwo.setPower(0);
                    extendOne.setPower(0);
                    return true;
                }

                extendTwo.setPower(extendEffort);
                extendOne.setPower(extendEffort);

                telemetry.addData("extend", currentExtend1);
                telemetry.addData("extend effort", extendEffort);
                telemetry.addData("extend error", extendController.getError());

                return false;
            }
        };
    }

    /**
     * This action will rotate the arm to a setpoint specified in radians.
     * The idea of this method is in a chain of actions like (extend to x -> rotate to y)
     * to pause the chain so actions aren't running concurrently.
     *
     * Also, the actual control logic isn't in this method because the action won't run constantly,
     * and we want the arm to respond to disturbance throughout the entire match
     *
     * @param angle target in radians
     *
     * @return
     */
    public Action rotateArm(double angle) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                pivotController.setSetpoint(angle);

                if (Math.abs(pivotController.getError()) <= 0.02) {
                    return true;
                }

                return false;
            }
        };
    }
}
