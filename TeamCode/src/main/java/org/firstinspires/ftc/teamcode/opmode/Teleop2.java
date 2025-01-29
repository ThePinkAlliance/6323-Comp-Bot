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
public class Teleop2 extends LinearOpMode {
    // On arm extend subtracted 0.5 from max extend length and it reached max
    private final double MAX_EXTEND_ROTATIONS = 6.539 - 0.5;
    private final double PIVOT_ZERO_OFFSET_RAD = 0.6258;

    // Amount of encoder ticks per rotation
    private final double PIVOT_GEAR_RATIO = (1/5281.1);
    private PIController pivotController = new PIController(0.95, 0.002);
    private PIController extendController = new PIController(0.5, 0.0005);


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
            pivotController.setSetpoint(pivotController.getSetpoint() + gamepad2.left_stick_y * (Math.PI / 16));
            extendController.setSetpoint(extendController.getSetpoint() + gamepad2.right_stick_y * (MAX_EXTEND_ROTATIONS / 16));


            executePivotControl(pivotMotor);
            executeExtendControl(extendOne, extendTwo);

            telemetry.addData("back right vel", drive.getRightBackVel());
            telemetry.addData("back left vel", drive.getLeftBackVel());
            telemetry.addData("front left vel", drive.getLeftFrontVel());
            telemetry.addData("front right vel", drive.getRightFrontVel());

            drive.setDriveCommand(gamepad1.left_stick_x * -1, gamepad1.left_stick_y, gamepad1.right_stick_x * -1);

            telemetry.update();
            executor.execute();
        }
    }

    private void executePivotControl(DcMotorEx pivotMotor) {
        double motorRotationsRad = (pivotMotor.getCurrentPosition() * PIVOT_GEAR_RATIO * 2 * Math.PI);
        double pivotEffort = pivotController.calculate(motorRotationsRad);
        pivotEffort = MathUtils.Helpers.clamp(pivotEffort, -1, 1);

        pivotMotor.setPower(pivotEffort);

        telemetry.addData("control effort", pivotEffort);
        telemetry.addData("pivot rotations", motorRotationsRad);
        telemetry.addData("pivot error", pivotController.getError().get());
    }

    private void executeExtendControl(DcMotorEx extendOne, DcMotorEx extendTwo) {
        double currentExtend1 = extendOne.getCurrentPosition() * -0.001859773;
        double extendEffort = MathUtils.Helpers.clamp(extendController.calculate(currentExtend1), -1, 1);

        extendTwo.setPower(extendEffort);
        extendOne.setPower(extendEffort);

        telemetry.addData("extend", currentExtend1);
        telemetry.addData("extend effort", extendEffort);
        telemetry.addData("extend error", extendController.getError().get());
    }

    /**
     * @param distance in rotations
     * @param extendOne left extend motor
     * @param extendTwo right extend motor (without encoder cable)
     * @return
     */
    public Action extendArm(double distance, DcMotorEx extendOne, DcMotorEx extendTwo) {
        return new Action() {
            boolean init = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    extendController.setSetpoint(distance);
                    init = true;
                }

                if (Math.abs(extendController.getError().get()) <= 0.15) {
                    telemetryPacket.put("extendError", extendController.getError());
                    return true;
                }

                telemetryPacket.put("extendError", extendController.getError());

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
            boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    pivotController.setSetpoint(angle);
                    init = true;
                }

                if (Math.abs(pivotController.getError().get()) <= 0.15) {
                    telemetry.addData("rotateCommand", 0);
                    return true;
                }

                telemetry.addData("rotateCommand", 1);

                return false;
            }
        };
    }
}
