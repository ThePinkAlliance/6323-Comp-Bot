package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.PIController;
import org.firstinspires.ftc.teamcode.lib.ActionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

/**
 * Servo port 0-1 are the vex servos
 * Servo port 2 is the gobuilda spin servo
 */
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    // On arm extend subtracted 0.5 from max extend length and it reached max
    private final double MAX_EXTEND_ROTATIONS = 6.539 - 0.5;
    private final double PIVOT_ZERO_OFFSET_RAD = 0.6258;
    // Amount of encoder ticks per rotation
    private final double PIVOT_GEAR_RATIO = (1/5281.1);
    private final double COMPENSATION_VOLTAGE = 12.30;
    private final double HIGH_BUCKET_ANGLE = 2.58;
    public PIController pivotController = new PIController(1, 0.002);
    public PIController extendController = new PIController(0.5, 0.001);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");

        Servo leftCollectServo = hardwareMap.get(Servo.class, "vexleft");
        Servo rightCollectServo = hardwareMap.get(Servo.class, "vexright");
        Servo bumper = hardwareMap.get(Servo.class, "bumper");

        ActionExecutor executor = new ActionExecutor();
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Drive drive = new Drive(hardwareMap);

        extendOne.setDirection(DcMotorSimple.Direction.FORWARD);
        extendTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftCollectServo.setDirection(Servo.Direction.REVERSE);
        rightCollectServo.setDirection(Servo.Direction.REVERSE);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            pivotController.setSetpoint(pivotController.getSetpoint() + gamepad2.left_stick_y * 0.09817477);
            extendController.setSetpoint(extendController.getSetpoint() + gamepad2.right_stick_y * (MAX_EXTEND_ROTATIONS / 32));

            if (gamepad1.a) {
                executor.run(new SequentialAction(rotateArm(HIGH_BUCKET_ANGLE), extendArm(MAX_EXTEND_ROTATIONS / 2, extendOne, extendTwo)));
            }

            if (gamepad1.y) {
                executor.run(new SequentialAction(rotateArm(HIGH_BUCKET_ANGLE), extendArm(MAX_EXTEND_ROTATIONS, extendOne, extendTwo)));
            }

            // Run the vex servos to run the collector
            if (gamepad1.right_bumper) {
                bumper.setPosition(0);
            } else {
                bumper.setPosition(0.2);
            }

            if (gamepad1.left_bumper) {
                leftCollectServo.setPosition(0.88);
                rightCollectServo.setPosition(0.88);
            } else {
                leftCollectServo.setPosition(0);
                rightCollectServo.setPosition(0);
            }

            if (gamepad1.b) {
                // -0.15 was estimated to close to specified encoder counts when at rest but I didn't get to test before leaving.
                executor.run(new SequentialAction(extendArm(0, extendOne, extendTwo), rotateArm(-0.15)));
            }

            executePivotControl(pivotMotor, voltageSensor);
            executeExtendControl(extendOne, extendTwo, voltageSensor);

            telemetry.addData("back right vel", drive.getRightBackVel());
            telemetry.addData("back left vel", drive.getLeftBackVel());
            telemetry.addData("front left vel", drive.getLeftFrontVel());
            telemetry.addData("front right vel", drive.getRightFrontVel());

            drive.setDriveCommand(gamepad1.left_stick_x * -1, gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.update();
            executor.execute();
        }
    }

    /**
     * executePivotControl runs the pivot pid controller to maintain the desired setpoint of the arm during a match.
     * In order to control the arm you need to call the pivot pid controller separately.
     */
    private void executePivotControl(DcMotorEx pivotMotor, VoltageSensor voltageSensor) {
        double motorRotationsRad = (pivotMotor.getCurrentPosition() * PIVOT_GEAR_RATIO * 2 * Math.PI);
        double pivotEffort = pivotController.calculate(motorRotationsRad) * (COMPENSATION_VOLTAGE / voltageSensor.getVoltage());
        pivotEffort = MathUtils.Helpers.clamp(pivotEffort, -1, 1);

        pivotMotor.setPower(pivotEffort);

        telemetry.addData("control effort", pivotEffort);
        telemetry.addData("pivot rotations", motorRotationsRad);
        telemetry.addData("pivot error", pivotController.getError().get());
    }

    /**
     * executeExtendControl runs the extend pid controller to maintain the desired setpoint of the arm during a match.
     * In order to control the arm you need to call the extend pid controller separately.
     */
    private void executeExtendControl(DcMotorEx extendOne, DcMotorEx extendTwo, VoltageSensor voltageSensor) {
        double currentExtend1 = extendOne.getCurrentPosition() * -0.001859773;
        double extendEffort = MathUtils.Helpers.clamp(extendController.calculate(currentExtend1) * (COMPENSATION_VOLTAGE / voltageSensor.getVoltage()), -1, 1);

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
                }

                // Exit if below 0.15 rotations of error
                if (Math.abs(extendController.getError().get()) <= 0.20) {
                    return false;
                }

                return true;
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
                if (init == false) {
                    pivotController.setSetpoint(angle);
                    init = true;
                }

                // Exit if below 0.15 radians of error
                if (Math.abs(pivotController.getError().get()) <= 0.15) {
                    return false;
                }

                return true;
            }
        };
    }
}
