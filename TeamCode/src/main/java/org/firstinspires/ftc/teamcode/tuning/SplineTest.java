package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PIController;
import org.firstinspires.ftc.teamcode.lib.ActionExecutor;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

// http://192.168.43.1:8080/dash
public final class SplineTest extends LinearOpMode {
    private final double COMPENSATION_VOLTAGE = 12.30;
    private final double PIVOT_GEAR_RATIO = (1/5281.1);
    private final double MAX_EXTEND_ROTATIONS = 6.539 - 0.5;

    public PIController pivotController = new PIController(1, 0.002);
    public PIController extendController = new PIController(0.5, 0.001);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");

        CRServo leftCollectServo = hardwareMap.get(CRServo.class, "vexleft");
        CRServo rightCollectServo = hardwareMap.get(CRServo.class, "vexright");

        ActionExecutor executor = new ActionExecutor();
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        extendOne.setDirection(DcMotorSimple.Direction.FORWARD);
        extendTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        Action goToBucket = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(10, 30), 0)
                .build();

        Action goToBucketStrafe = drive.actionBuilder(beginPose)
                        .strafeTo(new Vector2d(10, 0))
                                .build();

        Actions.runBlocking(
                    new ParallelAction(
                         controlAction(extendOne, extendTwo, pivotMotor, voltageSensor),
                        new SequentialAction(
                            goToBucketStrafe,
                                rotateArm(1.7),
                                extendArm(MAX_EXTEND_ROTATIONS)
                        )
                ));
    }

    public Action controlAction(DcMotorEx extendOne, DcMotorEx extendTwo, DcMotorEx pivotMotor, VoltageSensor voltageSensor) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                executePivotControl(pivotMotor, voltageSensor);
                executeExtendControl(extendOne, extendTwo, voltageSensor);

                return false;
            }
        };
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
     * @return
     */
    public Action extendArm(double distance) {
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
