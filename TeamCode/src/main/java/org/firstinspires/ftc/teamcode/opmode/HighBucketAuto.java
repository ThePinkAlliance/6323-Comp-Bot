package org.firstinspires.ftc.teamcode.opmode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MathUtils;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PIController;

// http://192.168.43.1:8080/dash
@Autonomous(name = "High Bucket")
public final class HighBucketAuto extends LinearOpMode {
    private boolean autoIsComplete = false;
    private final double COMPENSATION_VOLTAGE = 12.30;
    private final double PIVOT_GEAR_RATIO = (1/5281.1);
    private final double MAX_EXTEND_ROTATIONS = 6.539;

    public PIController pivotController = new PIController(1, 0.002);
    public PIController extendController = new PIController(0.5, 0.001);

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx pivotMotor = hardwareMap.get(DcMotorEx.class, "pivot");
        DcMotorEx extendOne = hardwareMap.get(DcMotorEx.class, "extendOne");
        DcMotorEx extendTwo = hardwareMap.get(DcMotorEx.class, "extendTwo");

        CRServo leftCollectServo = hardwareMap.get(CRServo.class, "vexleft");
        CRServo rightCollectServo = hardwareMap.get(CRServo.class, "vexright");
        Servo bumper = hardwareMap.get(Servo.class, "bumper");

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        extendOne.setDirection(DcMotorSimple.Direction.FORWARD);
        extendTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftCollectServo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCollectServo.setDirection(DcMotorSimple.Direction.REVERSE);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        Action goToBucketStrafe = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-13, 17))
                                .build();
        Action leave = drive.actionBuilder(new Pose2d(-10, 15, 0)).strafeToSplineHeading(new Vector2d(15, 40), -Math.PI/6).strafeToSplineHeading(new Vector2d(40, 40), -Math.PI/6).build();
        Action wait = drive.actionBuilder(new Pose2d(40, 20, 0)).waitSeconds(2).build();

        Actions.runBlocking(
                    new ParallelAction(controlAction(extendOne, extendTwo, pivotMotor, voltageSensor),
                        new SequentialAction(
                                // Extend to maximum distance
                                goToBucketStrafe,
                                rotateArm(2.24),
                                // Rotate the arm to straight vertical
                                drive.actionBuilder(new Pose2d(-10, 15, 0)).waitSeconds(1).build(),
                                // Spin the collector
                                new InstantAction(() -> {
                                    bumper.setPosition(0);
                                    leftCollectServo.setPower(0.4);
                                    rightCollectServo.setPower(0.4);
                                }),
                                wait,
                                // Stop the collector
                                new InstantAction(() -> { leftCollectServo.setPower(0); rightCollectServo.setPower(0); bumper.setPosition(0.2); }),
                                // Bring the arm to zero
//                                new ParallelAction(leave, rotateArm(1.34)),
                                // Stop any parallel actions
                                new InstantAction(() -> autoIsComplete = true)
                        )
                ));
    }

    public Action controlAction(DcMotorEx extendOne, DcMotorEx extendTwo, DcMotorEx pivotMotor, VoltageSensor voltageSensor) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.put("pivotError", pivotController.getError().get());
                telemetryPacket.put("extendError", extendController.getError().get());

                executePivotControl(pivotMotor, voltageSensor);
                executeExtendControl(extendOne, extendTwo, voltageSensor);

                if (autoIsComplete) {
                    return false;
                }

                return true;
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
            double startTime = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init) {
                    extendController.setSetpoint(distance);
                    startTime = System.currentTimeMillis();
                }

                if ((System.currentTimeMillis() - startTime) >= 1000) {
                    return false;
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
            double startTime = 0;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (init == false) {
                    pivotController.setSetpoint(angle);
                    init = true;
                    startTime = System.currentTimeMillis();
                }

                if ((System.currentTimeMillis() - startTime) >= 1000) {
                    return false;
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
