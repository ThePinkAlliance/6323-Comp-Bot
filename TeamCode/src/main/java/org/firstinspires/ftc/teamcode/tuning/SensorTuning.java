package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "Sensor Tuning")
public class SensorTuning extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        waitForStart();

        while (opModeIsActive()) {
            double leftFront = drive.leftFront.getCurrentPosition();
            double rightFront = drive.rightFront.getCurrentPosition();
            double leftBack = drive.leftBack.getCurrentPosition();
            double rightBack = drive.rightBack.getCurrentPosition();

            telemetry.addData("0 Ticks", leftFront);
            telemetry.addData("1 Ticks", leftBack);
            telemetry.addData("2 Ticks", rightBack);
            telemetry.addData("3 Ticks", rightFront);
            telemetry.update();
        }
    }
}
