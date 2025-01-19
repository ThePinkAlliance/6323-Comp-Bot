package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PinkMecanumDrive extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private LazyImu imu;
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    double trackWidthRadius = 8;


    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "3");
        backLeft = hardwareMap.get(DcMotorEx.class, "2");
        backRight = hardwareMap.get(DcMotorEx.class, "0");
        frontRight = hardwareMap.get(DcMotorEx.class, "1");
        imu = new LazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                logoFacingDirection, usbFacingDirection));

        waitForStart();

        while (opModeIsActive()) {
            double velocity_forward = (frontRight.getCurrentPosition() + frontLeft.getCurrentPosition() + backRight.getCurrentPosition() + backLeft.getCurrentPosition()) / 4;
            double velocity_strafe = (backLeft.getCurrentPosition() + frontRight.getCurrentPosition() - frontRight.getCurrentPosition() - backRight.getCurrentPosition()) / 4;
            double velocity_rot = (backRight.getCurrentPosition() + frontRight.getCurrentPosition() - frontRight.getCurrentPosition() - backLeft.getCurrentPosition()) / (4 * (2 * trackWidthRadius));


        }
    }
}
