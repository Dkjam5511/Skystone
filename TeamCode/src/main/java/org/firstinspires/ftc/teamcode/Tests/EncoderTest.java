package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Odometers;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

@TeleOp(name = "Encoder Test", group = "Tests")
public class EncoderTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();

        waitForStart();
        double XPos = odometers.getXPos();
        double rightPos = odometers.rightEncoder.getCurrentPosition();
        double leftPos = odometers.leftEncoder.getCurrentPosition();

        double beginXPos = XPos;
        double beginRightPos = rightPos;
        double beginLeftPos = leftPos;


        while (opModeIsActive()) {

            telemetry.addData("X Pos diff", XPos - beginXPos);
            telemetry.addData("Right diff", rightPos - beginRightPos);
            telemetry.addData("Left diff", leftPos - beginLeftPos);
            telemetry.addData("Diff", XPos - beginXPos + rightPos - beginRightPos + leftPos - beginLeftPos);
            telemetry.update();

            XPos = odometers.getXPos();
            rightPos = odometers.rightEncoder.getCurrentPosition();
            leftPos = odometers.leftEncoder.getCurrentPosition();
        }
    }
}
