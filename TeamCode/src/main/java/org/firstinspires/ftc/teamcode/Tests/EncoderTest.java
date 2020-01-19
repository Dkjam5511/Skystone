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

        while (opModeIsActive()) {
            telemetry.addData("X Pos",odometers.getXPos());
            telemetry.addData("Y Pos", odometers.getYPos());
            telemetry.update();
        }
    }
}
