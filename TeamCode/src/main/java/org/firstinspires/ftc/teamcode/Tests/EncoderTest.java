package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Odometers;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.List;

@TeleOp(name = "Encoder Test", group = "Tests")
public class EncoderTest extends OpMode {

    SampleMecanumDriveREVOptimized drive;

    @Override
    public void init() {
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
    }

    @Override
    public void loop() {
        List<Double> pos = drive.getWheelPositions();

        telemetry.addData("Left: ", pos.get(0));
        telemetry.addData("Right: ", pos.get(1));
        telemetry.addData("Center: ", pos.get(2));
    }
}
