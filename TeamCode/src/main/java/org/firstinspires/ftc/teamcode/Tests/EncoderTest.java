package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.Odometers;

@TeleOp (name = "Encoder Test", group = "Tests")
public class EncoderTest extends OpMode {
    DcMotor encoderMotor;
    DcMotor encoderMotor1;

    Odometers odometers;

    @Override
    public void init() {
        encoderMotor = hardwareMap.dcMotor.get("ir");
        encoderMotor1 = hardwareMap.dcMotor.get("il");

        odometers = new Odometers(encoderMotor, encoderMotor1);
    }

    @Override
    public void loop() {
        telemetry.addData("Encoder Value X:", odometers.getXPos());
        telemetry.addData("Encoder Value Y:", odometers.getYPos());

    }
}
