package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Encoder Test", group = "Tests")
public class EncoderTest extends OpMode {
    DcMotor encoderMotor;
    DcMotor encoderMotor1;

    @Override
    public void init() {
        encoderMotor = hardwareMap.dcMotor.get("lr");
        encoderMotor1 = hardwareMap.dcMotor.get("rr");
    }

    @Override
    public void loop() {
        telemetry.addData("Encoder Value X:", -encoderMotor.getCurrentPosition());
        telemetry.addData("Encoder Value Y:", encoderMotor1.getCurrentPosition());

    }
}
