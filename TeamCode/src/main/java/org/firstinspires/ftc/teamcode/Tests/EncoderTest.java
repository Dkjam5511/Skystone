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
        encoderMotor = hardwareMap.dcMotor.get("hl");

        encoderMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        encoderMotor.setPower(gamepad1.left_stick_y);

        telemetry.addData("Encoder Value X:", encoderMotor.getCurrentPosition());

    }
}
