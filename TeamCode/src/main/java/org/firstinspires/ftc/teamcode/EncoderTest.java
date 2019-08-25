package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Encoder Test", group = "Tests")
public class EncoderTest extends OpMode {
    DcMotor encoderMotor;
    @Override
    public void init() {
        encoderMotor = hardwareMap.dcMotor.get("em");
    }

    @Override
    public void loop() {
        telemetry.addData("Encoder Value:", encoderMotor.getCurrentPosition());
    }
}
