package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Intake Test", group = "Tests")
public class IntakeTest extends OpMode {

    DcMotor intakeL;
    DcMotor intakeR;

    @Override
    public void init() {
        intakeL = hardwareMap.dcMotor.get("il");
        intakeR = hardwareMap.dcMotor.get("ir");

        intakeL.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intakeL.setPower(.6);
        intakeR.setPower(1);
    }
}
