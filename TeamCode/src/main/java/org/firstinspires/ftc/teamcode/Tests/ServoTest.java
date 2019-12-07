package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Servo Test", group = "Tests")
public class ServoTest extends OpMode {

    Servo servoL;
    Servo servoR;

    ElapsedTime timer = new ElapsedTime();

    double pos = 0;

    @Override
    public void init() {
        servoL = hardwareMap.servo.get("cp");
        servoR = hardwareMap.servo.get("ss");
    }

    @Override
    public void loop() {
        if(gamepad1.a && timer.seconds() > .2){
            pos = pos - .01;
            timer.reset();
        }

        if (gamepad1.y && timer.seconds() > .2){
            pos = pos + .01;
            timer.reset();
        }

        servoL.setPosition(1);
        servoR.setPosition(pos);

        telemetry.addData("Pos: ", pos);
    }
}

// .02 .34 .7 1
