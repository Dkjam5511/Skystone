package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Servo Test", group = "Tests")
public class ServoTest extends OpMode {

    Servo servo1;
    Servo servo2;

    ElapsedTime timer = new ElapsedTime();

    double pos = .5;
    double pos2 = .5;

    @Override
    public void init() {
        servo1 = hardwareMap.servo.get("lc");
        servo2 = hardwareMap.servo.get("lcp");
    }

    @Override
    public void loop() {
        if(gamepad1.a && timer.seconds() > .2){
            pos = pos + .01;
            timer.reset();
        }

        if (gamepad1.y && timer.seconds() > .2){
            pos = pos - .01;
            timer.reset();
        }

        if(gamepad1.x && timer.seconds() > .2){
            pos2 = pos2 + .01;
            timer.reset();
        }

        if (gamepad1.b && timer.seconds() > .2){
            pos2 = pos2 - .01;
            timer.reset();
        }
        servo1.setPosition(pos);
        servo2.setPosition(pos2);

        telemetry.addData("Pos1: ", pos);
        telemetry.addData("Pos2: ", pos2);
        telemetry.update();
    }
}

// .02 .34 .7 1
