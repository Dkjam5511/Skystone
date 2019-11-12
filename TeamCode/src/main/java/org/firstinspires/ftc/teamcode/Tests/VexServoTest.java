package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "VEX Servo Test", group = "Tests")
public class VexServoTest extends OpMode {

    CRServo servoL;

    ElapsedTime timer = new ElapsedTime();

    double pos = 0;

    @Override
    public void init() {
        servoL = hardwareMap.crservo.get("vex");
    }

    @Override
    public void loop() {
        /*
        if(gamepad1.a && timer.seconds() > .2){
            pos = pos - .01;
            timer.reset();
        }

        if (gamepad1.y && timer.seconds() > .2){
            pos = pos + .01;
            timer.reset();
        }
*/

        pos = gamepad1.left_stick_y * .88;

        servoL.setPower(pos);

        telemetry.addData("Pos: ", pos);
    }
}

// .02 .34 .7 1
