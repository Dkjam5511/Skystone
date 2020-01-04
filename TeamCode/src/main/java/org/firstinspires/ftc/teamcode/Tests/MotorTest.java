package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Motor Test", group = "Tests")
public class MotorTest extends OpMode {

    DcMotor motor1;
    DcMotor motor2;

    double motor1pos;
    double motor2pos;

    double prevmotor1pos = 0;
    double prevmotor2pos = 0;

    double tickspersecond1;
    double tickspersecond2;

    double difference;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("vl");
        motor2 = hardwareMap.dcMotor.get("vl2");

        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer.reset();
    }

    @Override
    public void loop() {
        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.left_stick_y);

        if (timer.seconds() > 1){
            motor1pos = motor1.getCurrentPosition();
            motor2pos = motor2.getCurrentPosition();

            tickspersecond1 = motor1pos - prevmotor1pos;
            tickspersecond2 = motor2pos - prevmotor2pos;

            prevmotor1pos = motor1pos;
            prevmotor2pos = motor2pos;

            difference = Math.abs(tickspersecond1 - tickspersecond2);

            timer.reset();
        }

        telemetry.addData("1: ", tickspersecond1);
        telemetry.addData("2: ", tickspersecond2);
        telemetry.addData("Difference: ", difference);
        telemetry.update();

    }
}
