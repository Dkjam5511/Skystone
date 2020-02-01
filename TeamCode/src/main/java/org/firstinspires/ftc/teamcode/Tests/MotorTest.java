package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Motor Test", group = "Tests")
public class MotorTest extends OpMode {

    DcMotor lf;
    DcMotor lr;
    DcMotor rf;
    DcMotor rr;

    double lfPos = 0;
    double lrPos = 0;
    double rfPos = 0;
    double rrPos = 0;

    double prevlfpos = 0;
    double prevlrpos = 0;
    double prevrfpos = 0;
    double prevrrpos = 0;

    double ticksPerSecondlf;
    double ticksPerSecondlr;
    double ticksPerSecondrf;
    double ticksPerSecondrr;

    double difference;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        lr = hardwareMap.dcMotor.get("lr");
        rf = hardwareMap.dcMotor.get("rf");
        rr = hardwareMap.dcMotor.get("rr");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();
    }

    @Override
    public void loop() {
        lf.setPower(gamepad1.left_stick_y);
        lr.setPower(gamepad1.left_stick_y);
        rf.setPower(gamepad1.left_stick_y);
        rr.setPower(gamepad1.left_stick_y);

        if (timer.seconds() > 1){
            lfPos = lf.getCurrentPosition();
            lrPos = lr.getCurrentPosition();
            rfPos = rf.getCurrentPosition();
            rrPos = rr.getCurrentPosition();

            ticksPerSecondlf = lfPos - prevlfpos;
            ticksPerSecondlr = lrPos - prevlrpos;
            ticksPerSecondrf = rfPos - prevrfpos;
            ticksPerSecondrr = rrPos - prevrrpos;

            prevlfpos = lfPos;
            prevlrpos = lrPos;
            prevrfpos = rfPos;
            prevrrpos = rrPos;



            timer.reset();
        }

        telemetry.addData("LF: ", ticksPerSecondlf);
        telemetry.addData("LR: ", ticksPerSecondlr);
        telemetry.addData("RF: ", ticksPerSecondrf);
        telemetry.addData("RR: ", ticksPerSecondrr);
        telemetry.update();

    }
}
