package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp10435 extends OpMode {

    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;
    DcMotor intakeL;
    DcMotor intakeR;

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        intakeL = hardwareMap.dcMotor.get("il");
        intakeR = hardwareMap.dcMotor.get("ir");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;

        leftstickx = gamepad1.left_stick_x;
        leftsticky = -gamepad1.left_stick_y;
        rightstickx = gamepad1.right_stick_x;

        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .5;
        leftfrontpower = (wheelpower * Math.cos(stickangleradians) + rightX);
        rightfrontpower = (wheelpower * Math.sin(stickangleradians) - rightX);
        leftrearpower = (wheelpower * Math.sin(stickangleradians) + rightX);
        rightrearpower = (wheelpower * Math.cos(stickangleradians) - rightX);

        lf.setPower(leftfrontpower);
        rf.setPower(rightfrontpower);
        lr.setPower(leftrearpower);
        rr.setPower(rightrearpower);

        if (gamepad1.left_bumper){
            intakeL.setPower(0);
            intakeR.setPower(0);
        }

        if (gamepad1.right_bumper){
            intakeL.setPower(1);
            intakeR.setPower(1);
        }
    }
}
