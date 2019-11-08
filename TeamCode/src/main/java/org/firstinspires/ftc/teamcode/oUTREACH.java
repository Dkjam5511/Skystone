package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "Outreach", group = "TeleOp")
public class oUTREACH extends OpMode {

    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double leftstickx;
        double leftsticky;
        double rightstickx;
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

        lf.setPower(leftfrontpower * .5);
        rf.setPower(rightfrontpower * .5);
        lr.setPower(leftrearpower * .5);
        rr.setPower(rightrearpower * .5);
    }
}
