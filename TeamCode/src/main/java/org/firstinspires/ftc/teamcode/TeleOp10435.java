package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp10435 extends OpMode {

    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;
    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor vLift;
    DcMotor hLift;
    Servo stoneGrabber;
    Servo stoneSpinner;
    int stoneLevel = 0;
    final static int stoneTickHeight = 600;
    boolean lifting = false;
    boolean manualLift = false;
    boolean intakeOn = false;
    boolean extending = false;
    boolean retracting = false;
    boolean releasing = false;

    ElapsedTime aTimer = new ElapsedTime();
    ElapsedTime xTimer = new ElapsedTime();
    ElapsedTime bTimer = new ElapsedTime();
    ElapsedTime yTimer = new ElapsedTime();
    ElapsedTime downTimer = new ElapsedTime();
    ElapsedTime upTimer = new ElapsedTime();

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        intakeL = hardwareMap.dcMotor.get("il");
        intakeR = hardwareMap.dcMotor.get("ir");
        vLift = hardwareMap.dcMotor.get("vl");
        hLift = hardwareMap.dcMotor.get("hl");
        stoneGrabber = hardwareMap.servo.get("sg");
        stoneSpinner = hardwareMap.servo.get("ss");

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        vLift.setDirection(DcMotorSimple.Direction.REVERSE);
        hLift.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stoneGrabber.setPosition(.5);
        stoneSpinner.setPosition(.7);
    }

    @Override
    public void loop() {
        //Driving
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

        lf.setPower(leftfrontpower);
        rf.setPower(rightfrontpower);
        lr.setPower(leftrearpower);
        rr.setPower(rightrearpower);

        if (gamepad1.right_bumper) {
            intakeOn = true;
        }
        if (gamepad1.left_bumper) {
            intakeOn = false;
        }

        if (intakeOn) {
            if ((gamepad1.right_trigger == 1)) {
                intakeL.setPower(-.75);
                intakeR.setPower(-.75);
            } else {
                intakeL.setPower(.75);
                intakeR.setPower(.75);
            }
        } else {
            if (gamepad1.right_trigger == 1) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
            } else {
                intakeL.setPower(0);
                intakeR.setPower(0);
            }
        }

        //Stone Grabs
        if (gamepad2.right_trigger >= .75) {
            stoneGrabber.setPosition(0);
        }

        if (gamepad2.left_trigger >= .75) {
            stoneGrabber.setPosition(.5);
            releasing = true;
        }

        if (gamepad2.dpad_left) {
            stoneSpinner.setPosition(.34);
        }
        if (gamepad2.dpad_up) {
            stoneSpinner.setPosition(.02);
        }
        if (gamepad2.dpad_right) {
            stoneSpinner.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            stoneSpinner.setPosition(.7);
        }

        //Lift Controls
        if (gamepad2.a && aTimer.seconds() > .2) {
            stoneLevel++;
            lifting = true;
            manualLift = false;
            aTimer.reset();
        }

        if (gamepad2.x && xTimer.seconds() > .2) {
            lifting = true;
            manualLift = false;
            xTimer.reset();
        }

        if (gamepad2.y && yTimer.seconds() > .2) {
            lifting = false;
            manualLift = false;
            retracting = true;
            stoneSpinner.setPosition(.7);
            yTimer.reset();
        }

        if (gamepad2.b && bTimer.seconds() > .2) {
            stoneLevel = 0;
            lifting = false;
            manualLift = false;
            bTimer.reset();
        }

        if (gamepad2.right_bumper && upTimer.seconds() > .3) {
            stoneLevel++;
            manualLift = false;
            upTimer.reset();
        }

        if (gamepad2.left_bumper && downTimer.seconds() > .3) {
            stoneLevel--;
            manualLift = false;
            downTimer.reset();
        }

        if (gamepad2.start) {
            extending = true;
        }

        //Vertical Lift
        if (lifting) {
            if (Math.abs(gamepad2.left_stick_y) > .1) {
                manualLift = true;
            }

            if (manualLift) {
                if (releasing && vLift.getCurrentPosition() < stoneLevel * stoneTickHeight + 300) {
                    vLift.setPower(1);
                } else {
                    releasing = false;
                    vLift.setPower(-gamepad2.left_stick_y);
                }
            } else {
                if (vLift.getCurrentPosition() < stoneLevel * stoneTickHeight - 50) {
                    vLift.setPower(1);
                } else if (vLift.getCurrentPosition() > stoneLevel * stoneTickHeight + 50) {
                    vLift.setPower(-1);
                } else {
                    vLift.setPower(0);
                }
            }
        } else {
            if (vLift.getCurrentPosition() > 10) {
                vLift.setPower(-1);
            } else {
                vLift.setPower(0);
            }
        }

        //Horizontal Lift
        if (extending && hLift.getCurrentPosition() < 1000) {
            hLift.setPower(1);
            if (hLift.getCurrentPosition() > 700) {
                stoneSpinner.setPosition(.02);
            }
        } else if (retracting && hLift.getCurrentPosition() > 10) {
            hLift.setPower(1);
            stoneSpinner.setPosition(.7);
        } else {
            extending = false;
            retracting = false;
            hLift.setPower(-gamepad2.right_stick_y);
        }

        telemetry.addData("Stone Level: ", stoneLevel);
        telemetry.addData("Encoder: ", hLift.getCurrentPosition());
    }
}