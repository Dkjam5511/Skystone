package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain {
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lr;
    private DcMotor rr;

    public DriveTrain(DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr, boolean runUsingEncoder) {
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (runUsingEncoder) {
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.lf = lf;
        this.rf = rf;
        this.lr = lr;
        this.rr = rr;
    }

    public void setRunMode(DcMotor.RunMode runMode){
        lf.setMode(runMode);
        lr.setMode(runMode);
        rf.setMode(runMode);
        rr.setMode(runMode);
    }

    public DcMotor.RunMode getRunMode(){
        return lf.getMode();
    }

    public void applyPower(double lfpower, double rfpower, double lrpower, double rrpower) {
        lf.setPower(lfpower);
        rf.setPower(rfpower);
        lr.setPower(lrpower);
        rr.setPower(rrpower);
    }

    public double[] calcWheelPowers(double leftStickX, double leftStickY, double rightStickX) {

        double wheelPower;
        double stickAngleRadians;
        double rightX;
        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        wheelPower = Math.hypot(leftStickX, leftStickY);
        stickAngleRadians = Math.atan2(leftStickY, leftStickX);

        stickAngleRadians = stickAngleRadians - Math.PI / 4; //adjust by 45 degrees

        double sinAngleRadians = Math.sin(stickAngleRadians);
        double cosAngleRadians = Math.cos(stickAngleRadians);
        double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

        rightX = rightStickX * .5;

        lfPower = wheelPower * cosAngleRadians * factor + rightX;
        rfPower = wheelPower * sinAngleRadians * factor - rightX;
        lrPower = wheelPower * sinAngleRadians * factor + rightX;
        rrPower = wheelPower * cosAngleRadians * factor - rightX;


        double[] array = new double[]{lfPower, rfPower, lrPower, rrPower};

        return array;
    }
}
