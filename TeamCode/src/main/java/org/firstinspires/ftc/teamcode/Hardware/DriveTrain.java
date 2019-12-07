package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrain{
    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;

    public DriveTrain(DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr){
        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.lf = lf;
        this.rf = rf;
        this.lr = lr;
        this.rr = rr;
    }

    public void applyPower(double lfpower, double rfpower, double lrpower, double rrpower){
        lf.setPower(lfpower);
        rf.setPower(rfpower);
        lr.setPower(lrpower);
        rr.setPower(rrpower);
    }
}
