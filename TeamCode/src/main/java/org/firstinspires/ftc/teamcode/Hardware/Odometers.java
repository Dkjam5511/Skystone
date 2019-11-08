package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometers {
    public DcMotor xOdom;
    public DcMotor yOdom;

    public Odometers(DcMotor xOdom, DcMotor yOdom) {
        xOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.xOdom = xOdom;
        this.yOdom = yOdom;

    }

    public double getXPos() {
        return xOdom.getCurrentPosition();
    }

    public double getYPos() {
        return yOdom.getCurrentPosition(); //No negative because IntakeL is reversed
    }
}
