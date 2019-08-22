package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometers {
    public DcMotor xOdom;
    public DcMotor yOdom;

    public Odometers(DcMotor xOdom, DcMotor yOdom){
        this.xOdom = xOdom;
        this.yOdom = yOdom;
    }
}
