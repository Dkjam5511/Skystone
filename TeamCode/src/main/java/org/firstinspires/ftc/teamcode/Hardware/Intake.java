package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Intake {
    DcMotor intakeL;
    DcMotor intakeR;

    public Intake(DcMotor intakeL, DcMotor intakeR){
        this.intakeL = intakeL;
        this.intakeR = intakeR;

        //intakeL.setDirection(DcMotorSimple.Direction.REVERSE); Taken out for odom
    }

    public void on(){
        intakeL.setPower(1);
        intakeR.setPower(-1);
    }

    public void onSlow(){
        intakeL.setPower(.3);
        intakeR.setPower(-.3);
    }

    public void off(){
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    public void reverse(){
        intakeL.setPower(-.75);
        intakeR.setPower(.75);
    }
}
