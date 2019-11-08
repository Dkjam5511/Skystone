package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Intake {
    DcMotor intakeL;
    DcMotor intakeR;

    public Intake(DcMotor intakeL, DcMotor intakeR){

        intakeL.setDirection(DcMotor.Direction.REVERSE);

        this.intakeL = intakeL;
        this.intakeR = intakeR;
    }

    public void on(){
        intakeL.setPower(.75);
        intakeR.setPower(.75);
    }

    public void off(){
        intakeL.setPower(0);
        intakeR.setPower(0);
    }

    public void reverse(){
        intakeL.setPower(-.75);
        intakeR.setPower(-.75);
    }
}
