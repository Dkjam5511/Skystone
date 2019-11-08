package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalPositions;

public class FoundationGrabbers {
    Servo hookL;
    Servo hookR;

    public FoundationGrabbers(Servo hookL,Servo hookR){
        this.hookL = hookL;
        this.hookR = hookR;
    }

    public void down(){
        hookL.setPosition(GlobalPositions.HOOKL_DOWN);
        hookR.setPosition(GlobalPositions.HOOKR_DOWN);
    }

    public void up(){
        hookL.setPosition(GlobalPositions.HOOKL_UP);
        hookR.setPosition(GlobalPositions.HOOKR_UP);
    }

    public void ready(){
        hookL.setPosition(GlobalPositions.HOOKL_READY);
        hookR.setPosition(GlobalPositions.HOOKR_READY);
    }
}
