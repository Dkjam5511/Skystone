package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalPositions;

public class LiftSystem {
    private DcMotor vLift;
    private DcMotor vLift2;
    private Servo hLift;
    private Servo stoneGrabber;
    private Servo stoneSpinner;

    public LiftSystem(DcMotor vLift, DcMotor vLift2, Servo hLift, Servo stoneGrabber, Servo stoneSpinner) {
        this.vLift = vLift;
        this.vLift2 = vLift2;
        this.hLift = hLift;
        this.stoneGrabber = stoneGrabber;
        this.stoneSpinner = stoneSpinner;
    }

    public void grabStone() {
        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_DOWN);
    }

    public void dropStone() {
        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_UP);
    }

    public void setStoneSpinnerPos(double pos){
        stoneSpinner.setPosition(pos);
    }

    public void setHLiftPos(double pos){
        hLift.setPosition(pos);
    }



}
