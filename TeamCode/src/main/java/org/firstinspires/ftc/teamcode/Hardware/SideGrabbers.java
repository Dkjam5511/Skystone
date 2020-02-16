package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GlobalPositions;

public class SideGrabbers {

    private Servo rightClaw;
    private Servo rightClawPivot;
    private Servo leftClaw;
    private Servo leftClawPivot;

    public SideGrabbers(Servo rightClaw, Servo rightClawPivot, Servo leftClaw, Servo leftClawPivot){
        this.rightClaw = rightClaw;
        this.rightClawPivot = rightClawPivot;
        this.leftClaw = leftClaw;
        this.leftClawPivot = leftClawPivot;
    }

    public void lowerLeftClaw(){
        leftClawPivot.setPosition(GlobalPositions.LEFT_CLAW_PIVOT_DOWN);
    }

    public void raiseLeftClaw(){
        leftClawPivot.setPosition(GlobalPositions.LEFT_CLAW_PIVOT_UP);
    }

    public void clampLeftClaw(){
        leftClaw.setPosition(GlobalPositions.LEFT_CLAW_CLOSED);
    }

    public void openLeftClaw(){
        leftClaw.setPosition(GlobalPositions.LEFT_CLAW_OPEN);
    }

    public void lowerRightClaw(){
        rightClawPivot.setPosition(GlobalPositions.RIGHT_CLAW_PIVOT_DOWN);
    }

    public void RaiseRightClaw(){
        rightClawPivot.setPosition(GlobalPositions.RIGHT_CLAW_PIVOT_UP);
    }

    public void clampRightClaw(){
        rightClaw.setPosition(GlobalPositions.RIGHT_CLAW_CLOSED);
    }

    public void openRightClaw(){
        rightClaw.setPosition(GlobalPositions.RIGHT_CLAW_OPEN);
    }

    public Servo getLeftClaw() {
        return leftClaw;
    }

    public Servo getLeftClawPivot() {
        return leftClawPivot;
    }

    public Servo getRightClaw() {
        return rightClaw;
    }
}
