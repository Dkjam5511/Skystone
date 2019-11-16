package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalPositions;

public class LiftSystem {
    public CRServo hLift;
    DcMotor hLiftEncoder;
    Servo stoneGrabber;
    Servo stoneSpinner;

    public enum ExtensionState {
        EXTENDING, RETRACTING, STOPPED
    }

    ElapsedTime timeoutTimer = new ElapsedTime();

    public ExtensionState extensionState = ExtensionState.STOPPED;
    public ExtensionState prevExtensionState = ExtensionState.STOPPED;

    public LiftSystem(CRServo hLift, DcMotor hLiftEncoder, Servo stoneGrabber, Servo stoneSpinner) {
        hLiftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        hLiftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hLiftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.hLift = hLift;
        this.hLiftEncoder = hLiftEncoder;
        this.stoneGrabber = stoneGrabber;
        this.stoneSpinner = stoneSpinner;
    }

    public void runLift(){
        if (prevExtensionState != extensionState){ //Checks if the state has changed
            timeoutTimer.reset(); //Keeps track how long the state has been running for
        }
        prevExtensionState = extensionState;

        switch (extensionState){
            case EXTENDING:
                extend();
                break;
            case RETRACTING:
                retract();
                break;
            case STOPPED:
                break;
        }
    }

    public void grabStone() {
        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_DOWN);
    }

    public void dropStone() {
        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_UP);
    }

    public void extend() {
        DbgLog.msg("10435 hLift Extending");

        if (hLiftEncoder.getCurrentPosition() < 7000 && timeoutTimer.seconds() < 2) {
            hLift.setPower(GlobalPositions.HLIFT_FORWARD_SPEED);
            DbgLog.msg("10435 hLiftTicks: " + hLiftEncoder.getCurrentPosition());
            if (hLiftEncoder.getCurrentPosition() > 6000) {
                stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            }
        } else {
            hLift.setPower(0);
            extensionState = ExtensionState.STOPPED;
        }
    }

    public void retract() {
        if (hLiftEncoder.getCurrentPosition() > 20 && timeoutTimer.seconds() < 2) {
            hLift.setPower(GlobalPositions.HLIFT_REVERSE_SPEED);
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        } else {
            hLift.setPower(0);
            extensionState = ExtensionState.STOPPED;
        }

    }

    public void stopMotors() {
        hLift.setPower(0);
    }

}
