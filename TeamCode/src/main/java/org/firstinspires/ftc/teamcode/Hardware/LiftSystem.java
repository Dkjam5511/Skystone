package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalPositions;

public class LiftSystem {
    public Servo hLift;
    Servo stoneGrabber;
    public Servo stoneSpinner;

    public enum ExtensionState {
        EXTENDING, EXTENDINGFAR,RETRACTING, STOPPED
    }

    ElapsedTime timeoutTimer = new ElapsedTime();

    public ExtensionState extensionState = ExtensionState.STOPPED;
    public ExtensionState prevExtensionState = ExtensionState.STOPPED;

    public LiftSystem(Servo hLift, Servo stoneGrabber, Servo stoneSpinner) {
        this.hLift = hLift;
        this.stoneGrabber = stoneGrabber;
        this.stoneSpinner = stoneSpinner;
    }

    public void runLift(){
        if (prevExtensionState != extensionState){ //Checks if the state has changed
            timeoutTimer.reset(); //Keeps track how long the state has been running for
        }
        prevExtensionState = extensionState;

        switch (extensionState){
            case EXTENDINGFAR:
                extend(true);
                break;
            case EXTENDING:
                extend(false);
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

    public void extend(boolean far) {
        DbgLog.msg("10435 hLift Extending");

        if (timeoutTimer.seconds() > 1.5){
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
        }

        /*
        if (timeoutTimer.seconds() < 2) {
            hLift.setPower(GlobalPositions.HLIFT_FORWARD_SPEED);
            DbgLog.msg("10435 hLiftTicks: " + hLiftEncoder.getCurrentPosition());
            if (hLiftEncoder.getCurrentPosition() > 6000) {
                stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            }
        } else {
            hLift.setPower(0);
            extensionState = ExtensionState.STOPPED;
        }

         */
    }

    public void retract() {
        /*
        if (hLiftEncoder.getCurrentPosition() > 20 && timeoutTimer.seconds() < 2) {
            hLift.setPower(GlobalPositions.HLIFT_REVERSE_SPEED);
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        } else {
            hLift.setPower(0);
            extensionState = ExtensionState.STOPPED;
        }
         */
    }

    public void stopMotors() {
        /*
        hLift.setPower(0);

         */
    }

}
