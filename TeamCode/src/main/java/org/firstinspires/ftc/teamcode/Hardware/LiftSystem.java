package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalPositions;

public class LiftSystem {
    DcMotor vLift;
    public DcMotor hLift;
    Servo stoneGrabber;
    Servo stoneSpinner;

    public LiftSystem(DcMotor vLift, DcMotor hLift, Servo stoneGrabber, Servo stoneSpinner) {
        vLift.setDirection(DcMotorSimple.Direction.REVERSE);
        hLift.setDirection(DcMotorSimple.Direction.REVERSE);

        vLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.vLift = vLift;
        this.hLift = hLift;
        this.stoneGrabber = stoneGrabber;
        this.stoneSpinner = stoneSpinner;
    }

    public void grabStone() {
        stoneGrabber.setPosition(.56);
    }

    public void dropStone() {
        stoneGrabber.setPosition(0);
    }

    public void extend() {

        DbgLog.msg("10435 hLift Extending");

        ElapsedTime timeoutTimer = new ElapsedTime();

        while (hLift.getCurrentPosition() < 2000 && timeoutTimer.seconds() < 2) {
            hLift.setPower(1);
            DbgLog.msg("10435 hLiftTicks: " + hLift.getCurrentPosition());
            if (hLift.getCurrentPosition() > 1700) {
                stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            }
        }
        hLift.setPower(0);

    }

    public void retract() {
        ElapsedTime timeoutTimer = new ElapsedTime();

        while (hLift.getCurrentPosition() > 10) {
            hLift.setPower(-1);
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            timeoutTimer.reset();
        }
        while (timeoutTimer.seconds() < .5){
            hLift.setPower(-1);
        }

        hLift.setPower(0);


    }

    public void stopMotors(){
        vLift.setPower(0);
        hLift.setPower(0);
    }

}
