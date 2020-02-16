package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometers {
    public DcMotor frontEncoder;
    public DcMotor leftEncoder;
    public DcMotor rightEncoder;

    public Odometers(DcMotor frontEncoder, DcMotor leftEncoder, DcMotor rightEncoder) {
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.frontEncoder = frontEncoder;
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    public double getXPos() {
        return -frontEncoder.getCurrentPosition();
    }

    public double getYPos() {
        return (-rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition())/2; //No negative because IntakeL is reversed
    }

    public DcMotor getLeftEncoder() {
        return leftEncoder;
    }

    public DcMotor getRightEncoder() {
        return rightEncoder;
    }
    /*
    public double getHeading(){
        currentHeading = (((leftEncoder.getCurrentPosition() - rightEncoder.getCurrentPosition())*45.0/1778.0 )/35.56);
        //currentHeading += ((((leftEncoder.getCurrentPosition() - prevLeftTicks) * 6 * Math.PI - ((rightEncoder.getCurrentPosition() - prevRightTicks) * 6 * Math.PI))/38.1)*3)/20;

        //prevLeftTicks = leftEncoder.getCurrentPosition();
        //prevRightTicks = rightEncoder.getCurrentPosition();

        return currentHeading;
    }

     */
}
