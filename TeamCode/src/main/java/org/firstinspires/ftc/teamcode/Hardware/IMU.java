package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {

    BNO055IMU imu;
    Orientation angles;

    public IMU(BNO055IMU imu) {
        this.imu = imu;
    }

    public void initialize(){
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        //IMUParameters.mode = BNO055IMU.SensorMode.IMU;
        //IMUParameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        //IMUParameters.calibrationDataFile = "IMUCalibration.json";

        imu.initialize(IMUParameters);
    }

    public boolean getisGyroCalibrated(){
        return imu.isGyroCalibrated();
    }

    public BNO055IMU.CalibrationStatus getCalibrationStatus(){
        return imu.getCalibrationStatus();
    }

    public double getCurrentHeading() {
        double currentHeading;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle; //Because REV Hub is upside down
        if (currentHeading > 0) {
            currentHeading = 360 - currentHeading;
        } else {
            currentHeading = -currentHeading;
        }
        return currentHeading;
    }

    public double[] printAngles(){
        double[] values;

        values = new double[3];
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        values[0] = angles.firstAngle;
        values[1] = angles.secondAngle;
        values[2] = angles.thirdAngle;

        return values;
    }

    public double headingAdjustment(double targetHeading){
        double adjustment;
        double currentHeading;
        double degreesOff;
        boolean goRight;

        currentHeading = getCurrentHeading();

        goRight = targetHeading > currentHeading;
        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / 5, 2) + 2) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        return adjustment;
    }
}
