package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.RoboPosition;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Hardware.Odometers;

abstract public class Robot extends LinearOpMode {
    public DriveTrain driveTrain;
    public Odometers odometers;
    public IMU imu;
    BNO055IMU gyro;

    public void roboInit() {
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rr = hardwareMap.dcMotor.get("rr");
        DcMotor xOdom = hardwareMap.dcMotor.get("x");
        DcMotor yOdom = hardwareMap.dcMotor.get("y");
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        imu = new IMU(gyro);
        imu.initialize();

        driveTrain = new DriveTrain(lf, rf, lr, rr);
        odometers = new Odometers(xOdom, yOdom);
    }

    public void driveToPoint(double x, double y, double speed, double heading) {
        while (Math.abs(RoboPosition.currentXPos - x) > 50 && Math.abs(RoboPosition.currentYPos - y) > 50) {
            RoboPosition.currentXPos = odometers.xOdom.getCurrentPosition();
            RoboPosition.currentYPos = odometers.yOdom.getCurrentPosition();
            RoboPosition.currentHeading = imu.readCurrentHeading();
            double distanceToX = x - RoboPosition.currentXPos;
            double distanceToY = y - RoboPosition.currentXPos;
            double angleradians;
            angleradians = Math.atan2(distanceToX,distanceToY);
            double adjustment = headingAdjustment(heading);
            driveTrain.applyPower(speed * Math.cos(angleradians) + adjustment,speed * Math.sin(angleradians) - adjustment,speed * Math.cos(angleradians) + adjustment, speed * Math.sin(angleradians) - adjustment);
        }
        driveTrain.applyPower(0,0,0,0);
    }

    public double headingAdjustment(double targetHeading){
        double adjustment;
        double currentHeading;
        double degreesOff;
        boolean goRight;

        currentHeading = imu.readCurrentHeading();

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
