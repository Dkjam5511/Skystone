package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous.PID;
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

        waitForStart();
    }

    public void driveToPoint(double x, double y, double heading) {
        PID pid = new PID(x, y, .06,.06,.05);

        double distanceToX = x - RoboPosition.currentXPos;
        double distanceToY = y - RoboPosition.currentXPos;

        double[] vector = pid.calcErrors(distanceToX, distanceToY);

        while (vector[0] != 0 && vector[1] != 0) {
            RoboPosition.currentXPos = odometers.xOdom.getCurrentPosition();
            RoboPosition.currentYPos = odometers.yOdom.getCurrentPosition();
            RoboPosition.currentHeading = imu.readCurrentHeading();

            distanceToX = x - RoboPosition.currentXPos;
            distanceToY = y - RoboPosition.currentXPos;

            vector = pid.calcErrors(distanceToX, distanceToY);
            double wheelPower = Math.hypot(vector[0], vector[1]);

            double angleradians;
            angleradians = Math.atan2(distanceToX,distanceToY) - Math.PI / 4;

            double adjustment = imu.headingAdjustment(heading);

            driveTrain.applyPower(wheelPower * Math.cos(angleradians) + adjustment,wheelPower * Math.sin(angleradians) - adjustment,wheelPower * Math.cos(angleradians) + adjustment, wheelPower * Math.sin(angleradians) - adjustment);
        }
        driveTrain.applyPower(0,0,0,0);
    }
}
