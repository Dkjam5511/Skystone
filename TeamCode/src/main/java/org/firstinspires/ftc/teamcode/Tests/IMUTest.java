package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Robot;
@TeleOp (name = "IMUTest", group = "Tests")
public class IMUTest extends OpMode {

    public IMU imu;
    private BNO055IMU gyro;

    @Override
    public void init() {
        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        imu = new IMU(gyro);
        imu.initialize();
    }

    @Override
    public void loop() {
        double heading = imu.readCurrentHeading();
        telemetry.addData("Heading", heading);
        telemetry.update();
    }
}
