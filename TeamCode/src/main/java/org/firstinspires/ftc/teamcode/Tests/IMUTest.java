package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
@TeleOp (name = "IMUTest", group = "Tests")
public class IMUTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        while (opModeIsActive()) {
            double[] angles = imu.printAngles();
            telemetry.addData("Z: ", angles[0]);
            telemetry.addData("Y: ", angles[1]);
            telemetry.addData("X: ", angles[2]);
            telemetry.update();
        }
    }
}
