package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.Robot;

@TeleOp(name = "Encoder Test", group = "Tests")
public class EncoderTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("IMU Heading: ", localizer.imu.getIMUHeading());
            telemetry.addData("X: ", localizer.getXPosition());
            telemetry.addData("Y: ", localizer.getYPosition());
            telemetry.addData("RR Heading Degrees: ", localizer.getCurrentHeadingDegrees());
            telemetry.addData("RR Heading Radians: ", localizer.getCurrentHeadingRadians());
            telemetry.update();
        }
    }
}
