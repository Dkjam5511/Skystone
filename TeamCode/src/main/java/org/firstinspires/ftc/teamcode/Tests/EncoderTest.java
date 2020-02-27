package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Robot;

@TeleOp(name = "Encoder Test", group = "Tests")
public class EncoderTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Front Encoder", ((double)(odometers.frontEncoder.getCurrentPosition()) / 2400) * 2.3622 * Math.PI);
            telemetry.addData("IMU Heading: ", localizer.imu.getIMUHeading());
            telemetry.addData("X: ", localizer.getXPosition());
            telemetry.addData("Y: ", localizer.getYPosition());
            telemetry.addData("RR Heading Degrees: ", localizer.getCurrentHeadingDegrees());
            telemetry.addData("RR Heading Radians: ", localizer.getCurrentHeadingRadians());
            telemetry.addData("Right Wall distance:", rDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
