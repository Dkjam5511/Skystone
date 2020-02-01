package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Encoder Test", group = "Tests")
public class EncoderTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Pos: ",odometers.getLeftEncoder().getCurrentPosition());
            telemetry.addData("Right Pos: ", odometers.getRightEncoder().getCurrentPosition());
            telemetry.addData("RR Heading: ", getRRHeading());
            telemetry.addData("IMU Heading: ", imu.getCurrentHeading());
            telemetry.addData("IMU angles: ", imu.printAngles());
            telemetry.addData("Scuffed Heading: ", odometers.getHeading());
            telemetry.addData("Calibration status: ", imu.getCalibrationStatus().toString());
            telemetry.update();
        }
    }
}
