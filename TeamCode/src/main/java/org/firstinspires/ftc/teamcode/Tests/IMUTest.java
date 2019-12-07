package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
@TeleOp (name = "IMUTest", group = "Tests")
public class IMUTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        while (opModeIsActive()) {

            double leftstickx = 0;
            double leftsticky = 0;
            double rightstickx = 0;
            double wheelpower;
            double stickangleradians;
            double rightX;
            double leftfrontpower;
            double rightfrontpower;
            double leftrearpower;
            double rightrearpower;
            double dpadpower = .25;

            if (gamepad1.dpad_up) {
                leftsticky = dpadpower;
            } else if (gamepad1.dpad_right) {
                leftstickx = dpadpower;
            } else if (gamepad1.dpad_down) {
                leftsticky = -dpadpower;
            } else if (gamepad1.dpad_left) {
                leftstickx = -dpadpower;
            } else {
                leftstickx = gamepad1.left_stick_x;
                leftsticky = -gamepad1.left_stick_y;
                rightstickx = gamepad1.right_stick_x;
            }
            wheelpower = Math.hypot(leftstickx, leftsticky);
            stickangleradians = Math.atan2(leftsticky, leftstickx);

            stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

            rightX = rightstickx * .5;
            leftfrontpower = (wheelpower * Math.cos(stickangleradians) + rightX);
            rightfrontpower = (wheelpower * Math.sin(stickangleradians) - rightX);
            leftrearpower = (wheelpower * Math.sin(stickangleradians) + rightX);
            rightrearpower = (wheelpower * Math.cos(stickangleradians) - rightX);

            driveTrain.applyPower(leftfrontpower, rightfrontpower, leftrearpower, rightrearpower);
            /*
            double[] angles = imu.printAngles();
            telemetry.addData("Z: ", angles[0]);
            telemetry.addData("Y: ", angles[1]);
            telemetry.addData("X: ", angles[2]);
            telemetry.update();
                  */

            double heading = imu.readCurrentHeading();
            telemetry.addData("Heading", heading);
            telemetry.update();
        }
    }
}
