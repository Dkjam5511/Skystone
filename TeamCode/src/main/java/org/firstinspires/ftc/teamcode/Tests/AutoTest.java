package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous (name = "AutoTest", group = "Tests")
public class AutoTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        /*
        driveToPoint2(-10, 37,0, 10, 3.5 );
        sleep(2000);
        driveToPoint2(10, 0, 0,10,0);
        sleep(2000);
        driveToPoint2(0, -37, 0, 10,0);
        sleep(2000);
        driveToPoint2(-10, 80,0, 10, 3.5 );

         */

        driveToPoint(20,0,0,5);
        sleep(500);
        driveToPoint(-20,0,0,5);

        /*
        turn_to_heading(90, 0);
        sleep(2000);
        turn_to_heading(0, 0);

         */

/*
        turn_to_heading(315,0);
        intake.reverse();
        driveToPoint(0,8, 315, 3.5);
        intake.on();
        driveToPoint(0,7, 315, 5);
        driveToPoint(0, -15,315,9);
        driveToPoint(0,-70,0,9);

 */
    }
}
