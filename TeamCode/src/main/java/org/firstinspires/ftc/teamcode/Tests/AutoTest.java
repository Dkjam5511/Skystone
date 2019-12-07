package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous (name = "AutoTest", group = "Tests")
public class AutoTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();

        driveTrain.applyPower(.5,.5,.5,.5);
        sleep(2500);
        driveTrain.applyPower(0,0,0,0);

        /*
        driveToPoint3(-10, 37,0, 1, 3.5 );
        sleep(2000);
        driveToPoint3(10, 0, 0,1,0);
        sleep(2000);
        driveToPoint3(0, -37, 0, 1,0);
        sleep(2000);
        driveToPoint3(-10, 80,0, 1, 3.5 );
        */

        /*

        driveToPoint3(0,90,0, 1);
        sleep(2000);
        driveToPoint3(0,-90,0, 1);
        driveToPoint3(0,50,0, 1);
        */


        //driveToPoint3(7.5,-1,90, .7,1);
        //driveToPoint3(0, -(68 ), 90, 1, 1);

        /*
        turn_to_heading(90, 0);
        sleep(3000);
        turn_to_heading(90, 0);
        sleep(3000);
        turn_to_heading(0, 0);
        sleep(3000);
        turn_to_heading(0, 0);
        sleep(3000);
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
