package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "AutoTest", group = "Tests")
public class AutoTest extends Robot {
    @Override


    public void runOpMode() throws InterruptedException {
        roboInit();
        //checkCoast(0,48,0, 1, .15);
        driveToPoint(0, 95, 0, 9);
        //intake.on();
        // RED pick up left stone from left - this works
        //driveToPoint3(-40, 22, 45, .7, 3);

        // RED pick up center stone from right
        //driveToPoint3(10, 40, 345, .7, 3);

        // RED pick up right stone from the right
       // driveToPoint3(18, 35, 345, .7, 8);

/**** Code to test odometers
        double XPos = odometers.getXPos();
        double rightPos = odometers.rightEncoder.getCurrentPosition();
        double leftPos = odometers.leftEncoder.getCurrentPosition();
        telemetry.addData("X Pos", XPos);
        telemetry.addData("Right", rightPos);
        telemetry.addData("Left", leftPos);
        telemetry.update();
        sleep(2000);

        while (opModeIsActive()) {


            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning

            telemetry.addData("X Pos diff", XPos - odometers.getXPos());
            telemetry.addData("Right diff", rightPos - odometers.rightEncoder.getCurrentPosition());
            telemetry.addData("Left diff", leftPos - odometers.leftEncoder.getCurrentPosition());
            telemetry.update();
            XPos = odometers.getXPos();
            rightPos = odometers.rightEncoder.getCurrentPosition();
            leftPos = odometers.leftEncoder.getCurrentPosition();
            sleep(2000);

            turn_to_heading(0, -25);  // was 50;  put x inches in next statement to account for x encoder turning

            telemetry.addData("X Pos diff", XPos - odometers.getXPos());
            telemetry.addData("Right diff", rightPos - odometers.rightEncoder.getCurrentPosition());
            telemetry.addData("Left diff", leftPos - odometers.leftEncoder.getCurrentPosition());
            telemetry.update();
            XPos = odometers.getXPos();
            rightPos = odometers.rightEncoder.getCurrentPosition();
            leftPos = odometers.leftEncoder.getCurrentPosition();
            sleep(2000);
        }

 *******/

/*
            driveToPoint3(0, -68, 90, 1, 0);
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            driveToPoint3(0, 68, 90, 1, 0);
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            driveToPoint3(0, -68, 90, 1, 0);
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            driveToPoint3(0, 68, 90, 1, 0);

            turn_to_heading(0, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            sleep(200);
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            sleep(200);
            turn_to_heading(0, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            sleep(200);
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            sleep(200);
            turn_to_heading(0, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            sleep(200);
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
*/
        /*

        //driveToPoint3(0,24,0,.6,0);
        //driveToPoint3(0, -20, 0, .8, 0);

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
