package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.DriveParameters;
import org.firstinspires.ftc.teamcode.Autonomous.Robot;
import org.firstinspires.ftc.teamcode.Autonomous.ServoEvent;

import java.util.ArrayList;

@Autonomous(name = "AutoTest", group = "Tests")
public class AutoTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        try {

            double stoneX = 30;
            double travelX = 23;
            double foundationX = 29;
            double foundationY1 = 93;
            double foundationY2 = 83;

            roboInit();
            ArrayList<Pose2d> stoneOrder = new ArrayList<>(7);

            sideGrabbers.openLeftClaw();
            sideGrabbers.lowerLeftClaw();
            sleep(300);

            servoEventManager.addEvent(new ServoEvent(0, 6.9, 0, 0, ServoEvent.SE_LEFT_CLAW_CLOSE));

            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    add(new DriveParameters(new Pose2d(0, 10, Math.toRadians(0)), 1, 1, 3, 10,5, 1.4, 0));
                }
            });

            sideGrabbers.raiseLeftClaw();

        } catch (Exception e) {
            throw e;
        } finally {
            DbgLog.close();
            driveTrain.applyPower(0, 0, 0, 0);
            intake.off();
        }


    }

}


/*        intake.on();
        sleep(4000);*/
/****  Test X and Y encoders against RoadRunner X and Y  */
        /*
        for (int i = 0; i < 4; i++) {

            driveToPoint4(new ArrayList<Pose2d>() {
                {
                    add(new Pose2d(-40, 40, Math.toRadians(0)));
                }
            });
            sleep(1000);
            telemetry.addData("Side Encoders", ((double) (odometers.leftEncoder.getCurrentPosition() + odometers.rightEncoder.getCurrentPosition()) / 2400 / 2) * 2.3622 * Math.PI);
            telemetry.addData("RR X: ", localizer.getXPosition());
            telemetry.addData("Front Encoder", ((double) (odometers.frontEncoder.getCurrentPosition()) / 2400) * 2.3622 * Math.PI);
            telemetry.addData("RR Y: ", localizer.getYPosition());
            telemetry.addData("RR Heading: ", localizer.getCurrentHeadingDegrees());
            telemetry.update();


            driveToPoint4(new ArrayList<Pose2d>() {
                {
                    add(new Pose2d(0, 0, Math.toRadians(0)));
                }
            });
            sleep(1000);
            telemetry.addData("Side Encoders", ((double) (odometers.leftEncoder.getCurrentPosition() + odometers.rightEncoder.getCurrentPosition()) / 2400 / 2) * 2.3622 * Math.PI);
            telemetry.addData("RR X: ", localizer.getXPosition());
            telemetry.addData("Front Encoder", ((double) (odometers.frontEncoder.getCurrentPosition()) / 2400) * 2.3622 * Math.PI);
            telemetry.addData("RR Y: ", localizer.getYPosition());
            telemetry.addData("RR Heading: ", localizer.getCurrentHeadingDegrees());
            telemetry.update();
        }
        sleep(10000);
         */

        /*
        driveToPoint4(new ArrayList<Pose2d>() {
            {
                add(new Pose2d(0, -6, Math.toRadians(0)));
                add(new Pose2d(-80, -6, Math.toRadians(0)));
                add(new Pose2d(-80, 0, Math.toRadians(0)));
            }
        }, 1);

        driveToPoint4(new ArrayList<Pose2d>() {
            {
                add(new Pose2d(-80, -6, Math.toRadians(0)));
                add(new Pose2d(0, -6, Math.toRadians(0)));
                add(new Pose2d(0, 0, Math.toRadians(0)));
            }
        }, 1);

         */
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
