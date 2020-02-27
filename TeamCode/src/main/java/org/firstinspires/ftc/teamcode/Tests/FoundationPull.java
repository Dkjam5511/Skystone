package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.DriveParameters;
import org.firstinspires.ftc.teamcode.Autonomous.Robot;

@Autonomous(name = "FoundationPull", group = "Tests")
public class FoundationPull extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();

        grabbers.ready();

        double foundationX = 29;
        double foundationY2 = 84;


        localizer.setCurrentPose(new Pose2d(foundationX + 8, foundationY2, Math.toRadians(180)));


        // Foundation pull test.  Start it touching the platform...

        grabbers.down();
        sleep(500);

        pullFoundation(28, 60, true);
        pushFoundation(74);

// pull it in an arc
        /*

        driveTrain.applyPower(1, 1, 1, 1);
        sleep(400);

        driveTrain.applyPower(-.2, 1, .7, 1);
        sleep(1300);


         */
// push it tow the wall
/*        driveTrain.applyPower(-.7, -.7, -.7, -.7);
        sleep(700);
      */
        grabbers.up();

        driveTrain.applyPower(0, 0, 0, 0);
        //servoEventManager.addEvent(new ServoEvent(0, foundationY2 - 10, 0, 0, ServoEvent.SE_GRABBERS_UP));

// push it to wall and park
/*
        driveToPoint4(new ArrayList<DriveParameters>() {
            {
                //add(new DriveParameters(new Pose2d(foundationX + 8, foundationY2, Math.toRadians(180)), 2, 1, 2, 5));
                //add(new DriveParameters(new Pose2d(travelX, foundationY2 - 5, Math.toRadians(270)), .2, 1, 5, 5, 5));
                add(new DriveParameters(new Pose2d(travelX, 38, Math.toRadians(270)), 5, 1, 2, 5, 0));
            }
        });
*/

    }
}

