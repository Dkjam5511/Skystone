package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Red Main Auto", group = "Autonomous")
public class RedMainAuto extends Robot {

    VuforiaStuff.skystonePos pos;
    int stoneDiff;
    final static double STANDARD_SPEED_MODIFIER = 12;
    double pickupHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, true);
        intake.on();
        switch (pos) {
            case LEFT:
                driveToPoint(-12, 37, 0, 9);
                stoneDiff = 0;
                break;
            case CENTER:
                driveToPoint(2, 42, 0, 9);
                stoneDiff = 12;
                break;
            case RIGHT:
                driveToPoint(10, 9, 0, 9);
                driveToPoint(0, 33, 0, 9);
                stoneDiff = 18;
                break;
        }
        if (pos == VuforiaStuff.skystonePos.LEFT) {
            driveToPoint(0, -11, 0, STANDARD_SPEED_MODIFIER);
        } else {
            driveToPoint(0, -17, 0, STANDARD_SPEED_MODIFIER);
        }
        liftSystem.grabStone();
        intake.off();
        turn_to_heading(90, 0);
        driveToPoint(0, -65 - stoneDiff, 90, 11);
        turn_to_heading(180, 0); // do the drop
        grabbers.ready();
        liftSystem.extend();
        driveToPoint(0, -6, 180, 10);
        grabbers.down();
        liftSystem.dropStone();
        sleep(500);
        liftSystem.retract();
        sleep(200);
        driveToPoint(0, 14, 180, 3);
        turn_to_heading(95, 15);
        driveToPoint(0, -12, 90, 2.4);
        grabbers.up();
        sleep(200);
        driveToPoint(-2, 42, 90, 11);
        /*
        turn_to_heading(90, false);
        if (pos == VuforiaStuff.skystonePos.RIGHT){
            stoneDiff  = 10;
            pickupHeading = 45;
        }
        driveToPoint(0, 94 + stoneDiff, 90, STANDARD_SPEED_MODIFIER);
        turn_to_heading(pickupHeading, false);
        intake.on();
        driveToPoint(0, 18, pickupHeading, STANDARD_SPEED_MODIFIER);
        driveToPoint(0, -18, pickupHeading, STANDARD_SPEED_MODIFIER);
        liftSystem.grabStone();
        intake.off();
        turn_to_heading(90, false);
        driveToPoint(0, -85 - stoneDiff, 90, STANDARD_SPEED_MODIFIER);
        turn_to_heading(180, false);
        liftSystem.extend();
        sleep(200);
        liftSystem.dropStone();
        sleep(500);
        liftSystem.retract();
        turn_to_heading(90, false);
        driveToPoint(0, 35, 90, STANDARD_SPEED_MODIFIER);
         */
        liftSystem.hLift.setPower(-.2);
        sleep(500);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}
