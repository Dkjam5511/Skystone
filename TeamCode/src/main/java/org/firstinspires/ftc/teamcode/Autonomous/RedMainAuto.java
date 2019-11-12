package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        pos = vuforiaStuff.vuforiascan(false, false);
        intake.on();
        switch (pos) {
            case RIGHT:
                driveToPoint(12, 38, 0, 9);
                stoneDiff = 0;
                break;
            case CENTER:
                driveToPoint(-2, 43, 0, 9);
                stoneDiff = 12;
                break;
            case LEFT:
                driveToPoint(-8, 9, 0, 9);
                driveToPoint(0, 34, 0, 9);
                stoneDiff = 18;
                break;
        }
        if (pos == VuforiaStuff.skystonePos.RIGHT) {
            driveToPoint(0, -13, 0, STANDARD_SPEED_MODIFIER);
        } else {
            driveToPoint(0, -15, 0, STANDARD_SPEED_MODIFIER);
        }
        liftSystem.grabStone();
        intake.off();
        turn_to_heading(270, 0);
        driveToPoint(0, -65 - stoneDiff, 270, 11);
        turn_to_heading(180, 0); // do the drop
        grabbers.ready();
        liftSystem.extend();
        driveToPoint(0, -10, 180, 10);
        grabbers.down();
        sleep(1000);
        liftSystem.dropStone();
        sleep(500);
        liftSystem.retract();
        sleep(200);
        driveToPoint(0, 14, 180, 3);
        liftSystem.hLift.setPower(0);
        turn_to_heading(265, 15);
        driveToPoint(0, -12, 270, 2.4);
        grabbers.up();
        sleep(200);
        driveToPoint(2, 42, 270, 11);
        liftSystem.hLift.setPower(-.2);
        sleep(500);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}
