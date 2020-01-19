package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Red Auto", group = "Autonomous")
@Disabled
public class RedMainAuto extends Robot {

    VuforiaStuff.skystonePos pos;
    int stoneDiff;
    final static double STANDARD_SPEED_MODIFIER = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, true);
        intake.on();
        //liftSystem.hLift.setPower(-.3);
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
                /*
                driveToPoint2(-8, 42, 0, 9, 5);
                 */
                stoneDiff = 15;
                break;
        }
        if (pos == VuforiaStuff.skystonePos.RIGHT) {
            driveToPoint(0, -13, 0, STANDARD_SPEED_MODIFIER);
        } else {
            driveToPoint(0, -15, 0, STANDARD_SPEED_MODIFIER);
        }
        liftSystem.grabStone();
        //liftSystem.hLift.setPower(0);
        intake.off();
        turn_to_heading(270, 0);
        liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
        driveToPoint(0, -66 - stoneDiff, 270, 11);
        turn_to_heading(180, 0); // do the drop
        grabbers.ready();
        //liftSystem.extend();
        driveToPoint(0, -10, 180, 10);
        grabbers.down();
        sleep(1000);
        liftSystem.dropStone();
        sleep(500);
        //liftSystem.retract();
        liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
        driveToPoint(0, 14, 180, 3);
        turn_to_heading(265, 15);
        driveToPoint(0, -12, 270, 2.4);
        grabbers.up();
        sleep(200);
        driveToPoint(-2, 42, 270, 11);
        //liftSystem.hLift.setPower(-.2);
        sleep(500);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}
