package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Red Auto 2 Stone", group = "Autonomous")
public class RedAuto2Stone extends Robot {

    VuforiaStuff.skystonePos pos;
    int stoneDiff;
    int stoneDiff2;

    double driftAdjustment = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, true);
        intake.on();
        liftSystem.hLift.setPower(-.3);
        switch (pos) {
            case RIGHT:
                driveToPoint3(8, 42, 0, .6, 5);
                stoneDiff = 0;
                stoneDiff2 = 0;
                break;
            case CENTER:
                driveToPoint3(0, 42, 0, .4, 1);
                stoneDiff = 7;
                stoneDiff2 = 8;
                break;
            case LEFT:
                driveToPoint3(-8, 42, 0, .5, 6);
                //driveToPoint(-10, 9, 0, 9);
                //driveToPoint(0, 33, 0, 9);
                stoneDiff = 17;
                stoneDiff2 = 17;
                break;
        }
        driveToPoint3(0, -17, 0, .7, 0);
        liftSystem.grabStone();
        liftSystem.hLift.setPower(0);
        intake.off();
        turn_to_heading(270 + driftAdjustment, -25);  // put x inches in next statement to account for x encoder turning
        liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
        driveToPoint3(0, -(72 + stoneDiff), 270 + driftAdjustment, 1,0);
        turn_to_heading(180 + driftAdjustment, -25);
        liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDINGFAR;
        grabbers.ready();
        driveToPoint3(0, -9, 180, .4, 0);
        grabbers.down();
        sleep(300);
        driveToPoint(0, 16, 180 + driftAdjustment, 3);
        turn_to_heading(265, 15);
        liftSystem.dropStone();
        driveToPoint(0, -10, 270, 2.0);
        liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
        grabbers.up();
        sleep(200);
        if (pos == VuforiaStuff.skystonePos.LEFT) {
            driveToPoint3(0, 84 + stoneDiff2, 270 + driftAdjustment, 1, 0);
            intake.on();
            liftSystem.hLift.setPower(-.3);
            turn_to_heading(315, -60);
            driveToPoint3(12, 12, 315, .4, 1);
            driveToPoint3(0, -22, 315, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(270 + driftAdjustment, -60);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3(0, -(70 + stoneDiff2), 270 + driftAdjustment, 1, 0);
            liftSystem.dropStone();
            liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
            driveToPoint3(3, 40, 270 + driftAdjustment, 1, 1);
        } else {
            driveToPoint3(0, 92 + stoneDiff2, 270 + driftAdjustment, 1,0);
            intake.on();
            liftSystem.hLift.setPower(-.3);
            turn_to_heading(0, -25);
            driveToPoint3(0, 20, 0, .35, 0);
            driveToPoint3(0, -22, 0, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(270 + driftAdjustment, -25);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3(0, -(92 + stoneDiff2), 270 + driftAdjustment/2, 1, 0);
            liftSystem.dropStone();
            liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
            driveToPoint3(0, 43, 270, 1, 1);
        }
        liftSystem.hLift.setPower(-.2);
        sleep(300);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}