package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Blue Auto 2 Stone", group = "Autonomous")
public class BlueAuto2Stone extends Robot {

    VuforiaStuff.skystonePos pos;
    double stoneDiff;
    double stoneDiff2;

    double driftAdjustment = -2;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, false);
        intake.on();
        //liftSystem.hLift.setPower(-.3);
        switch (pos) {
            case LEFT:
                driveToPoint3(-8, 30, 0, .7, 5);
                stoneDiff = 0;
                stoneDiff2 = 0;
                break;
            case CENTER:
                driveToPoint3(0, 30, 0, .7, 1);
                stoneDiff = 8;
                stoneDiff2 = 6;
                break;
            case RIGHT:
                //driveToPoint3(10, 9, 0, 1,1);
                //driveToPoint3(0, 33, 0, 1,1);
                driveToPoint3(7, 30, 0, .7, 6);
                stoneDiff = 13;
                stoneDiff2 = 7.5;
                break;
        }
        driveToPoint3(0, 12, 0, .35, 1);
        driveToPoint3(0, -20, 0, .8, 0);
        liftSystem.grabStone();
        intake.off();
        turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
        liftSystem.hLift.setPosition(1);
        driveToPoint3(0, -(72 + stoneDiff), 90 + driftAdjustment, 1, 0);
        liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
        turn_to_heading(180, -25); // was 50
        grabbers.ready();
        driveToPoint3(0, -11, 180 + driftAdjustment, .4, 0);
        grabbers.down();
        sleep(300);
        driveToPoint(0, 17, 180 + driftAdjustment*2, 3);
        turn_to_heading(95 + driftAdjustment, 15);
        liftSystem.dropStone();
        driveToPoint(0, -12, 90 + driftAdjustment, 2.0);
        liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
        liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        grabbers.up();
        sleep(200);
        if (pos == VuforiaStuff.skystonePos.RIGHT) {
            //driveToPoint3(0, 44, 90, .7, 1);
            driveToPoint3(0, 94 + stoneDiff2, 90, 1, 0);
            intake.on();
            //liftSystem.hLift.setPower(-.3);
            turn_to_heading(45, -40);
            driveToPoint3(-13, 13, 45, .4, 1);
            driveToPoint3(0, -25, 45, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(90 + driftAdjustment, -40);
            liftSystem.hLift.setPosition(.75);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3(0, -(75 + stoneDiff2), 90 + driftAdjustment, 1, 0);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            sleep(500);
            liftSystem.dropStone();
            liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            driveToPoint3(0, 43, 90 + driftAdjustment, 1, 1);
        } else {
            driveToPoint3(0, 95 + stoneDiff2, 90, 1, 0);
            intake.on();
            //liftSystem.hLift.setPower(-.3);
            turn_to_heading(0, -25);
            driveToPoint3(0, 19, 0, .35, 0);
            driveToPoint3(0, -20, 0, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(90, -25);
            liftSystem.hLift.setPosition(.75);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3(0, -(92 + stoneDiff2), 90, 1, 0);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            sleep(500);
            liftSystem.dropStone();
            liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            driveToPoint3(0, 43, 90 + driftAdjustment, 1, 1);
        }
        //liftSystem.hLift.setPower(-.2);
        sleep(300);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}