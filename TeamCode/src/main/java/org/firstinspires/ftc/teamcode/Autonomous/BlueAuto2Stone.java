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

    double driftAdjustment = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, false);
        intake.onSlow();
        switch (pos) {
            case LEFT:
                driveToPoint3(-8, 30, 0, .7, 5);
                stoneDiff = 0;
                stoneDiff2 = 0;
                break;
            case CENTER:
                driveToPoint3(0, 30, 0, .7, 1);
                stoneDiff = 8;
                stoneDiff2 = 8;
                break;
            case RIGHT:
                driveToPoint3(8.5, 30, 0, .7, 6);
                stoneDiff = 16;
                stoneDiff2 = 6;
                break;
        }
        driveToPoint3Intake(0, 12, 0, .25, 6, 7);
        driveToPoint3(0, -18, 0, .8, 0);
        liftSystem.grabStone();
        intake.off();
        turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
        liftSystem.hLift.setPosition(1);
        driveToPoint3(0, -(70 + stoneDiff), 90 /*+ driftAdjustment*/, 1, 0);
        liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
        turn_to_heading(180, -25); // was 50
        grabbers.ready();
        driveToPoint3Grabbers(0, -12, 180 /*+ driftAdjustment*/, .3, 0, 1);
        grabbers.down();
        driveToPoint(0, 16, 180 /*+ driftAdjustment */, 3);
        turn_to_heading(95 /*+ driftAdjustment*/, 15);
        liftSystem.dropStone();
        driveToPoint(0, -12, 90 /*+ driftAdjustment*/, 2.0);
        liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
        liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        grabbers.up();
        sleep(200);
        if (pos == VuforiaStuff.skystonePos.RIGHT) {
            //driveToPoint3(0, 44, 90, .7, 1);
            driveToPoint3(0, 94 + stoneDiff2, 90 + driftAdjustment, 1, 0);
            intake.onSlow();
            //liftSystem.hLift.setPower(-.3);
            turn_to_heading(45 + driftAdjustment, -20);
            driveToPoint3Intake(-13, 13, 45 + driftAdjustment, .4, 1, 6);
            driveToPoint3(0, -26, 45 + driftAdjustment, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(90 + driftAdjustment, -20);
            liftSystem.hLift.setPosition(.75);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3SpinStone(0, -(75 + stoneDiff2), 90 + driftAdjustment, 1, 0, 30);
            //liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            //sleep(500);
            liftSystem.dropStone();
            //liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            driveToPoint3(0, 43, 90 + driftAdjustment, 1, 1);
        } else {
            driveToPoint3(0, 95 + stoneDiff2, 90 + driftAdjustment, 1, 0);
            intake.onSlow();
            //liftSystem.hLift.setPower(-.3);
            turn_to_heading(0 + driftAdjustment, -25);
            driveToPoint3Intake(0, 19, 0 + driftAdjustment, .3, 0, 8);
            driveToPoint3(0, -21, 0 + driftAdjustment, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(90 + driftAdjustment, -25);
            liftSystem.hLift.setPosition(.75);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3SpinStone(0, -(92 + stoneDiff2), 90 + driftAdjustment, 1, 0, 30);
            //liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            //sleep(500);
            liftSystem.dropStone();
            //liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
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