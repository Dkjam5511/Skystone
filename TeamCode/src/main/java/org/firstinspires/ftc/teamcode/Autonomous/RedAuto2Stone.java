package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Red Auto 2 Stone", group = "Autonomous")
public class RedAuto2Stone extends Robot {

    VuforiaStuff.skystonePos pos;
    double stoneDiff;
    double stoneDiff2;

    double driftAdjustment = -2;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, true);
        intake.onSlow();
        switch (pos) {
            case RIGHT:
                driveToPoint3(8.5, 30, 0, .7, 5);
                stoneDiff = 0;
                stoneDiff2 = 0;
                break;
            case CENTER:
                driveToPoint3(0, 30, 0, .7, 1);
                stoneDiff = 8;
                stoneDiff2 = 8;
                break;
            case LEFT:
                driveToPoint3(-8.5, 30, 0, .7, 6);
                stoneDiff = 13;
                stoneDiff2 = 7;
                break;
        }
        driveToPoint3Intake(0, 12, 0, .25, 6, 7);
        driveToPoint3(0, -18, 0, .8, 0);
        liftSystem.grabStone();
        intake.off();
        turn_to_heading(270 /*+ driftAdjustment*/, -25);  // put x inches in next statement to account for x encoder turning
        liftSystem.hLift.setPosition(1);
        driveToPoint3(0, -(72 + stoneDiff), 270 + driftAdjustment, 1,0);
        liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
        turn_to_heading(180, -25);
        grabbers.ready();
        driveToPoint3Grabbers(0, -12, 180 + driftAdjustment, .3, 0, 1);
        grabbers.down();
        driveToPoint(0, 13.5, 180 + driftAdjustment, 3);
        turn_to_heading(265 + driftAdjustment, 15);
        liftSystem.dropStone();
        driveToPoint(0, -15, 270, 2.0);
        liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
        liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        grabbers.up();
        sleep(200);
        if (pos == VuforiaStuff.skystonePos.LEFT) {
            driveToPoint3(0, 94 + stoneDiff2, 270 + driftAdjustment, 1, 0);
            intake.onSlow();
            //liftSystem.hLift.setPower(-.3);
            turn_to_heading(315, -20);
            driveToPoint3Intake(12, 12, 315, .4, 1, 2);
            driveToPoint3(0, -22, 315, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(270 + driftAdjustment, -20);
            liftSystem.hLift.setPosition(.75);
            driveToPoint3SpinStone(0, -(75 + stoneDiff2), 270 + driftAdjustment, 1, 0, 30);
            //liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            //sleep(500);
            liftSystem.dropStone();
            //sleep(300);
           // liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            driveToPoint3(3, 40, 270 + driftAdjustment, 1, 1);
        } else {
            driveToPoint3(0, 93 + stoneDiff2, 270 + driftAdjustment, 1, 0);
            intake.onSlow();
            //liftSystem.hLift.setPower(-.3);
            turn_to_heading(0, -25);
            driveToPoint3Intake(0, 20, 0, .3, 0, 6);
            driveToPoint3(0, -19, 0, .8, 0);
            liftSystem.grabStone();
            turn_to_heading(270 + driftAdjustment, -25);
            liftSystem.hLift.setPosition(.75);
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint3SpinStone(0, -(95 + stoneDiff2), 270 + driftAdjustment, 1, 0, 30);
            //liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
            //sleep(500);
            liftSystem.dropStone();
            //liftSystem.hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            driveToPoint3(0, 44, 270, 1, 1);
        }

        //liftSystem.hLift.setPower(-.2);
        sleep(300);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}