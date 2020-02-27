package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalPositions;

@Autonomous(name = "Blue Auto 2 Stone", group = "Autonomous")
public class BlueAuto2Stone extends Robot {

    VuforiaStuff.skystonePos pos;
    double stoneDiff;
    double stoneDiff2;

    double driftAdjustment = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            roboInit();
            pos = vuforiaStuff.vuforiascan(false, false);
            localizer.setCurrentPose(new Pose2d(0, 0, Math.toRadians(0)));
            intake.onSlow();
            switch (pos) {
                case LEFT:
                    driveToPoint3(-8, 30, 0, .7, 5);
                    stoneDiff = 0;
                    stoneDiff2 = 0;
                    break;
                case CENTER:
                    driveToPoint3(3, 30, 0, .7, 1);
                    stoneDiff = 9;
                    stoneDiff2 = 8;
                    break;
                case RIGHT:
                    driveToPoint3(8.5, 30, 0, .7, 6);
                    stoneDiff = 16;
                    stoneDiff2 = 8;
                    break;
            }
            driveToPoint3Intake(0, 12, 0, .25, 6, 4);
            driveToPoint3(0, -18, 0, .8, 0);
            liftSystem.grabStone();
            intake.off();
            turn_to_heading(90, -25);  // was 50;  put x inches in next statement to account for x encoder turning
            liftSystem.setHLiftPos(1);
            driveToPoint3(0, -(70 + stoneDiff), 90 /*+ driftAdjustment*/, 1, 0);
            liftSystem.setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_UP);
            turn_to_heading(180, -25); // was 50
            grabbers.ready();
            driveToPoint3Grabbers(0, -12, 180 /*+ driftAdjustment*/, .3, 0, 1);
            grabbers.down();
            driveToPoint(0, 16, 180 /*+ driftAdjustment */, 3);
            turn_to_heading(95 /*+ driftAdjustment*/, 15);
            liftSystem.dropStone();
            driveToPoint(0, -12, 90 /*+ driftAdjustment*/, 2.0);
            liftSystem.setHLiftPos(GlobalPositions.MIN_HLIFT_POS);
            liftSystem.setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_DOWN);
            grabbers.up();
            sleep(200);
            if (pos == VuforiaStuff.skystonePos.RIGHT) {
                driveToPoint3(0, 91.5 + stoneDiff2, 90 + driftAdjustment, 1, 0);
                intake.onSlow();
                turn_to_heading(45 + driftAdjustment, -20);
                driveToPoint3Intake(-13, 13, 45 + driftAdjustment, .4, 1, 4);
                driveToPoint3(0, -26, 45 + driftAdjustment, .8, 0);
                liftSystem.grabStone();
                turn_to_heading(90 + driftAdjustment, -20);
                liftSystem.setHLiftPos(.75);
                driveToPoint3SpinStone(0, -(71 + stoneDiff2), 90 + driftAdjustment, 1, 0, 30);
            } else {
                driveToPoint3(0, 95 + stoneDiff2, 90 + driftAdjustment, 1, 0);
                intake.onSlow();
                turn_to_heading(0 + driftAdjustment, -25);
                driveToPoint3Intake(0, 20, 0, .3, 0, 8);
                driveToPoint3(0, -21, 0 + driftAdjustment, .8, 0);
                liftSystem.grabStone();
                turn_to_heading(90 + driftAdjustment, -25);
                liftSystem.setHLiftPos(.75);
                driveToPoint3SpinStone(0, -(90 + stoneDiff2), 90 + driftAdjustment, 1, 0, 30);
            }
            liftSystem.dropStone();
            liftSystem.setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_DOWN);
            driveToPoint3(0, 43, 90 + driftAdjustment, 1, 1);

        } catch (Exception e) {
            throw e;
        } finally {
            DbgLog.close();
            driveTrain.applyPower(0, 0, 0, 0);
            intake.off();
        }
    }
}