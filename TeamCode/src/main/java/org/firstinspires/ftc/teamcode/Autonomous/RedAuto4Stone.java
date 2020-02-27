package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

@Autonomous(name = "4 STONE Red Auto", group = "Autonomous")
public class RedAuto4Stone extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            roboInit();
            localizer.setCurrentPose(new Pose2d(0, 0, Math.toRadians(0)));

            double stoneX = 29;
            double travelX = 24;
            double foundationX = 29;
            double foundationY1 = -97;
            double foundationY2 = -86;
            double stonePickupHeading = 90;

            ArrayList<Pose2d> stonePositions = new ArrayList<Pose2d>() {
                {
                    add(0, new Pose2d(stoneX, -26, Math.toRadians(stonePickupHeading))); //left
                    add(1, new Pose2d(stoneX, -18, Math.toRadians(stonePickupHeading))); //center
                    add(2, new Pose2d(stoneX, -10, Math.toRadians(stonePickupHeading))); //right
                    add(3, new Pose2d(stoneX, -2, Math.toRadians(stonePickupHeading))); //4th
                    add(4, new Pose2d(stoneX, 6, Math.toRadians(stonePickupHeading))); //5th
                    add(5, new Pose2d(stoneX, 14, Math.toRadians(stonePickupHeading))); //6th
                }
            };

            ArrayList<Pose2d> stoneOrder = new ArrayList<>(7);

            VuforiaStuff.skystonePos pos = vuforiaStuff.vuforiascan(false, false);

            telemetry.addData("POS: ", pos);
            telemetry.update();

            switch (pos) {
                case LEFT:
                    stoneOrder.add(0, stonePositions.get(3));
                    stoneOrder.add(1, stonePositions.get(0));
                    stoneOrder.add(2, stonePositions.get(1));
                    stoneOrder.add(3, stonePositions.get(2));
                    stoneOrder.add(4, stonePositions.get(4));
                    stoneOrder.add(5, stonePositions.get(5));
                    break;
                case CENTER:
                    stoneOrder.add(0, stonePositions.get(4));
                    stoneOrder.add(1, stonePositions.get(1));
                    stoneOrder.add(2, stonePositions.get(0));
                    stoneOrder.add(3, stonePositions.get(2));
                    stoneOrder.add(4, stonePositions.get(3));
                    stoneOrder.add(5, stonePositions.get(5));
                    break;
                case RIGHT:
                    stoneOrder.add(0, stonePositions.get(5));
                    stoneOrder.add(1, stonePositions.get(2));
                    stoneOrder.add(2, stonePositions.get(0));
                    stoneOrder.add(3, stonePositions.get(1));
                    stoneOrder.add(4, stonePositions.get(3));
                    stoneOrder.add(5, stonePositions.get(4));
                    break;
            }

            sideGrabbers.openLeftClaw();
            sideGrabbers.lowerLeftClaw();

            servoEventManager.addEvent(new ServoEvent(stoneX - 2, 0, 1, 200, ServoEvent.SE_RIGHT_CLAW_CLOSE));

            double targetStoneY = stoneOrder.get(0).getY();
            double targetStoneX = stoneOrder.get(0).getX();
// Pick up first stone
            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    add(new DriveParameters(new Pose2d(14, targetStoneY + 3, Math.toRadians(0)), 1, 1, 8, 12, 5, 2, 7));
                    add(new DriveParameters(new Pose2d(targetStoneX + 1, targetStoneY, Math.toRadians(stonePickupHeading)), 1, 1, 1.5, 5, 5, 0, 9));
                }
            });

            sideGrabbers.raiseLeftClaw();
            sleep(200);

// Drive to foundation and drop
            servoEventManager.addEvent(new ServoEvent(0, -60, 0, 0, ServoEvent.SE_RIGHT_CLAW_PIVOT_PLACE_HIGH));
            servoEventManager.addEvent(new ServoEvent(foundationX - 2, foundationY1 + 5, 1, 0, ServoEvent.SE_RIGHT_CLAW_OPEN));

            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    //add(new Pose2d(25, stoneOrder.get(0).getY(), Math.toRadians(stonePickupHeading)));
                    add(new DriveParameters(new Pose2d(travelX, foundationY1, Math.toRadians(stonePickupHeading)), 8, 1, 10, 30, 5, 0, 0));
                    add(new DriveParameters(new Pose2d(foundationX, foundationY1, Math.toRadians(stonePickupHeading)), 3, 1, 3, 5, 5, 2, 8));
                }
            });

            stoneOrder.remove(0);

//Beginning of For Loop
            for (int i = 0; i < 3; i++) {

                localizer.artificialAdjust(-0, -0);

                double targetStoneY2 = stoneOrder.get(i).getY();

                servoEventManager.addEvent(new ServoEvent(0, foundationY2 + 5, 0, 0, ServoEvent.SE_RIGHT_CLAW_RAISE));
                servoEventManager.addEvent(new ServoEvent(0, foundationY2 + 5, 0, 0, ServoEvent.SE_RIGHT_CLAW_CLOSE));
                servoEventManager.addEvent(new ServoEvent(0, 40, 0, 0, ServoEvent.SE_RIGHT_CLAW_OPEN));
                servoEventManager.addEvent(new ServoEvent(0, 37, 0, 0, ServoEvent.SE_RIGHT_CLAW_LOWER));
                servoEventManager.addEvent(new ServoEvent(stoneX - 2, targetStoneY2 + 2, 1, 300, ServoEvent.SE_RIGHT_CLAW_CLOSE));

// Drive to stone 2, 3, or 4

                driveToPointTest2(new ArrayList<DriveParameters>() {
                    {
                        add(new DriveParameters(new Pose2d(travelX, targetStoneY2 - 8, Math.toRadians(stonePickupHeading)), 10, 1, 5, 20, 5, 0, 0));
                        add(new DriveParameters(new Pose2d(stoneX + 1, targetStoneY2, Math.toRadians(stonePickupHeading)), 1, 1, 2, 2, 5, 0, 8));
                    }
                });

                sideGrabbers.raiseLeftClaw();

// Drive to foundation and drop

                double foundationXShift = 0; //Used to dtermine which nubs to place the stones in
                double foundationY = foundationY1; // Used to determine whcih of the two Y pos depths to place the stone at

                switch (i) {
                    case 0:
                        foundationXShift = 3;
                        foundationY = foundationY1;
                        break;
                    case 1:
                        foundationXShift = 0;
                        foundationY = foundationY2;
                        break;
                    case 2:
                        foundationXShift = 3;
                        foundationY = foundationY2 - 2;
                        break;
                }

                servoEventManager.addEvent(new ServoEvent(0, 60, 0, 0, ServoEvent.SE_RIGHT_CLAW_PIVOT_PLACE_HIGH));
                servoEventManager.addEvent(new ServoEvent(foundationX - 5.5, foundationY + 7, 1, 0, ServoEvent.SE_RIGHT_CLAW_PIVOT_PLACE));
                servoEventManager.addEvent(new ServoEvent(foundationX - 5, foundationY + 6.5, 1, 0, ServoEvent.SE_RIGHT_CLAW_OPEN));

                double finalFoundationXShift = foundationXShift;
                double finalFoundationY = foundationY;
                driveToPointTest2(new ArrayList<DriveParameters>() {
                    {
                        add(new DriveParameters(new Pose2d(travelX, finalFoundationY, Math.toRadians(stonePickupHeading)), 10, 1, 5, 20, 5, 0, 0));
                        add(new DriveParameters(new Pose2d(foundationX - finalFoundationXShift, finalFoundationY, Math.toRadians(stonePickupHeading)), 3, 1, 3, 5, 3, 2, 8));
                    }
                });

            }  // end of loop

// Lift claw and pull foundation
            sideGrabbers.raiseLeftClaw();
            grabbers.ready();

            servoEventManager.addEvent(new ServoEvent(foundationX, 0, 0, 0, ServoEvent.SE_RIGHT_CLAW_RAISE));
            servoEventManager.addEvent(new ServoEvent(foundationX, 0, 0, 0, ServoEvent.SE_RIGHT_CLAW_CLOSE));
            servoEventManager.addEvent(new ServoEvent(foundationX + 4, 0, 0, 0, ServoEvent.SE_GRABBERS_DOWN));


            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    add(new DriveParameters(new Pose2d(foundationX + 6, foundationY2 + 2, Math.toRadians(180)), 1, 1, 2, 2, 5, 6, 0));
                    //add(new DriveParameters(new Pose2d(travelX, foundationY2 - 15, Math.toRadians(180)),2,5, 5, 5));

                }
            });

/*
        driveTrain.applyPower(1, 1, 1, 1);
        sleep(400);

        driveTrain.applyPower(-.2, 1, .7, 1);
        sleep(1200);

        driveToPoint(0, 24, 180, 3);
        turn_to_heading(95 *//*+ driftAdjustment*//*, 15);
        driveToPoint(0, -8, 90 *//*+ driftAdjustment*//*, 2.0);
             */

            pullFoundation(30, -60, true);
            pushFoundation(-65);

            grabbers.up();
            sleep(200);

            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    //add(new DriveParameters(new Pose2d(travelX, foundationY2, Math.toRadians(stonePickupHeading)),1,10, 10, 5,0));
                    add(new DriveParameters(new Pose2d(travelX, -28, Math.toRadians(stonePickupHeading)), 3, 1, 4, 4, 5, 0, 0));
                }
            });
        } catch (Exception e){
            throw e;
        } finally {
            DbgLog.close();
            driveTrain.applyPower(0,0,0,0);
        }


    }
}
