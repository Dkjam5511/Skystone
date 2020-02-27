package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;

import java.util.ArrayList;

@Autonomous(name = "Intake 4 STONE", group = "Autonomous")
public class BlueAutoIntake4Stone extends Robot {

    VuforiaStuff.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            roboInit();
            /*
            localizer.setCurrentPose(new Pose2d(0, 0, Math.toRadians(0)));

            double stoneX = 36;
            double travelX = 25;
            double foundationX = 30;
            double foundationY1 = 76;
            double foundationY2 = 60;
            double stonePickupHeading = 300;

            ArrayList<Pose2d> stonePositions = new ArrayList<Pose2d>() {
                {
                    add(0, new Pose2d(stoneX, 10, Math.toRadians(0))); //left
                    add(1, new Pose2d(stoneX, 2, Math.toRadians(stonePickupHeading))); //center
                    add(2, new Pose2d(stoneX, -6, Math.toRadians(stonePickupHeading))); //right
                    add(3, new Pose2d(stoneX, -14, Math.toRadians(stonePickupHeading))); //4th
                    add(4, new Pose2d(stoneX, -22, Math.toRadians(stonePickupHeading))); //5th
                    add(5, new Pose2d(stoneX, -28, Math.toRadians(stonePickupHeading))); //6th
                }
            };

            ArrayList<Pose2d> stoneOrder = new ArrayList<>(7);

            VuforiaStuff.skystonePos pos = vuforiaStuff.vuforiascan(false, false);

            telemetry.addData("POS: ", pos);
            telemetry.update();

            switch (pos) {
                case LEFT:
                    stoneOrder.add(0, stonePositions.get(0));
                    stoneOrder.add(1, stonePositions.get(3));
                    stoneOrder.add(2, stonePositions.get(1));
                    stoneOrder.add(3, stonePositions.get(2));
                    stoneOrder.add(4, stonePositions.get(4));
                    stoneOrder.add(5, stonePositions.get(5));
                    break;
                case CENTER:
                    stoneOrder.add(0, stonePositions.get(1));
                    stoneOrder.add(1, stonePositions.get(4));
                    stoneOrder.add(2, stonePositions.get(0));
                    stoneOrder.add(3, stonePositions.get(2));
                    stoneOrder.add(4, stonePositions.get(3));
                    stoneOrder.add(5, stonePositions.get(5));
                    break;
                case RIGHT:
                    stoneOrder.add(0, stonePositions.get(2));
                    stoneOrder.add(1, stonePositions.get(5));
                    stoneOrder.add(2, stonePositions.get(0));
                    stoneOrder.add(3, stonePositions.get(1));
                    stoneOrder.add(4, stonePositions.get(3));
                    stoneOrder.add(5, stonePositions.get(4));
                    break;
            }

            intake.onSlow();

            double targetStoneY = stoneOrder.get(0).getY();
            double targetStoneX = stoneOrder.get(0).getX();

            servoEventManager.addEvent(new ServoEvent(stoneX - 3, 0, 0, 0, ServoEvent.SE_INTAKE_ON));

            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    add(new DriveParameters(new Pose2d(targetStoneX, targetStoneY, Math.toRadians(0)), 1, 1, 1.5, 3, 10, 0, 0));
                }
            });

            stoneOrder.remove(0);

// Drive to foundation, pull and turn foundation, push foundation
            servoEventManager.addEvent(new ServoEvent(0, 15, 0, 0, ServoEvent.SE_GRAB_STONE));
            servoEventManager.addEvent(new ServoEvent(0, 25, 0, 0, ServoEvent.SE_INTAKE_OFF));
            servoEventManager.addEvent(new ServoEvent(0, 25, 0, 0, ServoEvent.SE_EXTEND_HLIFT_FAR));
            servoEventManager.addEvent(new ServoEvent(0, 0, 0, 0, ServoEvent.SE_GRABBERS_READY));
            servoEventManager.addEvent(new ServoEvent(foundationX + 5, 0, 1, 0, ServoEvent.SE_GRABBERS_DOWN));
            servoEventManager.addEvent(new ServoEvent(0, 0, 1, 0, ServoEvent.SE_ROTATE_STONE_180));
            servoEventManager.addEvent(new ServoEvent(0, foundationY1 - 5, 2, 0, ServoEvent.SE_DROP_STONE));
            servoEventManager.addEvent(new ServoEvent(0, foundationY1 - 15, 2, 0, ServoEvent.SE_GRABBERS_UP));

            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    //add(new Pose2d(25, stoneOrder.get(0).getY(), Math.toRadians(270)));
                    add(new DriveParameters(new Pose2d(travelX, foundationY1, Math.toRadians(270)), 1, 1, 1.5, 3, 10, 0, 0));
                    add(new DriveParameters(new Pose2d(foundationX + 8, foundationY1, Math.toRadians(180)), 1, 1, 1.5, 3, 10, 0, 0));
                    add(new DriveParameters(new Pose2d(travelX, foundationY1 - 20, Math.toRadians(270)), 1, 1, 1.5, 3, 10, 0, 0));
                }
            });

            //grabbers.up();
            //sleep(300);


            for (int i = 0; i < 2; i++) {
                int finalI = i;

//            if (finalI > 0) {
//                localizer.artificialAdjust(-1.5, -2);
//            }


// Drive to stones 2 and 3

                driveToPointTest2(new ArrayList<DriveParameters>() {
                    {
                        //add(new Pose2d(25, 84, Math.toRadians(270)));
                        add(new DriveParameters(new Pose2d(travelX, stoneOrder.get(finalI).getY(), Math.toRadians(270)), 1, 1, 1.5, 3, 10, 0, 0));
                        add(new DriveParameters(stoneOrder.get(finalI), 1, 1, 1.5, 3, 10, 0, 0));
                    }
                });

                intake.on();
                driveToPoint3(0, 9, 45, .3, 0);
                liftSystem.grabStone();
                intake.off();

// Drive to foundation and drop
                driveToPointTest2(new ArrayList<DriveParameters>() {
                    {
                        //add(new Pose2d(25, stoneOrder.get(finalI).getY(), Math.toRadians(270)), );
                        add(new DriveParameters(new Pose2d(travelX, foundationY2, Math.toRadians(270)), 1, 1, 1.5, 3, 10, 0, 0));
                        add(new DriveParameters(new Pose2d(foundationX, foundationY2, Math.toRadians(270)), 1, 1, 1.5, 3, 10, 0, 0));
                    }
                });
            }

// Park
            driveToPointTest2(new ArrayList<DriveParameters>() {
                {
                    add(new DriveParameters(new Pose2d(travelX, 28, Math.toRadians(270)), 2.5, 1, 4, 4, 5, 0, 0));
                }
            }); */

        } catch (Exception e) {
            throw e;
        } finally {
            DbgLog.close();
            driveTrain.applyPower(0, 0, 0, 0);
        }

    }
}


