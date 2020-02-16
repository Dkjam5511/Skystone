package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.GlobalPositions;

import java.util.ArrayList;
import java.util.LinkedList;

@Autonomous(name = "4 STONE", group = "Autonomous")
public class BlueAuto4Stone extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        localizer.setCurrentPose(new Pose2d(0, 0, Math.toRadians(0)));

        ArrayList<Pose2d> stonePositions = new ArrayList<Pose2d>() {
            {
                add(0, new Pose2d(31, 15, Math.toRadians(270))); //left
                add(1, new Pose2d(31, 6, Math.toRadians(270))); //center
                add(2, new Pose2d(31, -2, Math.toRadians(270))); //right
                add(3, new Pose2d(31, -10, Math.toRadians(270))); //4th
                add(4, new Pose2d(31, -18, Math.toRadians(270))); //5th
                add(5, new Pose2d(31, -26, Math.toRadians(270))); //6th
                add(6, new Pose2d());
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

        sideGrabbers.openLeftClaw();
        sideGrabbers.lowerLeftClaw();

        driveToPoint4(new ArrayList<Pose2d>() {
            {
                add(new Pose2d(8, 0, Math.toRadians(0)));
                add(stoneOrder.get(0));
            }
        });

        sideGrabbers.clampLeftClaw();
        sleep(500);

        servoEventManager.addEvent(new ServoEvent(sideGrabbers.getLeftClawPivot(), GlobalPositions.LEFT_CLAW_PIVOT_UP, new Vector2d(31, stoneOrder.get(0).getY())));

        driveToPoint4(new ArrayList<Pose2d>() {
            {
                add(new Pose2d(27, stoneOrder.get(0).getY(), Math.toRadians(270)));
                add(new Pose2d(27, 84, Math.toRadians(270)));
                add(new Pose2d(32.5, 84, Math.toRadians(270)));
            }
        });

        stoneOrder.remove(0);


        for (int i = 0; i < stoneOrder.size(); i++) {
            int finalI = i;

            sideGrabbers.lowerLeftClaw();
            sleep(200);
            sideGrabbers.openLeftClaw();
            sleep(1000);
            sideGrabbers.raiseLeftClaw();
            sleep(500);
            sideGrabbers.clampLeftClaw();

            servoEventManager.addEvent(new ServoEvent(sideGrabbers.getLeftClawPivot(), GlobalPositions.LEFT_CLAW_PIVOT_DOWN, new Vector2d(25, 14)));
            servoEventManager.addEvent(new ServoEvent(sideGrabbers.getLeftClaw(), GlobalPositions.LEFT_CLAW_OPEN, new Vector2d(25, 14)));

            driveToPoint4(new ArrayList<Pose2d>() {
                {
                    add(new Pose2d(27, 70, Math.toRadians(270)));
                    add(new Pose2d(27, stoneOrder.get(finalI).getY(), Math.toRadians(270)));
                    add(stoneOrder.get(finalI));
                }
            });

            sideGrabbers.clampLeftClaw();
            sleep(500);

            servoEventManager.addEvent(new ServoEvent(sideGrabbers.getLeftClawPivot(), GlobalPositions.LEFT_CLAW_PIVOT_UP, new Vector2d(31, stoneOrder.get(finalI).getY())));

            driveToPoint4(new ArrayList<Pose2d>() {
                {
                    add(new Pose2d(27, stoneOrder.get(finalI).getY(), Math.toRadians(270)));
                    add(new Pose2d(27, 84, Math.toRadians(270)));
                    add(new Pose2d(32.5, 84, Math.toRadians(270)));
                }
            });
        }
    }
}
