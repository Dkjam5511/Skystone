package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;

import kotlin.Unit;

@Autonomous(name = "Roadrunner Test", group = "Tests")
public class RoadRunnerTest extends Robot {

    VuforiaStuff.skystonePos pos;
    Pose2d currentPose = new Pose2d(-33, 63, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        drive.setPoseEstimate(currentPose);
        pos = vuforiaStuff.vuforiascan(false, false);
        intake.on();
        switch (pos) {
            case LEFT:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(currentPose,SampleMecanumDriveBase.slowConstraints)
                                .splineTo(new Pose2d(-20.0, 30.0, Math.toRadians(240)))
                                .forward(18.0)
                                .build()
                );
                currentPose = drive.getPoseEstimate();
                break;
            case CENTER:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.slowConstraints)
                                .splineTo(new Pose2d(-25.0, 33.0, Math.toRadians(240.0)))
                                .forward(18.0)
                                .build()
                );
                currentPose = drive.getPoseEstimate();
                break;
            case RIGHT:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.slowConstraints)
                                .splineTo(new Pose2d(-31.0, 39.0, Math.toRadians(240.0)))
                                .forward(18.0)
                                .build()
                );
                currentPose = drive.getPoseEstimate();
                break;
        }
        drive.setPoseEstimate(currentPose);
        drive.followTrajectorySync(
                drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .back(18.0)

                        .addMarker(2, () -> {
                            liftSystem.grabStone();
                            intake.off();
                            return Unit.INSTANCE;
                        })

                        .reverse()
                        .splineTo(new Pose2d(0.0, 37.0, Math.toRadians(180.0)))

                        .addMarker(2.5, () -> {
                            liftSystem.hLift.setPosition(.7);
                            return Unit.INSTANCE;
                        })

                        .addMarker(3.5, () -> {
                            liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
                            grabbers.ready();
                            return Unit.INSTANCE;
                        })

                        .addMarker(new Vector2d(38.0, 31.0), () -> {
                            grabbers.down();
                            liftSystem.dropStone();
                            return Unit.INSTANCE;
                        })

                        .splineTo(new Pose2d(38.0, 31.0, Math.toRadians(90.0)))

                        .addMarker(new Vector2d(25, 37), () -> {
                            grabbers.up();
                            liftSystem.hLift.setPosition(0);
                            return Unit.INSTANCE;
                        })


                        .reverse()
                        .splineTo(new Pose2d(0.0, 40.0, Math.toRadians(175.0)))
                        .build()
        );
        currentPose = drive.getPoseEstimate();
        intake.on();
        switch (pos) {
            case LEFT:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.slowConstraints)
                                .splineTo(new Pose2d(-52.0, 40.0, Math.toRadians(225.0)))
                                .lineTo(new Vector2d(-52.0, 16.0), new SplineInterpolator(Math.toRadians(225.0), Math.toRadians(225.0)))
                                .build()
                );
                currentPose = drive.getPoseEstimate();
                break;
            case CENTER:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.slowConstraints)
                                .forward(44.0)
                                .splineTo(new Pose2d(-52.0, 40.0, Math.toRadians(225.0)))
                                .lineTo(new Vector2d(-52.0, 16.0), new SplineInterpolator(Math.toRadians(225.0), Math.toRadians(225.0)))
                                .build()
                );
                currentPose = drive.getPoseEstimate();
                break;
            case RIGHT:
                drive.followTrajectorySync(
                        drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.slowConstraints)
                                .forward(54.0)
                                .splineTo(new Pose2d(-64.0, 34.0, Math.toRadians(240.0)))
                                .strafeTo(new Vector2d(-64.0, 16.0))
                                .build()
                );
                currentPose = drive.getPoseEstimate();
                break;
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .setReversed(false)
                        .splineTo(new Pose2d(0.0, 40.0, Math.toRadians(180.0)))
                        .addMarker(2, () -> {
                            liftSystem.grabStone();
                            intake.off();
                            return Unit.INSTANCE;
                        })

                        .addMarker(2.5, () -> {
                            liftSystem.hLift.setPosition(.7);
                            return Unit.INSTANCE;
                        })

                        .lineTo(new Vector2d(50.0, 40.0))
                        .reverse()
                        .lineTo(new Vector2d(0.0, 40.0))
                        .build()
        );
        currentPose = drive.getPoseEstimate();
    }
}
