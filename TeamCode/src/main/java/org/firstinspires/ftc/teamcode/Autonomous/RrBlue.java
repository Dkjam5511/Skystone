package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;

@Autonomous (name = "RR Blue", group = "Autonomous")
@Disabled
public class RrBlue extends Robot {

    VuforiaStuff.skystonePos pos;
    Pose2d currentPose = new Pose2d(-33, 63, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        drive.setPoseEstimate(currentPose);

        drive.followTrajectorySync(
        drive.trajectoryBuilder(currentPose,SampleMecanumDriveBase.constraints)
                .strafeTo(new Vector2d(-15.0,35.0))
                .forward(8)
                .build()
        );

        currentPose = drive.getPoseEstimate();

        drive.followTrajectorySync(
                drive.trajectoryBuilder(currentPose,SampleMecanumDriveBase.slowConstraints)
                    .forward(9)
                .build()
        );
        currentPose = drive.getPoseEstimate();

        grabbers.ready();

        drive.followTrajectorySync(
                drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .back(19)
                        .setReversed(true)
                        .splineTo(new Pose2d(0.0, 37.0, Math.toRadians(180.0)))
                        .splineTo(new Pose2d(50.0, 31.0, Math.toRadians(90.0)))
                        .build()
        );
        currentPose = new Pose2d(50.0, 31.0, Math.toRadians(90.0));
        //grabbers.down();
        sleep(700);

        drive.followTrajectorySync(
                drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .setReversed(false)
                        .splineTo(new Pose2d(0.0, 40.0, Math.toRadians(175.0)))
                        .forward(48)
                        .build()
        );
        currentPose = drive.getPoseEstimate();
        drive.turnSync(Math.toRadians(45.0));
        currentPose = drive.getPoseEstimate();
        drive.followTrajectorySync(
                drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .strafeTo(new Vector2d(-48.0, 20.0))
                        .build()
        );
    }
}
