package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "RR Blue", group = "Autonomous")
@Disabled
public class RrBlue extends Robot {

    VuforiaStuff.skystonePos pos;
    Pose2d currentPose = new Pose2d(-33, 63, Math.toRadians(270));

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        roboInit();
        localizer.drive.setPoseEstimate(currentPose);

        localizer.drive.followTrajectorySync(
        localizer.drive.trajectoryBuilder(currentPose,SampleMecanumDriveBase.constraints)
                .strafeTo(new Vector2d(-15.0,35.0))
                .forward(8)
                .build()
        );

        currentPose = localizer.drive.getPoseEstimate();

        localizer.drive.followTrajectorySync(
                localizer.drive.trajectoryBuilder(currentPose,SampleMecanumDriveBase.slowConstraints)
                    .forward(9)
                .build()
        );
        currentPose = localizer.drive.getPoseEstimate();

        grabbers.ready();

        localizer.drive.followTrajectorySync(
                localizer.drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .back(19)
                        .setReversed(true)
                        .splineTo(new Pose2d(0.0, 37.0, Math.toRadians(180.0)))
                        .splineTo(new Pose2d(50.0, 31.0, Math.toRadians(90.0)))
                        .build()
        );
        currentPose = new Pose2d(50.0, 31.0, Math.toRadians(90.0));
        //grabbers.down();
        sleep(700);

        localizer.drive.followTrajectorySync(
                localizer.drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .setReversed(false)
                        .splineTo(new Pose2d(0.0, 40.0, Math.toRadians(175.0)))
                        .forward(48)
                        .build()
        );
        currentPose = localizer.drive.getPoseEstimate();
        localizer.drive.turnSync(Math.toRadians(45.0));
        currentPose = localizer.drive.getPoseEstimate();
        localizer.drive.followTrajectorySync(
                localizer.drive.trajectoryBuilder(currentPose, SampleMecanumDriveBase.constraints)
                        .strafeTo(new Vector2d(-48.0, 20.0))
                        .build()
        );

         */
    }


}
