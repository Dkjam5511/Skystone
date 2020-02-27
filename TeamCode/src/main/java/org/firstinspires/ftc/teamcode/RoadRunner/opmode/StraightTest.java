package org.firstinspires.ftc.teamcode.RoadRunner.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(0,0,0), SampleMecanumDriveBase.constraints)
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
