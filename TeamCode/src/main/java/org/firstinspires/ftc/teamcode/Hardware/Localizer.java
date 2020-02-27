package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.RoadRunner.mecanum.SampleMecanumDriveREVOptimized;

public class Localizer {
    public SampleMecanumDriveBase drive;
    public IMU imu;
    public double XWallDiff = 0;

    public Localizer(BNO055IMU imu, HardwareMap hardwareMap){
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        this.imu = new IMU(imu);
    }

    public Pose2d getCurrentPose(){
        drive.update();
        return drive.getPoseEstimate();
    }

    public void setCurrentPose(Pose2d currentPose){
        this.drive.setPoseEstimate(currentPose);
    }

    public double getCurrentHeadingDegrees() {
        drive.update();
        return 360 - Math.toDegrees(drive.getPoseEstimate().getHeading());
    }

    public double getCurrentHeadingRadians() {
        drive.update();
        return drive.getPoseEstimate().getHeading();
    }

    public double getXPosition(){
        drive.update();
        return drive.getPoseEstimate().getX() + XWallDiff;
    }

    public double getYPosition(){
        drive.update();
        return drive.getPoseEstimate().getY();
    }

    public double headingAdjustment(double targetHeading){
        double adjustment;
        double currentHeading;
        double degreesOff;
        boolean goRight;

        currentHeading = getCurrentHeadingDegrees();

        goRight = targetHeading > currentHeading;
        degreesOff = Math.abs(targetHeading - currentHeading);

        if (degreesOff > 180) {
            goRight = !goRight;
            degreesOff = 360 - degreesOff;
        }

        if (degreesOff < .3) {
            adjustment = 0;
        } else {
            adjustment = (Math.pow((degreesOff + 2) / 5, 2) + 2) / 100;
        }

        if (goRight) {
            adjustment = -adjustment;
        }
        return adjustment;
    }

    public void artificialAdjust(double heading, double x){
        Pose2d pose = drive.getPoseEstimate();

        drive.setPoseEstimate(new Pose2d(pose.getX() - x, pose.getY(), pose.getHeading() + Math.toRadians(heading)));

    }

}
