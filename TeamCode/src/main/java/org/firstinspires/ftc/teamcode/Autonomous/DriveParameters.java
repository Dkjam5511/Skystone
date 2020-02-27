package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class DriveParameters {

    private Pose2d targetPose;
    private double xCorrectFactor = 1;
    private double yCorrectFactor = 1;
    private double distanceToleranceX;
    private double distanceToleranceY;
    private double headingTolerance;
    private double speedModifier;
    private double pidP;

    public DriveParameters(Pose2d targetPose, double xCorrectFactor, double yCorrectFactor, double distanceToleranceX, double distanceToleranceY, double headingTolerance, double speedModifier, double pidP){
        this.targetPose = targetPose;

        if (xCorrectFactor != 0) {
            this.xCorrectFactor = xCorrectFactor;
        }
        if (yCorrectFactor != 0) {
            this.yCorrectFactor = yCorrectFactor;
        }
        this.distanceToleranceX = distanceToleranceX;
        this.distanceToleranceY = distanceToleranceY;
        this.headingTolerance = Math.toRadians(headingTolerance);
        this.speedModifier = speedModifier;
        this.pidP = pidP;
    }


    public Pose2d getTargetPose() {
        return targetPose;
    }

    public double getxCorrectFactor() {
        return xCorrectFactor;
    }

    public double getyCorrectFactor() {
        return yCorrectFactor;
    }

    public double getDistanceToleranceX() {
        return distanceToleranceX;
    }

    public double getDistanceToleranceY() {
        return distanceToleranceY;
    }

    public double getHeadingTolerance() {
        return headingTolerance;
    }

    public double getSpeedModifier() {
        return speedModifier;
    }

    public double getPidP() {
        return pidP;
    }

}
