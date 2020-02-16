package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Method;

public class ServoEvent {

    private Servo servo;
    private double position;
    private Vector2d pose;

    public ServoEvent(Servo servo, double position, Vector2d pose){
        this.servo = servo;
        this.position = position;
        this.pose = pose;
    }

    public Servo getServo() {
        return servo;
    }

    public double getPosition() {
        return position;
    }

    public Vector2d getPose() {
        return pose;
    }

    public boolean checkXPos (double currentXInches){
        return (getPose().getX() > currentXInches - 2) && (getPose().getX() < currentXInches + 2);
    }

    public boolean checkYPos (double currentYInches){
        return (getPose().getY() > currentYInches - 2) &&  (getPose().getY() < currentYInches + 2);
    }
}
