package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.Servo;

public class ServoEvent {

    private double xTrigger;
    private double yTrigger;
    private int Pose2dEventNumber;
    private int sleepAfter;
    private int eventType;
    private boolean eventCompleted;

    public static final int SE_MOVE_SERVO = 1;
    public static final int SE_GRABBERS_READY = 2;
    public static final int SE_GRABBERS_DOWN = 3;
    public static final int SE_GRABBERS_UP = 4;
    public static final int SE_GRAB_STONE = 5;
    public static final int SE_DROP_STONE = 6;
    public static final int SE_EXTEND_HLIFT_FAR = 7;
    public static final int SE_EXTEND_HLIFT_CLOSE = 8;
    public static final int SE_ROTATE_STONE_180 = 9;
    public static final int SE_ROTATE_STONE_90 = 10;
    public static final int SE_ROTATE_STONE_270 = 11;
    public static final int SE_RETRACT_HLIFT = 12;
    public static final int SE_INTAKE_ON = 13;
    public static final int SE_INTAKE_ON_SLOW = 14;
    public static final int SE_INTAKE_OFF = 15;
    public static final int SE_LEFT_CLAW_CLOSE = 16;
    public static final int SE_LEFT_CLAW_OPEN = 17;
    public static final int SE_LEFT_CLAW_LOWER = 18;
    public static final int SE_LEFT_CLAW_RAISE = 19;
    public static final int SE_LEFT_CLAW_PIVOT_PLACE = 20;
    public static final int SE_LEFT_CLAW_PIVOT_PLACE_HIGH = 21;
    public static final int SE_RIGHT_CLAW_CLOSE = 22;
    public static final int SE_RIGHT_CLAW_OPEN = 23;
    public static final int SE_RIGHT_CLAW_LOWER = 24;
    public static final int SE_RIGHT_CLAW_RAISE = 25;
    public static final int SE_RIGHT_CLAW_PIVOT_PLACE = 26;
    public static final int SE_RIGHT_CLAW_PIVOT_PLACE_HIGH = 27;


    public ServoEvent(double xTrigger, double yTrigger, int Pose2dEventNumber, int sleepAfter, int eventType){
        this.xTrigger = xTrigger;
        this.yTrigger = yTrigger;
        this.Pose2dEventNumber = Pose2dEventNumber;
        this.sleepAfter = sleepAfter;
        this.eventType = eventType;
        this.eventCompleted = false;

    }

    public double getXTrigger() { return xTrigger; }

    public double getYTrigger() { return yTrigger; }

    public int getSleepAfter() {
        return sleepAfter;
    }

    public int getPose2dEventNumber() {
        return Pose2dEventNumber;
    }

    public int getEventType() {
        return eventType;
    }

    public boolean getEventCompleted() {
        return eventCompleted;
    }

    public void setEventCompleted (boolean val) {eventCompleted = val;}

    public boolean checkXPos (double currentXInches, double startingX){
        if (startingX < xTrigger){
            return currentXInches >= xTrigger;
        } else {
            return currentXInches <= xTrigger;
        }
    }

    public boolean checkYPos (double currentYInches, double startingY){
        if (startingY < yTrigger){
            return currentYInches >= yTrigger;
        } else {
            return currentYInches <= yTrigger;
        }
    }

}
