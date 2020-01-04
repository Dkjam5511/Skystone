package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Robot;


@Autonomous(name = "Blue Auto", group = "Autonomous")
public class BlueMainAuto extends Robot {

    VuforiaStuff.skystonePos pos;
    int stoneDiff;
    final static double STANDARD_SPEED_MODIFIER = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, false);
        intake.on();
       // liftSystem.hLift.setPower(-.3);
        switch (pos) {
            case LEFT:
                driveToPoint(-12, 37, 0, 9);
                stoneDiff = 0;
                break;
            case CENTER:
                driveToPoint(2, 42, 0, 9);
                stoneDiff = 16;
                break;
            case RIGHT:
                driveToPoint(10, 9, 0, 9);
                driveToPoint(0, 33, 0, 9);
                //driveToPoint2(10, 42, 0, 9, 4);
                stoneDiff = 20;
                break;
        }
        if (pos == VuforiaStuff.skystonePos.LEFT) {
            driveToPoint(0, -12, 0, STANDARD_SPEED_MODIFIER);
        } else {
            driveToPoint(0, -17, 0, STANDARD_SPEED_MODIFIER);
        }
        liftSystem.grabStone();
        //liftSystem.hLift.setPower(0);
        intake.off();
        turn_to_heading(90, 0);
        liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
        driveToPoint(0, -64.5 - stoneDiff, 90, 11);
        turn_to_heading(180, 0); // do the drop
        grabbers.ready();
        //liftSystem.extend();
        driveToPoint(0, -10, 180, 10);
        grabbers.down();
        sleep(1000);
        liftSystem.dropStone();
        sleep(500);
        //liftSystem.retract();
        liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
        driveToPoint(0, 14, 180, 3);
        turn_to_heading(95, 15);
        driveToPoint(0, -14, 90, 2.4);
        grabbers.up();
        sleep(200);
        driveToPoint(0, 44, 90, 11);
        //liftSystem.hLift.setPower(-.2);
        sleep(500);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}
