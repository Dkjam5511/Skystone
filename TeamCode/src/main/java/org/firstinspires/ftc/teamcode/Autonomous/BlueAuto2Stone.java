package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "Blue Auto 2 Stone", group = "Autonomous")
public class BlueAuto2Stone extends Robot {

    VuforiaStuff.skystonePos pos;
    int stoneDiff;
    final static double STANDARD_SPEED_MODIFIER = 12;
    double pickupHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(false, false);
        intake.on();
        liftSystem.hLift.setPower(-.3);
        switch (pos) {
            case LEFT:
                driveToPoint(-12, 37, 0, 9);
                stoneDiff = 0;
                break;
            case CENTER:
                driveToPoint(2, 42, 0, 9);
                stoneDiff = 12;
                break;
            case RIGHT:
                driveToPoint(10, 9, 0, 9);
                driveToPoint(0, 33, 0, 9);
                //driveToPoint2(7, 42, 0, 9, 6);
                stoneDiff = 18;
                break;
        }
        if (pos == VuforiaStuff.skystonePos.LEFT) {
            driveToPoint(0, -12, 0, STANDARD_SPEED_MODIFIER);
        } else {
            driveToPoint(0, -17, 0, STANDARD_SPEED_MODIFIER);
        }
        liftSystem.grabStone();
        liftSystem.hLift.setPower(0);
        intake.off();
        turn_to_heading(90, 50);  // put x inches in next statement to account for x encoder turning
        liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
        driveToPoint(0, -65 - stoneDiff, 90, 11);
        turn_to_heading(180, 50);
        grabbers.ready();
        driveToPoint(0, -9, 180, 9);
        grabbers.down();
        liftSystem.dropStone();
        //sleep(500);  // was prior to lift system retracting, but moved retracting to after drive
        driveToPoint(0, 14, 180, 3);
        liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
        turn_to_heading(95, 15);
        driveToPoint(0, -12, 90, 2.2);
        grabbers.up();
        sleep(200);
        if (pos == VuforiaStuff.skystonePos.RIGHT) {
            driveToPoint(0, 44, 90, 11);
        } else {
            driveToPoint(2, 90 + stoneDiff, 90, 11);
            intake.on();
            liftSystem.hLift.setPower(-.3);
            turn_to_heading(0, 40);
            driveToPoint(0, 16, 0, 3.5);
            driveToPoint(0, -16, 0, 9);
            liftSystem.grabStone();
            turn_to_heading(90, 50);  // put x inches in next statement to account for x encoder turning
            liftSystem.extensionState = LiftSystem.ExtensionState.EXTENDING;
            driveToPoint(-6, -(83 + stoneDiff), 90, 11);
            liftSystem.dropStone();
            liftSystem.extensionState = LiftSystem.ExtensionState.RETRACTING;
            driveToPoint(7, 47, 90, 11);
        }
        liftSystem.hLift.setPower(-.2);
        sleep(500);
        driveTrain.applyPower(0, 0, 0, 0);
        liftSystem.stopMotors();
        intake.off();
    }
}