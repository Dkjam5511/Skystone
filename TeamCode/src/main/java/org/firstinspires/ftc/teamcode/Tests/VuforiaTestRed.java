package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.Autonomous.Robot;

@Autonomous (name = "Vuforia Test R", group = "Tests")
public class VuforiaTestRed extends Robot {

    VuforiaStuff.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(true, true );
        telemetry.addData("Pos: ", pos);
        telemetry.update();
        sleep(3000);
    }
}
