package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous (name = "Vuforia Test", group = "Tests")
public class VuforiaTest extends Robot {

    VuforiaStuff.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan();
        telemetry.addData("Pos: ", pos);
        telemetry.update();
        sleep(2000);
    }
}
