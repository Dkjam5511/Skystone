package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous (name = "Vuforia Test Blue", group = "Tests")
public class VuforiaTestBlue extends Robot {

    VuforiaStuff.skystonePos pos;

    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        pos = vuforiaStuff.vuforiascan(true, false );
        telemetry.addData("Pos: ", pos);
        telemetry.update();
        sleep(3000);
    }
}
