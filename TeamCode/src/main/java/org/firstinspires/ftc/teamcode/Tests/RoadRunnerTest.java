package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous (name = "Roadrunner Test", group = "Tests")
public class RoadRunnerTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        roboInit();
        turnToHeadingRR(180);
    }
}
