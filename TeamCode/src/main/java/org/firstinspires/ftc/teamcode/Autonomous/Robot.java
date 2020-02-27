package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.DbgLog;
import org.firstinspires.ftc.teamcode.GlobalPositions;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.FoundationGrabbers;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Hardware.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Odometers;
import org.firstinspires.ftc.teamcode.Hardware.SideGrabbers;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.text.DecimalFormat;
import java.util.ArrayList;

import static java.lang.String.format;

abstract public class Robot extends LinearOpMode {
    public DriveTrain driveTrain;
    public Intake intake;
    public FoundationGrabbers grabbers;
    public Odometers odometers;
    public VuforiaStuff vuforiaStuff;
    public LiftSystem liftSystem;
    public SideGrabbers sideGrabbers;
    public Localizer localizer;
    public ServoEventManager servoEventManager;
    public DbgLog DbgLog;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";
    BNO055IMU gyro;
    public DistanceSensor rDistance;

    double gsPreviousSpeed;
    double gsPreviousXInches;
    double gsPreviousYInches;
    double gsPreviousTime;

    boolean gsFirstRun = true;
    ElapsedTime gsSpeedTimer = new ElapsedTime();
    ElapsedTime runTime = new ElapsedTime();

    public void roboInit() {

        DbgLog = new DbgLog();
        DbgLog.openLog();

        DbgLog.msg("10435 Robot.java beginning OP MODE, running roboInit");

        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rr = hardwareMap.dcMotor.get("rr");
        DcMotor intakeL = hardwareMap.dcMotor.get("il");
        DcMotor intakeR = hardwareMap.dcMotor.get("ir");
        DcMotor vLift = hardwareMap.dcMotor.get("vl");
        DcMotor vLift2 = hardwareMap.dcMotor.get("vl2");
        Servo hLift = hardwareMap.servo.get("hl");
        Servo stoneGrabber = hardwareMap.servo.get("sg");
        Servo stoneSpinner = hardwareMap.servo.get("ss");
        Servo hookL = hardwareMap.servo.get("hkl");
        Servo hookR = hardwareMap.servo.get("hkr");
        Servo rightClaw = hardwareMap.servo.get("rc");
        Servo rightClawPivot = hardwareMap.servo.get("rcp");
        Servo leftClaw = hardwareMap.servo.get("lc");
        Servo leftClawPivot = hardwareMap.servo.get("lcp");
        ExpansionHubMotor leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "il");
        ExpansionHubMotor rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "ir");
        ExpansionHubMotor frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "vl2");
        Servo capstonePost = hardwareMap.servo.get("cp");

        rDistance = hardwareMap.get(DistanceSensor.class, "rDistance");

        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        localizer = new Localizer(gyro, hardwareMap);
        localizer.imu.initialize();

        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_UP);
        stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        hookL.setPosition(GlobalPositions.HOOKL_UP);
        hookR.setPosition(GlobalPositions.HOOKR_UP);
        capstonePost.setPosition(GlobalPositions.CAPSTONE_START);
        hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
        rightClaw.setPosition(GlobalPositions.RIGHT_CLAW_INIT);
        rightClawPivot.setPosition(GlobalPositions.RIGHT_CLAW_PIVOT_UP);
        leftClaw.setPosition(GlobalPositions.LEFT_CLAW_INIT);
        leftClawPivot.setPosition(GlobalPositions.LEFT_CLAW_PIVOT_UP);

        vuforiaStuff = new VuforiaStuff(vuforia);
        driveTrain = new DriveTrain(lf, rf, lr, rr, false);
        intake = new Intake(intakeL, intakeR);
        grabbers = new FoundationGrabbers(hookL, hookR);
        odometers = new Odometers(frontEncoder, leftEncoder, rightEncoder);
        liftSystem = new LiftSystem(vLift, vLift2, hLift, stoneGrabber, stoneSpinner);
        sideGrabbers = new SideGrabbers(rightClaw, rightClawPivot, leftClaw, leftClawPivot);
        servoEventManager = new ServoEventManager();

        grabbers.up();

        telemetry.addLine("Ready to Start");
        telemetry.addData("ENCODER: ", lf.getMode().toString());
        telemetry.update();
        waitForStart();
        runTime.reset();
    }

    public void driveToPoint(double xInches, double yInches, double heading, double speedModifier) {

        DbgLog.msg(
                "10435-starting driveToPoint"
                        + " X:" + String.format("%6.2f", xInches)
                        + " Y:" + String.format("%6.2f", yInches)
                        + " Heading:" + String.format("%7.2f", heading)
                        + " SPM:" + String.format("%6.2f", speedModifier)
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;

        double maxWheelPower;
        double wheelPower = .15; //Minimum speed we start at

        gsFirstRun = true;

        ElapsedTime timeoutTimer = new ElapsedTime();

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        DbgLog.msg(
                "10435 driveToPoint"
                        + " XPos:" + String.format("%6.2f", currentXInches)
                        + " YPos:" + String.format("%6.2f", currentYInches)
                        + " Wheel Power:" + String.format("%6.2f", wheelPower)
                        + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
        );

        while ((distanceRemaining > 1 || currentSpeed > 3) && opModeIsActive() && timeoutTimer.seconds() < .75) {

            localizer.drive.update();

            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 25) / 100;

            double speedIncrease = .15;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);

            lfPower = wheelPower * Math.cos(angleRadians) - adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) - adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + adjustment;

            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            telemetry.addData("XPos: ", currentXInches);
            telemetry.addData("YPos: ", currentYInches);
            telemetry.addData("Current Speed:", currentSpeed);
            telemetry.addData("Wheel Power: ", wheelPower);
            telemetry.addData("distanceToX: ", distanceToX);
            telemetry.addData("distanceToY: ", distanceToY);
            telemetry.addData("Distance remaining: ", distanceRemaining);
            telemetry.addData("andleradians: ", angleRadians);
            telemetry.addData("angleradianDegrees: ", Math.toDegrees(angleRadians));
            telemetry.update();

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            distanceToX = xInches - currentXInches;
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            DbgLog.msg(
                    "10435 driveToPoint"
                            + " XPos:" + String.format("%6.2f", currentXInches)
                            + " YPos:" + String.format("%6.2f", currentYInches)
                            + " distanceToX:" + String.format("%6.2f", distanceToX)
                            + " distanceToY:" + String.format("%6.2f", distanceToY)
                            + " Wheel Power:" + String.format("%6.2f", wheelPower)
                            + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
                            + " angleradianDegrees:" + String.format("%7.2f", Math.toDegrees(angleRadians + Math.PI / 4))
                            + " currentSpeed:" + String.format("%6.2f", currentSpeed)
                            + " ajustment:" + String.format("%6.2f", adjustment)
                            + " current heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
    }

    public void driveToPoint4(ArrayList<DriveParameters> parametersList) {
/*
        DbgLog.msg(
                "10435 driveToPoint4 STARTING"
        );

        logServoEnvents();

        int lastPoseNum = 0;
        int parmetersListSize = parametersList.size();

        for (int poseNum = 0; poseNum < parmetersListSize; poseNum++) {

            DriveParameters parameters = parametersList.get(poseNum);
            lastPoseNum = poseNum;

            double targetX = parameters.getTargetPose().getX();
            double targetY = parameters.getTargetPose().getY();
            double targetHeading = parameters.getTargetPose().getHeading();
            double distanceTolerance = parameters.getDistanceTolerance();
            double speedModifer = parameters.getSpeedModifier();
            double xCorrectFactor = parameters.getxCorrectFactor();
            double yCorrectFactor = parameters.getyCorrectFactor();
            double headingTolerance = parameters.getHeadingTolerance();

            double currentXInches = localizer.getXPosition();
            double currentYInches = localizer.getYPosition();
            double startingX = currentXInches;
            double startingY = currentYInches;

            double distanceToX = targetX - currentXInches;
            double distanceToY = targetY - currentYInches;

            double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));

            if (speedModifer == 0) {
                speedModifer = 6;
            }

            *//* Old auto calc of dtpDistanceReamining
            if (i < parametersList.size() - 1) {
                if (distanceRemaining < 40) {
                    distanceTolerance = 4;
                } else {
                    distanceTolerance = 7;
                }
            } else {
                distanceTolerance = GlobalPositions.DTP_DISTANCE_REMAINING;
            }

             *//*

            double wheelPower = .2;
            double speedIncrease = .2;
            double minWheelPower = 30;
            double currentSpeed = 0;
            double maxSpeed = 0;
            double minSpeed = 0;
            boolean maxSpeedReached;

            maxSpeedReached = false;

            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;

            double maxWheelPower;

            double angleRadians;

            DbgLog.msg(
                    "10435 driveToPoint4"
                            + "    Target Pose:" + String.format("%2d", poseNum)
                            + "    Target Heading:" + String.format("%6.1f", Math.toDegrees(targetHeading))
                            + "    Target X:" + String.format("%6.2f", targetX)
                            + "    Target Y:" + String.format("%6.2f", targetY)
                            + "    XPos:" + String.format("%6.2f", currentXInches)
                            + "    YPos:" + String.format("%6.2f", currentYInches)
                            + "    Distance to X:" + String.format("%6.2f", distanceToX)
                            + "    Distance to Y:" + String.format("%6.2f", distanceToY)
                            + "    Distance Remaining:" + String.format("%6.2f", distanceRemaining)
                            + "    X Correct Factor:" + String.format("%4.1f", xCorrectFactor)
                            + "    Y Correct Factor:" + String.format("%4.1f", yCorrectFactor)
                            + "    Distance Tolerance:" + String.format("%5.1f", distanceTolerance)
                            + "    Heading Tolerance:" + String.format("%6.1f", Math.toDegrees(headingTolerance))
                            + "    Speed Modifier:" + String.format("%4.1f", speedModifer)
            );

            while ((distanceRemaining > distanceTolerance || Math.abs(localizer.getCurrentHeadingRadians() - targetHeading) > headingTolerance) && opModeIsActive()) {


                currentXInches = localizer.getXPosition();
                currentYInches = localizer.getYPosition();

                runServoEvents(poseNum, currentXInches, startingX, currentYInches, startingY);

                distanceToX = targetX - currentXInches;
                distanceToY = targetY - currentYInches;

                distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));

                currentSpeed = getSpeed(currentXInches, currentYInches);

               *//* maxSpeed = (distanceRemaining - 10) / .7 + 20;

                if (!maxSpeedReached) {
                    maxSpeedReached = (currentSpeed >= maxSpeed);
                }

                //  This is a hack.  This only works when the robot is oriented in the Y direction, so X is sideways.

                if (distanceToX / distanceToY > 1) {
                    minWheelPower = 40;
                } else {
                    minWheelPower = 25;
                }


                minSpeed = Math.pow(distanceRemaining / speedModifer, 3);
                if (minSpeed > 100 - minWheelPower) {
                    minSpeed = 100 - minWheelPower;
                }
                minSpeed = 42 / (100 - minWheelPower) * minSpeed + 5;

                if (maxSpeedReached && currentSpeed >= minSpeed) {
                    maxWheelPower = (Math.pow(distanceRemaining / speedModifer, 3) + minWheelPower) / 100;
                } else {
                    if (maxSpeedReached) {
                        maxWheelPower = .7;
                    } else{
                        maxWheelPower = 1;
                    }
                }*//*

                maxWheelPower = (Math.pow(distanceRemaining / speedModifer, 3) + minWheelPower) / 100;

                wheelPower += speedIncrease;
                if (wheelPower > maxWheelPower) {
                    wheelPower = maxWheelPower;
                }

                double adjustment = localizer.headingAdjustment(360 - Math.toDegrees(targetHeading));

                if (adjustment > maxWheelPower) {
                    adjustment = maxWheelPower;
                } else if (adjustment < -maxWheelPower) {
                    adjustment = -maxWheelPower;
                }

                angleRadians = Math.atan2(distanceToX * xCorrectFactor, distanceToY * yCorrectFactor) - Math.PI / 4 + targetHeading;

                lfPower = wheelPower * Math.sin(angleRadians) - adjustment;
                rfPower = wheelPower * Math.cos(angleRadians) + adjustment;
                lrPower = wheelPower * Math.cos(angleRadians) - adjustment;
                rrPower = wheelPower * Math.sin(angleRadians) + adjustment;

                driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

                telemetry.addData("Current Left Ticks: ", odometers.leftEncoder.getCurrentPosition());
                telemetry.addData("Current Right Ticks: ", -odometers.rightEncoder.getCurrentPosition());
                telemetry.addData("Current Front Ticks: ", -odometers.frontEncoder.getCurrentPosition());
                telemetry.addData("Current X: ", currentXInches);
                telemetry.addData("Current Y: ", currentYInches);
                telemetry.addData("Current Heading: ", localizer.getCurrentHeadingDegrees());
                telemetry.addData("Current Speed: ", currentSpeed);
                telemetry.update();

                DbgLog.msg(
                        "10435 driveToPoint4"
                                + "    XPos:" + String.format("%6.2f", currentXInches)
                                + "    YPos:" + String.format("%6.2f", currentYInches)
                                + "    Distance Remaining:" + String.format("%6.2f", distanceRemaining)
                                + "    Angle to TargetXY:" + String.format("%7.2f", Math.toDegrees(angleRadians + Math.PI/4))
                                + "    Adjustment:" + String.format("%7.2f", adjustment)
                                + "    Current Heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
                                //+ "    X Correct Factor:" + String.format("%4.1f", xCorrectFactor)  // only log these if dynamically changing them
                                //+ "    Y Correct Factor:" + String.format("%4.1f", yCorrectFactor)  // only log these if dynamically changing them
                                + "    Wheel Power:" + String.format("%6.3f", wheelPower)
                                + "    Max Wheel Power:" + String.format("%6.3f", maxWheelPower)
                                + "    Min Wheel Power:" + String.format("%6.3f", minWheelPower)
                                + "    Current Speed:" + String.format("%6.2f", currentSpeed)
                                + "    Max Speed:" + String.format("%6.2f", maxSpeed)
                                + "    Max Speed Reached:" + String.format("%5b", maxSpeedReached)
                                + "    Min Speed:" + String.format("%6.2f", minSpeed)
                                + "    LF Wheel Power:" + String.format("%5.2f", lfPower)
                                + "    LR Wheel Power:" + String.format("%5.2f", lrPower)
                                + "    RF Wheel Power:" + String.format("%5.2f", rfPower)
                                + "    RR Wheel Power:" + String.format("%5.2f", rrPower)
                );

            }
        }

        driveTrain.applyPower(0, 0, 0, 0);
        runServoEvents(lastPoseNum + 1, 0,0,0,0);
        servoEventManager.removeAllEvents();
        sleep(1000);
        DbgLog.msg(
                "10435 driveToPoint4"
                        + "    XPos:" + String.format("%6.2f", localizer.getXPosition())
                        + "    YPos:" + String.format("%6.2f", localizer.getYPosition())
                        + "    Current Heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
        );*/
    }

    public void driveToPointTest2(ArrayList<DriveParameters> parametersList) {

        DbgLog.msg(
                "10435 driveToPointTest2 STARTING"
        );

        logServoEvents();

        int lastPoseNum = 0;
        int parmetersListSize = parametersList.size();
        double currentSpeed = 0;
        double wallXDiffTotal = 0;
        double wallXDiffAvg = 0;
        int wallXDiffCount = 0;
        double wallDistance = 0;

        for (int poseNum = 0; poseNum < parmetersListSize; poseNum++) {

            DriveParameters parameters = parametersList.get(poseNum);
            lastPoseNum = poseNum;

            double targetX = parameters.getTargetPose().getX();
            double targetY = parameters.getTargetPose().getY();
            double targetHeadingRadians = parameters.getTargetPose().getHeading();
            double targetHeadingDegrees = Math.toDegrees(targetHeadingRadians);
            double distanceToleranceX = parameters.getDistanceToleranceX();
            double distanceToleranceY = parameters.getDistanceToleranceY();
            double speedModifer = parameters.getSpeedModifier();
            double xCorrectFactor = parameters.getxCorrectFactor();
            double yCorrectFactor = parameters.getyCorrectFactor();
            double headingTolerance = parameters.getHeadingTolerance();
            double currentHeading = 360 - localizer.getCurrentHeadingDegrees();  // Converts our orientation to Radians orientation with 90 degrees pointing left

            double currentXInches = localizer.getXPosition();
            double currentYInches = localizer.getYPosition();
            double startingX = currentXInches;
            double startingY = currentYInches;

            double distanceToX = targetX - currentXInches;
            double distanceToY = targetY - currentYInches;

            double speedIncrease = .2;
            double wheelPower = currentSpeed / 75 / 100;  // speedIncrease will be added to this in the while loop

            double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));
            double startingDistanceRemaining = distanceRemaining;

            double pidP = parameters.getPidP();
            if (pidP == 0) {
                pidP = 5.45;
            }
            double pidI = 0;
            double pidD = 1.65;

            PIDCoefficients coeffs = new PIDCoefficients(pidP, pidI, pidD);
            PIDFController controller = new PIDFController(coeffs);

            controller.setTargetPosition(startingDistanceRemaining);

            if (speedModifer == 0) {
                speedModifer = 1;
            }


            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;

            double maxWheelPower;

            double angleRadians;

            DbgLog.msg(
                    "10435 driveToPointTest2"
                            + "    Pose:" + String.format("%2d", poseNum)
                            + "    Heading:" + String.format("%6.1f", targetHeadingDegrees)
                            + "    TargetX:" + String.format("%6.2f", targetX)
                            + "    TargetY:" + String.format("%6.2f", targetY)
                            + "    ToleranceX:" + String.format("%5.1f", distanceToleranceX)
                            + "    ToleranceY:" + String.format("%5.1f", distanceToleranceY)
                            + "    XPos:" + String.format("%6.2f", currentXInches)
                            + "    YPos:" + String.format("%6.2f", currentYInches)
                            + "    Distance to X:" + String.format("%6.2f", distanceToX)
                            + "    Distance to Y:" + String.format("%6.2f", distanceToY)
                            + "    Distance Remaining:" + String.format("%6.2f", distanceRemaining)
                            + "    X Correct Factor:" + String.format("%4.1f", xCorrectFactor)
                            + "    Y Correct Factor:" + String.format("%4.1f", yCorrectFactor)
                            + "    Heading Tolerance:" + String.format("%6.1f", Math.toDegrees(headingTolerance))
                            + "    Speed Modifier:" + String.format("%4.1f", speedModifer)
                            + "    pidP:" + String.format("%4.1f", pidP)
            );

            double speedTolerance;
            if (distanceToleranceX <= 2 || distanceToleranceY <= 2) {
                speedTolerance = 7;
            } else {
                speedTolerance = 70;
            }

            boolean firstLoop = true;
            double currentHeadingDiff = Math.abs(currentHeading - targetHeadingDegrees);
            if (currentHeadingDiff > 180) {
                currentHeadingDiff = 360 - currentHeadingDiff;
            }
            while ((Math.abs(distanceToX) > distanceToleranceX || Math.abs(distanceToY) > distanceToleranceY || currentSpeed > speedTolerance || currentHeadingDiff > headingTolerance) && opModeIsActive()) {

                runServoEvents(poseNum, currentXInches, startingX, currentYInches, startingY);

                distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));

                currentSpeed = getSpeed(currentXInches, currentYInches);

                maxWheelPower = Math.abs(controller.update(startingDistanceRemaining - distanceRemaining) / 100) * speedModifer;

                if (firstLoop & maxWheelPower < wheelPower) { // If robot is already moving, wheelPower will not be zero.  This overrides the 0 that would come from controller.update on first run
                    maxWheelPower = wheelPower + speedIncrease;
                }
                firstLoop = false;

                if (maxWheelPower > 1) {
                    maxWheelPower = 1;
                }

                if (currentSpeed < 6 && maxWheelPower < .17) {  // raise the max wheel power if we're stopped
                    maxWheelPower = .17;
                }

                wheelPower += speedIncrease;
                if (wheelPower > maxWheelPower) {
                    wheelPower = maxWheelPower;
                }

                double adjustment;
                if (speedModifer == 1.01 && distanceToX > 20) {  // secret code of 1.01
                    adjustment = 0;
                } else {
                    adjustment = localizer.headingAdjustment(360 - targetHeadingDegrees);
                }

                if (adjustment > maxWheelPower) {
                    adjustment = maxWheelPower;
                } else if (adjustment < -maxWheelPower) {
                    adjustment = -maxWheelPower;
                }

                angleRadians = Math.atan2(distanceToX * xCorrectFactor, distanceToY * yCorrectFactor) - Math.PI / 4 + targetHeadingRadians;

                double sinAngleRadians = Math.sin(angleRadians);
                double cosAngleRadians = Math.cos(angleRadians);
                double factor = 1 / (Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians)) + Math.abs(adjustment));

                lfPower = wheelPower * sinAngleRadians * factor - adjustment;
                rfPower = wheelPower * cosAngleRadians * factor + adjustment;
                lrPower = wheelPower * cosAngleRadians * factor - adjustment;
                rrPower = wheelPower * sinAngleRadians * factor + adjustment;

                driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

                /*
                telemetry.addData("Current Left Ticks: ", odometers.leftEncoder.getCurrentPosition());
                telemetry.addData("Current Right Ticks: ", -odometers.rightEncoder.getCurrentPosition());
                telemetry.addData("Current Front Ticks: ", -odometers.frontEncoder.getCurrentPosition());
                telemetry.addData("Current X: ", currentXInches);
                telemetry.addData("Current Y: ", currentYInches);
                telemetry.addData("Current Heading: ", localizer.getCurrentHeadingDegrees());
                telemetry.addData("Current Speed: ", currentSpeed);

                telemetry.addData("Angle Radians: ", angleRadians);
                telemetry.addData("Angle Radians Degrees: ", Math.toDegrees(angleRadians));
                telemetry.update();
                 */

                DbgLog.msg(
                        "10435 driveToPointTest2"
                                + "    XPos:" + String.format("%6.2f", currentXInches)
                                + "    YPos:" + String.format("%6.2f", currentYInches)
                                + "    Distance Remaining:" + String.format("%6.2f", distanceRemaining)
                                + "    Distance RemainingX:" + String.format("%6.2f", distanceToX)
                                + "    Distance RemainingY:" + String.format("%6.2f", distanceToY)
                                + "    Angle to TargetXY:" + String.format("%7.2f", Math.toDegrees(angleRadians + Math.PI / 4))
                                + "    Adjustment:" + String.format("%7.2f", adjustment)
                                + "    Current Heading:" + String.format("%7.2f", currentHeading)
                                + "    Heading Off:" + String.format("%7.2f", currentHeadingDiff)
                                //+ "    X Correct Factor:" + String.format("%4.1f", xCorrectFactor)  // only log these if dynamically changing them
                                //+ "    Y Correct Factor:" + String.format("%4.1f", yCorrectFactor)  // only log these if dynamically changing them
                                + "    Wheel Power:" + String.format("%6.3f", wheelPower)
                                + "    Max Wheel Power:" + String.format("%6.3f", maxWheelPower)
                                + "    Current Speed:" + String.format("%6.2f", currentSpeed)
                                + "    LF Wheel Power:" + String.format("%5.2f", lfPower)
                                + "    LR Wheel Power:" + String.format("%5.2f", lrPower)
                                + "    RF Wheel Power:" + String.format("%5.2f", rfPower)
                                + "    RR Wheel Power:" + String.format("%5.2f", rrPower)
                                + "    Wall Distance:" + String.format("%6.2f", wallDistance)
                                + "    Wall-X Diff:" + String.format("%6.2f", wallDistance - currentXInches)
                                + "    Avg Diff:" + String.format("%6.2f", wallXDiffAvg)
                                + "    X Diff Count:" + String.format("%3d", wallXDiffCount)
                );


                currentXInches = localizer.getXPosition();
                currentYInches = localizer.getYPosition();
                currentHeading = 360 - localizer.getCurrentHeadingDegrees();
                currentHeadingDiff = Math.abs(currentHeading - targetHeadingDegrees);
                if (currentHeadingDiff > 180) {
                    currentHeadingDiff = 360 - currentHeadingDiff;
                }
                distanceToX = targetX - currentXInches;
                distanceToY = targetY - currentYInches;

                double wallDiffNormal = 1.7;  // Normal difference in distance readings between distance sensor and X Pos.

                if (targetHeadingDegrees == 270) {
                    wallDistance = rDistance.getDistance(DistanceUnit.INCH) - wallDiffNormal;
                    //telemetry.addData("Wall Distance: ", wallDistance);
                    //telemetry.update();
                } //else if (targetHeadingDegrees == 90) {walldistance = lDistance.getDistance(DistanceUnit.INCH);}


                if (Math.abs(currentHeading - targetHeadingDegrees) <= 4 && ((targetHeadingDegrees == 90 && currentYInches > -20) || (targetHeadingDegrees == 270 && (currentYInches < 20 || currentYInches > 60)))) {
                    double wallXDiff = wallDistance - currentXInches;
                    if (Math.abs(wallXDiff) < 4) {
                        wallXDiffTotal = wallXDiffTotal + wallXDiff;
                        wallXDiffCount = wallXDiffCount + 1;
                        wallXDiffAvg = wallXDiffTotal / wallXDiffCount;
                    }
                }

                if (wallXDiffCount > 3) {
                    //localizer.artificialAdjust(0, -(wallXDiffAvg - 2));  // 2 would be what x diff is supposed to be
                    if (wallXDiffAvg > 1) {
                        localizer.XWallDiff += wallXDiffAvg;
                        telemetry.addData("Adjusted: ", wallXDiffAvg);
                        telemetry.update();
                        DbgLog.msg(
                                "10435 driveToPointTest2"
                                        + "    Wall Distance Adjusted:" + String.format("%6.2f", wallXDiffAvg)
                                        + "    Old X:" + String.format("%6.2f", currentXInches)
                                        + "    New X:" + String.format("%6.2f", localizer.getXPosition())
                        );
                        currentXInches = localizer.getXPosition();
                    }
                    wallXDiffTotal = 0;
                    wallXDiffCount = 0;
                    wallXDiffAvg = 0;
                }

            }

            wallXDiffTotal = 0;
            wallXDiffCount = 0;
            wallXDiffAvg = 0;

        }

        driveTrain.applyPower(0, 0, 0, 0);
        runServoEvents(lastPoseNum + 1, 0, 0, 0, 0);
        servoEventManager.removeAllEvents();
        DbgLog.msg(
                "10435 driveToPointTest2"
                        + "    XPos:" + String.format("%6.2f", localizer.getXPosition())
                        + "    YPos:" + String.format("%6.2f", localizer.getYPosition())
                        + "    Current Heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
        );
    }


    public void driveToPointTest(ArrayList<DriveParameters> parametersList) {
/*
        DbgLog.msg(
                "10435 driveToPointTest STARTING"
        );

        logServoEvents();

        int lastPoseNum = 0;

        for (int poseNum = 0; poseNum < parametersList.size(); poseNum++) {

            DriveParameters parameters = parametersList.get(poseNum);
            lastPoseNum = poseNum;

            double targetX = parameters.getTargetPose().getX();
            double targetY = parameters.getTargetPose().getY();
            double targetHeading = parameters.getTargetPose().getHeading();
            double distanceTolerance = parameters.getDistanceTolerance();
            double speedModifer = parameters.getSpeedModifier();
            double xCorrectFactor = parameters.getxCorrectFactor();
            double yCorrectFactor = parameters.getyCorrectFactor();
            double headingTolerance = parameters.getHeadingTolerance();

            double currentXInches = localizer.getXPosition();
            double currentYInches = localizer.getYPosition();
            double startingX = currentXInches;
            double startingY = currentYInches;

            double distanceToX = targetX - currentXInches;
            double distanceToY = targetY - currentYInches;

            double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));
            double startingDistanceRemaining = distanceRemaining;

            if (speedModifer == 0) {
                speedModifer = 8;
            }

            double wheelPower = 0;
            double speedIncrease = .3;
            double minWheelPower = 30;
            double currentSpeed = 0;
            double maxSpeed = 0;
            double minSpeed = 0;
            boolean maxSpeedReached;

            maxSpeedReached = false;

            double lfPower;
            double rfPower;
            double lrPower;
            double rrPower;

            double maxWheelPower;

            double angleRadians;

            double pidP = 4.2;
            double pidI = 0;
            double pidD = .1;

            PIDCoefficients coeffs = new PIDCoefficients(pidP,pidI,pidD);
            PIDFController controller = new PIDFController(coeffs);

            controller.setTargetPosition(startingDistanceRemaining);

            DbgLog.msg(
                    "10435 driveToPointTest"
                            + "    Target Pose:" + String.format("%2d", poseNum)
                            + "    Target Heading:" + String.format("%6.1f", Math.toDegrees(targetHeading))
                            + "    Target X:" + String.format("%6.2f", targetX)
                            + "    Target Y:" + String.format("%6.2f", targetY)
                            + "    PID: " + String.format("%4.1f", pidP)  + " " + String.format("%4.1f", pidI)+ " " + String.format("%4.1f", pidD)
                            + "    XPos:" + String.format("%6.2f", currentXInches)
                            + "    YPos:" + String.format("%6.2f", currentYInches)
                            + "    Distance to X:" + String.format("%6.2f", distanceToX)
                            + "    Distance to Y:" + String.format("%6.2f", distanceToY)
                            + "    Distance Remaining:" + String.format("%6.2f", distanceRemaining)
                            + "    X Correct Factor:" + String.format("%4.1f", xCorrectFactor)
                            + "    Y Correct Factor:" + String.format("%4.1f", yCorrectFactor)
                            + "    Distance Tolerance:" + String.format("%5.1f", distanceTolerance)
                            + "    Heading Tolerance:" + String.format("%6.1f", Math.toDegrees(headingTolerance))
                            + "    Speed Modifier:" + String.format("%4.1f", speedModifer)
            );

            while ((distanceToX > 1 || distanceToY > 1) && opModeIsActive()) {

                currentXInches = localizer.getXPosition();
                currentYInches = localizer.getYPosition();

                runServoEvents(poseNum, currentXInches, startingX, currentYInches, startingY);

                distanceToX = targetX - currentXInches;
                distanceToY = targetY - currentYInches;

                distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));
                //distanceRemaining = targetX - currentXInches;

                currentSpeed = getSpeed(currentXInches, currentYInches);

               *//* maxSpeed = (distanceRemaining - 10) / .7 + 20;

                if (!maxSpeedReached) {
                    maxSpeedReached = (currentSpeed >= maxSpeed);
                }

                //  This is a hack.  This only works when the robot is oriented in the Y direction, so X is sideways.

                if (distanceToX / distanceToY > 1) {
                    minWheelPower = 40;
                } else {
                    minWheelPower = 25;
                }


                minSpeed = Math.pow(distanceRemaining / speedModifer, 3);
                if (minSpeed > 100 - minWheelPower) {
                    minSpeed = 100 - minWheelPower;
                }
                minSpeed = 42 / (100 - minWheelPower) * minSpeed + 5;

                if (maxSpeedReached && currentSpeed >= minSpeed) {
                    maxWheelPower = (Math.pow(distanceRemaining / speedModifer, 3) + minWheelPower) / 100;
                } else {
                    if (maxSpeedReached) {
                        maxWheelPower = .7;
                    } else{
                        maxWheelPower = 1;
                    }
                }*//*

                //maxWheelPower = (Math.pow( (distanceRemaining + distanceModifier) / speedModifer, 3) + minWheelPower) / 100;
                *//*
                if (currentXInches < targetX) {
                    maxWheelPower = 1;
                } else {
                    maxWheelPower = .05;
                    if (currentSpeed < 10) {
                        maxWheelPower = 0;
                    } else if (currentSpeed < 30) {
                        maxWheelPower = .16;
                    }
                }

                 *//*

                //double realDistanceRemaining = distanceRemaining;

              *//*  if (angleRadians < 0){
                    realDistanceRemaining *= -1;
                }*//*

                angleRadians = Math.atan2(distanceToX * xCorrectFactor, distanceToY * yCorrectFactor) - Math.PI / 4 + targetHeading;

                if (angleRadians < 0) {
                    distanceRemaining = -distanceRemaining;
                }

                maxWheelPower = controller.update(startingDistanceRemaining - distanceRemaining) / 100;

                wheelPower += speedIncrease;
                if (wheelPower > 1) {
                    wheelPower = 1;
                } else if (wheelPower > maxWheelPower) {
                    wheelPower = maxWheelPower;
                }

                double adjustment = localizer.headingAdjustment(360 - Math.toDegrees(targetHeading));

                *//*
                if (adjustment > wheelPower) {
                    adjustment = wheelPower;
                } else if (adjustment < -wheelPower) {
                    adjustment = -wheelPower;
                }

                 *//*

                double sinAngleRadians = Math.sin(angleRadians);
                double cosAngleRadians = Math.cos(angleRadians);
                double factor = 1 / Math.max(Math.abs(sinAngleRadians), Math.abs(cosAngleRadians));

                lfPower = wheelPower * sinAngleRadians * factor - adjustment;
                rfPower = wheelPower * cosAngleRadians * factor + adjustment;
                lrPower = wheelPower * cosAngleRadians * factor - adjustment;
                rrPower = wheelPower * sinAngleRadians * factor + adjustment;

                driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

                telemetry.addData("Wheel Power: ", wheelPower);
                telemetry.addData("Current Left Ticks: ", odometers.leftEncoder.getCurrentPosition());
                telemetry.addData("Current Right Ticks: ", -odometers.rightEncoder.getCurrentPosition());
                telemetry.addData("Current Front Ticks: ", -odometers.frontEncoder.getCurrentPosition());
                telemetry.addData("Current X: ", currentXInches);
                telemetry.addData("Current Y: ", currentYInches);
                telemetry.addData("Current Heading: ", localizer.getCurrentHeadingDegrees());
                telemetry.addData("Current Speed: ", currentSpeed);
                telemetry.update();

                DbgLog.msg(
                        "10435 driveToPointTest"
                                + "    XPos:" + String.format("%6.2f", currentXInches)
                                + "    YPos:" + String.format("%6.2f", currentYInches)
                                + "    Distance Remaining:" + String.format("%6.2f", distanceRemaining)
                                + "    Angle to TargetXY:" + String.format("%7.2f", Math.toDegrees(angleRadians + Math.PI/4))
                                + "    Adjustment:" + String.format("%7.2f", adjustment)
                                + "    Current Heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
                                + "    sinAngleRadians:" + String.format("%7.2f", sinAngleRadians)
                                + "    cosAngleRadians:" + String.format("%7.2f", cosAngleRadians)
                                + "    Wheel Power:" + String.format("%6.3f", wheelPower)
                                + "    Max Wheel Power:" + String.format("%6.3f", maxWheelPower)
                                + "    Min Wheel Power:" + String.format("%6.3f", minWheelPower)
                                + "    Current Speed:" + String.format("%6.2f", currentSpeed)
                                + "    Max Speed:" + String.format("%6.2f", maxSpeed)
                                + "    Max Speed Reached:" + String.format("%5b", maxSpeedReached)
                                + "    Min Speed:" + String.format("%6.2f", minSpeed)
                                + "    LF Wheel Power:" + String.format("%5.2f", lfPower)
                                + "    LR Wheel Power:" + String.format("%5.2f", lrPower)
                                + "    RF Wheel Power:" + String.format("%5.2f", rfPower)
                                + "    RR Wheel Power:" + String.format("%5.2f", rrPower)
                );

            }
        }

        driveTrain.applyPower(0, 0, 0, 0);
        runServoEvents(lastPoseNum + 1, 0,0,0,0);
        servoEventManager.removeAllEvents();
        DbgLog.msg(
                "10435 driveToPointTest"
                        + "    XPos:" + String.format("%6.2f", localizer.getXPosition())
                        + "    YPos:" + String.format("%6.2f", localizer.getYPosition())
                        + "    Current Heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
        );*/
    }

    public void pullFoundation(double targetX, double targetY, boolean blue) {

        double startingHeading = localizer.getCurrentHeadingDegrees();
        double currentHeading;

        double currentXInches = localizer.getXPosition();
        double currentYInches = localizer.getYPosition();

        double distanceToX = currentXInches - targetX;
        double distanceToY = currentYInches - targetY;

        double wheelPower = .2;
        double speedIncrease = .1;

        double insideFront = -.2;
        double insideRear = .7;
        double outsideFront = 1;
        double outsideRear = 1;

        double currentSpeed;
        currentSpeed = getSpeed(currentXInches, currentYInches);
        currentSpeed = 0;  // throw away current speed.  Just calling it above to make sure it's set.

        boolean timeOut = false;
        boolean haveMoved = false;

        currentHeading = localizer.getCurrentHeadingDegrees();

        double degreesOff;

        DbgLog.msg(
                "10435 pullFoundationX"
                        + " XPos:" + currentXInches
                        + " YPos:" + currentYInches
                        + " distanceToX:" + distanceToX
                        + " Speed:" + currentSpeed
                        + " Current Heading:" + currentHeading
                        + " haveMoved:" + haveMoved
                        + " timeout:" + timeOut
        );

        ElapsedTime timer = new ElapsedTime();

        timeOut = (currentHeading > 190 || currentHeading < 170);  // if we're not already against the platform pointing to 180, skip it.

        while (distanceToX > 1 && !timeOut && opModeIsActive()) {

            driveTrain.applyPower(wheelPower, wheelPower, wheelPower, wheelPower);

            wheelPower += speedIncrease;
            if (wheelPower > 1) {
                wheelPower = 1;
            }

            currentHeading = localizer.getCurrentHeadingDegrees();

            degreesOff = Math.abs(currentHeading - startingHeading);

            if (degreesOff > 180) {
                degreesOff = Math.abs(360 - degreesOff);
            }

            if (degreesOff > 15) {
                timeOut = true;
            }

            currentXInches = localizer.getXPosition();
            currentYInches = localizer.getYPosition();

            distanceToX = currentXInches - targetX;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > 2) {
                haveMoved = true;
                timer.reset();
            }

            if (timer.seconds() > 1 || (haveMoved && currentSpeed < 1)) {
                timeOut = true;
            }

            DbgLog.msg(
                    "10435 pullFoundationX"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " distanceToX:" + distanceToX
                            + " Speed:" + currentSpeed
                            + " Current Heading:" + currentHeading
                            + " Degrees off:" + degreesOff
                            + " haveMoved:" + haveMoved
                            + " timeout:" + timeOut
            );

        }

        haveMoved = false;

        while (distanceToY > 1 && !timeOut && opModeIsActive()) {
            currentYInches = localizer.getYPosition();

            distanceToY = currentYInches - targetY;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > 4) {
                haveMoved = true;
                timer.reset();
            }

            if (timer.seconds() > 2 || (haveMoved && currentSpeed < 1)) {
                timeOut = true;
            }

            if (blue) {
                driveTrain.applyPower(insideFront, outsideFront, insideRear, outsideRear);
            } else {
                driveTrain.applyPower(outsideFront, insideFront, outsideRear, insideRear);
            }

            DbgLog.msg(
                    "10435 pullFoundationY"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " distanceToY:" + distanceToY
                            + " Speed:" + currentSpeed
                            + " haveMoved:" + haveMoved
                            + " Current Heading:" + localizer.getCurrentHeadingDegrees()
            );
        }

        driveTrain.applyPower(0, 0, 0, 0);

    }

    public void pushFoundation(double targetY) {
        double currentXInches = localizer.getXPosition();
        double currentYInches = localizer.getYPosition();

        double distanceToY = targetY - currentYInches;

        double currentSpeed;

        boolean timeOut = false;
        boolean haveMoved = false;

        ElapsedTime timer = new ElapsedTime();

        while (distanceToY > 1 && !timeOut) {
            currentXInches = localizer.getXPosition();
            currentYInches = localizer.getYPosition();

            distanceToY = targetY - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > 2) {
                haveMoved = true;
                timer.reset();
            }

            if (timer.seconds() > 1.5 || (haveMoved && currentSpeed < 1)) {
                timeOut = true;
            }

            driveTrain.applyPower(-1, -1, -1, -1);
        }
        driveTrain.applyPower(0, 0, 0, 0);
    }

    public void driveToPoint3(double xInches, double yInches, double heading, double maxWheelPower, double xCorrectFactor) {

        DbgLog.msg(
                "10435-starting driveToPoint3"
                        + " X:" + String.format("%6.2f", xInches)
                        + " Y:" + String.format("%6.2f", yInches)
                        + " Heading:" + String.format("%7.2f", heading)
                        + " maxWheelPower:" + String.format("%6.2f", maxWheelPower)
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double highestCurrentSpeed = 0;
        double distanceToStop;

        double speedIncrease = .15;
        double wheelPower = .15; //Minimum speed we start at

        boolean stop = false;

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;

        gsFirstRun = true;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        DbgLog.msg(
                "10435-driveToPoint3"
                        + " XPos:" + String.format("%6.2f", currentXInches)
                        + " YPos:" + String.format("%6.2f", currentYInches)
                        + " Wheel Power:" + String.format("%6.2f", wheelPower)
                        + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING || currentSpeed > GlobalPositions.DTP_SPEED_SENSITIVITY /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1) {


            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop) {
                if (currentSpeed > 30) {
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);  // adjustment curve seems very suited for very slow speeds. more adjustment at higher wheel powers

            if (adjustment > maxWheelPower) {
                adjustment = maxWheelPower;
            } else if (adjustment < -maxWheelPower) {
                adjustment = -maxWheelPower;
            }

            double leftAdjustment = -adjustment;
            double rightAdjustment = adjustment;

            if (stop) {
                if (adjustment > 0) {
                    if (Math.sin(angleRadians) > 0) {
                        rightAdjustment *= 3;
                    } else {
                        leftAdjustment *= 3;
                    }
                } else {
                    if (Math.sin(angleRadians) > 0) {
                        leftAdjustment *= 3;
                    } else {
                        rightAdjustment *= 3;
                    }
                }
            }

            lfPower = wheelPower * Math.cos(angleRadians) + leftAdjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + rightAdjustment;
            lrPower = wheelPower * Math.sin(angleRadians) + leftAdjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + rightAdjustment;


            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            if (xCorrectFactor == 0) {
                distanceToX = 0;
            } else {
                distanceToX = xInches - currentXInches;
            }
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > highestCurrentSpeed) {
                highestCurrentSpeed = currentSpeed;
            }

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            //distanceToStop = (-4.11) + .346 * currentSpeed + .00969 * Math.pow(currentSpeed, 2);
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217 * Math.pow(currentSpeed, 2)) + 0.00093 * Math.pow(currentSpeed, 3) - (1.23e-05 * Math.pow(currentSpeed, 4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            DbgLog.msg(
                    "10435 driveToPoint3"
                            + " XPos:" + String.format("%6.2f", currentXInches)
                            + " YPos:" + String.format("%6.2f", currentYInches)
                            + " distanceToX:" + String.format("%6.2f", distanceToX)
                            + " distanceToY:" + String.format("%6.2f", distanceToY)
                            + " Wheel Power:" + String.format("%6.2f", wheelPower)
                            + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
                            + " angleradianDegrees:" + String.format("%7.2f", Math.toDegrees(angleRadians + Math.PI / 4))
                            + " currentSpeed:" + String.format("%6.2f", currentSpeed)
                            + " adjustment:" + String.format("%6.2f", adjustment)
                            + " left adjustment:" + String.format("%6.2f", leftAdjustment)
                            + " right adjustment:" + String.format("%6.2f", rightAdjustment)
                            + " Left Front Power: " + String.format("%6.2f", lfPower)
                            + " Right Front Power: " + String.format("%6.2f", rfPower)
                            + " distance to stop:" + String.format("%6.2f", distanceToStop)
                            + " current heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
                            + " stop" + stop
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
        telemetry.addData("distance remaining: ", distanceToY);
        telemetry.addData("Power distance: ", yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
    }

    public void driveToPoint3Grabbers(double xInches, double yInches, double heading, double maxWheelPower, double xCorrectFactor, double inchesToGrab) {

        DbgLog.msg(
                "10435-starting driveToPoint3Grabbers"
                        + " X:" + String.format("%6.2f", xInches)
                        + " Y:" + String.format("%6.2f", yInches)
                        + " Heading:" + String.format("%7.2f", heading)
                        + " maxWheelPower:" + String.format("%6.2f", maxWheelPower)
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double highestCurrentSpeed = 0;
        double distanceToStop;

        double speedIncrease = .15;
        double wheelPower = .15; //Minimum speed we start at

        boolean stop = false;

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;

        gsFirstRun = true;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        /*
        if (xCorrectFactor == 0) {
            xCorrectFactor = 1;
        }
        */

        DbgLog.msg(
                "10435 driveToPoint3Grabbers"
                        + " XPos:" + String.format("%6.2f", currentXInches)
                        + " YPos:" + String.format("%6.2f", currentYInches)
                        + " Wheel Power:" + String.format("%6.2f", wheelPower)
                        + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING || currentSpeed > GlobalPositions.DTP_SPEED_SENSITIVITY /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1) {


            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop) {
                if (currentSpeed > 30) {
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);

            if (adjustment > maxWheelPower) {
                adjustment = maxWheelPower;
            } else if (adjustment < -maxWheelPower) {
                adjustment = -maxWheelPower;
            }

            lfPower = wheelPower * Math.cos(angleRadians) - adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) - adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + adjustment;

            if (distanceRemaining < inchesToGrab) {
                grabbers.down();
            }

            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            if (xCorrectFactor == 0) {
                distanceToX = 0;
            } else {
                distanceToX = xInches - currentXInches;
            }
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > highestCurrentSpeed) {
                highestCurrentSpeed = currentSpeed;
            }

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            //distanceToStop = (-4.11) + .346 * currentSpeed + .00969 * Math.pow(currentSpeed, 2);
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217 * Math.pow(currentSpeed, 2)) + 0.00093 * Math.pow(currentSpeed, 3) - (1.23e-05 * Math.pow(currentSpeed, 4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            DbgLog.msg(
                    "10435 driveToPoint3Grabbers"
                            + " XPos:" + String.format("%6.2f", currentXInches)
                            + " YPos:" + String.format("%6.2f", currentYInches)
                            + " distanceToX:" + String.format("%6.2f", distanceToX)
                            + " distanceToY:" + String.format("%6.2f", distanceToY)
                            + " Wheel Power:" + String.format("%6.2f", wheelPower)
                            + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
                            + " angleradianDegrees:" + String.format("%7.2f", Math.toDegrees(angleRadians + Math.PI / 4))
                            + " currentSpeed:" + String.format("%6.2f", currentSpeed)
                            + " adjustment:" + String.format("%6.2f", adjustment)
                            + " current heading:" + String.format("%7.2f", localizer.getCurrentHeadingDegrees())
                            + " distance to stop:" + String.format("%6.2f", distanceToStop)
                            + " stop" + stop
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
        telemetry.addData("distance remaining: ", distanceToY);
        telemetry.addData("Power distance: ", yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
    }

    public void driveToPoint3SpinStone(double xInches, double yInches, double heading, double maxWheelPower, double xCorrectFactor, double spinnerUpDistance) {

        DbgLog.msg(
                "10435-starting driveToPoint3SpinStone"
                        + " X:" + String.format("%6.2f", xInches)
                        + " Y:" + String.format("%6.2f", yInches)
                        + " Heading:" + String.format("%7.2f", heading)
                        + " maxWheelPower:" + String.format("%6.2f", maxWheelPower)
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double highestCurrentSpeed = 0;
        double distanceToStop;

        double speedIncrease = .15;
        double wheelPower = .15; //Minimum speed we start at

        boolean stop = false;

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;

        gsFirstRun = true;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        /*
        if (xCorrectFactor == 0) {
            xCorrectFactor = 1;
        }
        */

        DbgLog.msg(
                "10435 driveToPoint3SpinStone"
                        + " XPos:" + String.format("%6.2f", currentXInches)
                        + " YPos:" + String.format("%6.2f", currentYInches)
                        + " Wheel Power:" + String.format("%6.2f", wheelPower)
                        + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING || currentSpeed > GlobalPositions.DTP_SPEED_SENSITIVITY /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1) {


            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop) {
                if (currentSpeed > 30) {
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);

            if (adjustment > maxWheelPower) {
                adjustment = maxWheelPower;
            } else if (adjustment < -maxWheelPower) {
                adjustment = -maxWheelPower;
            }

            double leftAdjustment = -adjustment;
            double rightAdjustment = adjustment;

            if (stop) {
                if (adjustment > 0) {
                    if (Math.sin(angleRadians) > 0) {
                        rightAdjustment *= 3;
                    } else {
                        leftAdjustment *= 3;
                    }
                } else {
                    if (Math.sin(angleRadians) > 0) {
                        leftAdjustment *= 3;
                    } else {
                        rightAdjustment *= 3;
                    }
                }
            }

            lfPower = wheelPower * Math.cos(angleRadians) + leftAdjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + rightAdjustment;
            lrPower = wheelPower * Math.sin(angleRadians) + leftAdjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + rightAdjustment;

            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            if (xCorrectFactor == 0) {
                distanceToX = 0;
            } else {
                distanceToX = xInches - currentXInches;
            }
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > highestCurrentSpeed) {
                highestCurrentSpeed = currentSpeed;
            }

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            //distanceToStop = (-4.11) + .346 * currentSpeed + .00969 * Math.pow(currentSpeed, 2);
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217 * Math.pow(currentSpeed, 2)) + 0.00093 * Math.pow(currentSpeed, 3) - (1.23e-05 * Math.pow(currentSpeed, 4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            if (distanceRemaining < spinnerUpDistance && spinnerUpDistance > 0) {
                liftSystem.setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_UP);
                spinnerUpDistance = 0;
            }

            DbgLog.msg(
                    "10435 driveToPoint3SpinStone"
                            + " XPos:" + String.format("%6.2f", currentXInches)
                            + " YPos:" + String.format("%6.2f", currentYInches)
                            + " distanceToX:" + String.format("%6.2f", distanceToX)
                            + " distanceToY:" + String.format("%6.2f", distanceToY)
                            + " Wheel Power:" + String.format("%6.2f", wheelPower)
                            + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
                            + " angleradianDegrees:" + String.format("%6.2f", Math.toDegrees(angleRadians + Math.PI / 4))
                            + " currentSpeed:" + String.format("%6.2f", currentSpeed)
                            + " adjustment:" + String.format("%6.2f", adjustment)
                            + " left adjustment:" + String.format("%6.2f", leftAdjustment)
                            + " right adjustment:" + String.format("%6.2f", rightAdjustment)
                            + " Left Front Power: " + String.format("%6.2f", lfPower)
                            + " Right Front Power: " + String.format("%6.2f", rfPower)
                            + " Left Front Power: " + String.format("%6.2f", lfPower)
                            + " Right Front Power: " + String.format("%6.2f", rfPower)
                            + " current heading:" + String.format("%6.2f", localizer.getCurrentHeadingDegrees())
                            + " distance to stop:" + String.format("%6.2f", distanceToStop)
                            + " stop" + stop
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
        telemetry.addData("distance remaining: ", distanceToY);
        telemetry.addData("Power distance: ", yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
    }

    public void driveToPoint3Intake(double xInches, double yInches, double heading, double maxWheelPower, double xCorrectFactor, double intakeFastDistance) {

        DbgLog.msg(
                "10435-starting driveToPointIntake"
                        + " X:" + String.format("%6.2f", xInches)
                        + " Y:" + String.format("%6.2f", yInches)
                        + " Heading:" + String.format("%7.2f", heading)
                        + " maxWheelPower:" + String.format("%6.2f", maxWheelPower)
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double highestCurrentSpeed = 0;
        double distanceToStop;

        double speedIncrease = .15;
        double wheelPower = .15; //Minimum speed we start at

        boolean stop = false;

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;

        gsFirstRun = true;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        /*
        if (xCorrectFactor == 0) {
            xCorrectFactor = 1;
        }
        */

        DbgLog.msg(
                "10435 driveToPointIntake"
                        + " XPos:" + String.format("%6.2f", currentXInches)
                        + " YPos:" + String.format("%6.2f", currentYInches)
                        + " Wheel Power:" + String.format("%6.2f", wheelPower)
                        + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING || currentSpeed > GlobalPositions.DTP_SPEED_SENSITIVITY /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1) {


            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop) {
                if (currentSpeed > 30) {
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading) * wheelPower / .5;

            if (adjustment > maxWheelPower) {
                adjustment = maxWheelPower;
            } else if (adjustment < -maxWheelPower) {
                adjustment = -maxWheelPower;
            }

            lfPower = wheelPower * Math.cos(angleRadians) - adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) - adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + adjustment;


            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            if (xCorrectFactor == 0) {
                distanceToX = 0;
            } else {
                distanceToX = xInches - currentXInches;
            }
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > highestCurrentSpeed) {
                highestCurrentSpeed = currentSpeed;
            }

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            //distanceToStop = (-4.11) + .346 * currentSpeed + .00969 * Math.pow(currentSpeed, 2);
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217 * Math.pow(currentSpeed, 2)) + 0.00093 * Math.pow(currentSpeed, 3) - (1.23e-05 * Math.pow(currentSpeed, 4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            if (distanceRemaining < intakeFastDistance && intakeFastDistance > 0) {
                intake.on();
                intakeFastDistance = 0;
            }

            DbgLog.msg(
                    "10435 driveToPoint3Intake"
                            + " XPos:" + String.format("%6.2f", currentXInches)
                            + " YPos:" + String.format("%6.2f", currentYInches)
                            + " distanceToX:" + String.format("%6.2f", distanceToX)
                            + " distanceToY:" + String.format("%6.2f", distanceToY)
                            + " Wheel Power:" + String.format("%6.2f", wheelPower)
                            + " Distance remaining:" + String.format("%6.2f", distanceRemaining)
                            + " angleradianDegrees:" + String.format("%6.2f", Math.toDegrees(angleRadians + Math.PI / 4))
                            + " currentSpeed:" + String.format("%6.2f", currentSpeed)
                            + " adjustment:" + String.format("%6.2f", adjustment)
                            + " current heading:" + String.format("%6.2f", localizer.getCurrentHeadingDegrees())
                            + " distance to stop:" + String.format("%6.2f", distanceToStop)
                            + " stop" + stop
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
        telemetry.addData("distance remaining: ", distanceToY);
        telemetry.addData("Power distance: ", yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
    }

    public void driveToPointU(double xInches, double yInches, double heading, double maxWheelPower, double xCorrectFactor, double secondXinches, double secondXDistanceRemaining) {

        DbgLog.msg(
                "10435-starting driveToPoint3"
                        + " X:" + xInches
                        + " Y:" + yInches
                        + " Heading:" + heading
                        + " maxWheelPower:" + maxWheelPower
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double highestCurrentSpeed = 0;
        double distanceToStop;

        double speedIncrease = .15;
        double wheelPower = .15; //Minimum speed we start at

        boolean stop = false;

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;

        gsFirstRun = true;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        DbgLog.msg(
                "10435-driveToPoint3"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING || currentSpeed > GlobalPositions.DTP_SPEED_SENSITIVITY /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1) {


            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop) {
                if (currentSpeed > 30) {
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);  // adjustment curve seems very suited for very slow speeds. more adjustment at higher wheel powers

            if (adjustment > maxWheelPower) {
                adjustment = maxWheelPower;
            } else if (adjustment < -maxWheelPower) {
                adjustment = -maxWheelPower;
            }

            double leftAdjustment = -adjustment;
            double rightAdjustment = adjustment;

            if (stop) {
                if (adjustment > 0) {
                    if (Math.sin(angleRadians) > 0) {
                        rightAdjustment *= 3;
                    } else {
                        leftAdjustment *= 3;
                    }
                } else {
                    if (Math.sin(angleRadians) > 0) {
                        leftAdjustment *= 3;
                    } else {
                        rightAdjustment *= 3;
                    }
                }
            }

            lfPower = wheelPower * Math.cos(angleRadians) + leftAdjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + rightAdjustment;
            lrPower = wheelPower * Math.sin(angleRadians) + leftAdjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + rightAdjustment;


            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            if (distanceRemaining < secondXDistanceRemaining) {
                xInches = secondXinches;
            }

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            if (xCorrectFactor == 0) {
                distanceToX = 0;
            } else {
                distanceToX = xInches - currentXInches;
            }
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > highestCurrentSpeed) {
                highestCurrentSpeed = currentSpeed;
            }

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }

            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            //distanceToStop = (-4.11) + .346 * currentSpeed + .00969 * Math.pow(currentSpeed, 2);
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217 * Math.pow(currentSpeed, 2)) + 0.00093 * Math.pow(currentSpeed, 3) - (1.23e-05 * Math.pow(currentSpeed, 4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > GlobalPositions.DTP_DISTANCE_REMAINING) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            DbgLog.msg(
                    "10435 driveToPoint3"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " distanceToX:" + distanceToX
                            + " distanceToY:" + distanceToY
                            + " Wheel Power:" + wheelPower
                            + " Distance remaining:" + distanceRemaining
                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
                            + " currentSpeed:" + currentSpeed
                            + " adjustment:" + adjustment
                            + " left adjustment:" + leftAdjustment
                            + " right adjustment:" + rightAdjustment
                            + " Left Front Power: " + lfPower
                            + " Right Front Power: " + rfPower
                            + " distance to stop:" + distanceToStop
                            + " current heading:" + localizer.getCurrentHeadingDegrees()
                            + " stop" + stop
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
        telemetry.addData("distance remaining: ", distanceToY);
        telemetry.addData("Power distance: ", yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
    }

    public void driveToPoint2(double xInches, double yInches, double heading,
                              double speedModifier, double xCorrectFactor) {

        DbgLog.msg(
                "10435-starting driveToPoint2"
                        + " X:" + xInches
                        + " Y:" + yInches
                        + " Heading:" + heading
                        + " SPM:" + speedModifier
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;

        double maxWheelPower;
        double wheelPower = .15; //Minimum speed we start at

        gsFirstRun = true;

        ElapsedTime timeoutTimer = new ElapsedTime();

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        DbgLog.msg(
                "10435 driveToPoint2"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        if (xCorrectFactor == 0) {
            xCorrectFactor = 1;
        }

        while (distanceRemaining > 1 && opModeIsActive() && timeoutTimer.seconds() < 1) {


            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 30) / 100;

            double speedIncrease = .05;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);

            lfPower = wheelPower * Math.cos(angleRadians) - adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) - adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + adjustment;

            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            telemetry.addData("XPos: ", currentXInches);
            telemetry.addData("YPos: ", currentYInches);
            telemetry.addData("Current Speed:", currentSpeed);
            telemetry.addData("Wheel Power: ", wheelPower);
            telemetry.addData("distanceToX: ", distanceToX);
            telemetry.addData("distanceToY: ", distanceToY);
            telemetry.addData("Distance remaining: ", distanceRemaining);
            telemetry.addData("andleradians: ", angleRadians);
            telemetry.addData("angleradianDegrees: ", Math.toDegrees(angleRadians));
            telemetry.update();

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            distanceToX = xInches - currentXInches;
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            DbgLog.msg(
                    "10435 driveToPoint"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " Wheel Power:" + wheelPower
                            + " Distance remaining:" + distanceRemaining
                            + " angleradianDegrees:" + Math.toDegrees(angleRadians)
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
    }

    public void checkCoast(double xInches, double yInches, double heading,
                           double maxWheelPower, double coastingPower) {

        DbgLog.msg(
                "10435-starting checkCoast"
                        + " X:" + xInches
                        + " Y:" + yInches
                        + " Heading:" + heading
        );

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        double currentSpeed;
        double highestCurrentSpeed = 0;

        double wheelPower = .15; //Minimum speed we start at

        gsFirstRun = true;

        ElapsedTime timeoutTimer = new ElapsedTime();

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
        double distanceToX = xInches - currentXInches;
        double distanceToY = yInches - currentYInches;
        currentSpeed = getSpeed(currentXInches, currentYInches);

        double distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the triangle is the remaining distance

        DbgLog.msg(
                "10435 checkCoast"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        while (distanceToY > 1 && opModeIsActive() && timeoutTimer.seconds() < 1) {


            double speedIncrease = .15;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI / 4;

            double adjustment = localizer.headingAdjustment(heading);

            lfPower = wheelPower * Math.cos(angleRadians) - adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) + adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) - adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) + adjustment;

            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);

            telemetry.addData("XPos: ", currentXInches);
            telemetry.addData("YPos: ", currentYInches);
            telemetry.addData("Current Speed:", currentSpeed);
            telemetry.addData("Wheel Power: ", wheelPower);
            telemetry.addData("distanceToX: ", distanceToX);
            telemetry.addData("distanceToY: ", distanceToY);
            telemetry.addData("Distance remaining: ", distanceRemaining);
            telemetry.addData("andleradians: ", angleRadians);
            telemetry.addData("angleradianDegrees: ", Math.toDegrees(angleRadians));
            telemetry.update();

            currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            distanceToX = xInches - currentXInches;
            distanceToY = yInches - currentYInches;

            currentSpeed = getSpeed(currentXInches, currentYInches);

            if (currentSpeed > highestCurrentSpeed) {
                highestCurrentSpeed = currentSpeed;
            }

            if (Math.abs(currentSpeed) > .5) {
                timeoutTimer.reset();
            }
            distanceRemaining = Math.sqrt(Math.pow(distanceToX, 2) + Math.pow(distanceToY, 2));  // hypotenuse of the x and y is the remaining distance

            DbgLog.msg(
                    "10435 checkCoast"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " distanceToX:" + distanceToX
                            + " distanceToY:" + distanceToY
                            + " Wheel Power:" + wheelPower
                            + " Distance remaining:" + distanceRemaining
                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
                            + " currentSpeed:" + currentSpeed
                            + " ajustment:" + adjustment
                            + " current heading:" + localizer.getCurrentHeadingDegrees()
            );
        }
        driveTrain.applyPower(coastingPower, coastingPower, coastingPower, coastingPower);
        telemetry.addData(("Power distance: "), yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
        sleep(3000);
    }

    public void turn_to_heading(double target_heading, double speedModifier) {
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        // This is just for logging
        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();


        DbgLog.msg("10435 Starting TURN_TO_HEADING"
                + " Target Heading:" + String.format("%7.2f", target_heading)
                + " Speed modifier:" + String.format("%6.2f", speedModifier)
        );

        currentHeading = localizer.getCurrentHeadingDegrees();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .5 && opModeIsActive() && timeoutTimer.seconds() < 2) {  // 11/21 changed from .5 to .3

            localizer.drive.update();

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 3) + 15) / 100;
            } else {
                if (speedModifier != 0) {
                    wheelPower = (Math.pow((degreesToTurn) / speedModifier, 4) + 35) / 100;
                } else {
                    wheelPower = (Math.pow((degreesToTurn) / 30, 4) + 15) / 100;
                }
            }

            if (goRight) {
                wheelPower = -wheelPower;
            }

            driveTrain.applyPower(-wheelPower, wheelPower, -wheelPower, wheelPower);

            currentHeading = localizer.getCurrentHeadingDegrees();

            degreesToTurn = Math.abs(target_heading - currentHeading);       // Calculate how far is remaining to turn

            goRight = target_heading > currentHeading;

            if (degreesToTurn > 180) {
                goRight = !goRight;
                degreesToTurn = 360 - degreesToTurn;
            }

            if (Math.abs(currentHeading - prevHeading) > 1) {  // if it has turned at least one degree
                timeoutTimer.reset();
                prevHeading = currentHeading;
            }

        }

        driveTrain.applyPower(0, 0, 0, 0);

        currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
        currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;

        DbgLog.msg("10435 Ending TURN_TO_HEADING"
                + " currentHeading:" + String.format("%7.2f", currentHeading)
                + " Odometer XInches:" + String.format("%6.2f", currentXInches)
                + " Odometer YInches:" + String.format("%6.2f", currentYInches)
        );

        telemetry.addData("Heading: ", currentHeading);
        telemetry.addData("Odometer XInches: ", currentXInches);
        telemetry.addData("Odometer YInches: ", currentYInches);
        telemetry.update();

    } // end of turn_to_heading

    private double getSpeed(double xInches, double yInches) {
        double distanceFromPrevPoint;
        double returnSpeed;

        if (gsFirstRun) {
            gsPreviousXInches = xInches;
            gsPreviousYInches = yInches;
            gsPreviousTime = gsSpeedTimer.seconds();
            gsSpeedTimer.reset();
            gsPreviousSpeed = 0;
            returnSpeed = gsPreviousSpeed;
            gsFirstRun = false;
        } else {
            distanceFromPrevPoint = Math.sqrt(Math.pow((xInches - gsPreviousXInches), 2) + Math.pow((yInches - gsPreviousYInches), 2));
            returnSpeed = distanceFromPrevPoint * 1 / (gsSpeedTimer.seconds() - gsPreviousTime);
            gsPreviousSpeed = returnSpeed;
            gsPreviousXInches = xInches;
            gsPreviousYInches = yInches;
            gsPreviousTime = gsSpeedTimer.seconds();
            //DbgLog.msg("10435 getspeed: " + returnSpeed + " inches/sec");
        }

        return returnSpeed; //inches per second
    }

    private void runServoEvents(int poseNum, double currentXInches, double startingX, double currentYInches, double startingY) {

        int numEvents = servoEventManager.getEvents().size();

        for (int j = 0; j < numEvents; j++) {
            ServoEvent event = servoEventManager.getEvents().get(j);
            int eventType = event.getEventType();

            double yTrigger = event.getYTrigger();
            double xTrigger = event.getXTrigger();

/*
            DbgLog.msg("10435 runServoEvents: "
                    + " Pose num:" + event.getPose2dEventNumber()
                    + " Event num:" + eventNum
                    + " j:" + j
                    + " Event type:" + eventType
                    + " X Trigger:" + xTrigger
                    + " Current X:" + currentXInches
                    + " Y Trigger:" + yTrigger
                    + " Current Y:" + currentYInches
            );
*/
            boolean triggerHit = false;

            if (!event.getEventCompleted()) {
                triggerHit = (event.getPose2dEventNumber() < poseNum);  // Runs all events that didn't trigger in previous pose
                if (event.getPose2dEventNumber() == poseNum) {
                    if (yTrigger != 0 && xTrigger != 0) {
                        triggerHit = event.checkYPos(currentYInches, startingY) && event.checkXPos(currentXInches, startingX);
                    } else if (yTrigger != 0) {
                        triggerHit = event.checkYPos(currentYInches, startingY);
                    } else if (xTrigger != 0) {
                        triggerHit = event.checkXPos(currentXInches, startingX);
                    }
                }
            }

            if (triggerHit) {
                switch (eventType) {
                    case ServoEvent.SE_GRABBERS_READY:
                        grabbers.ready();
                        DbgLog.msg("10435 runServoEvents: Ran grabbers.ready");
                        break;
                    case ServoEvent.SE_GRABBERS_DOWN:
                        grabbers.down();
                        DbgLog.msg("10435 runServoEvents: Ran grabbers.down");
                        break;
                    case ServoEvent.SE_GRABBERS_UP:
                        grabbers.up();
                        DbgLog.msg("10435 runServoEvents: Ran grabbers.up");
                        break;
                    case ServoEvent.SE_GRAB_STONE:
                        liftSystem.grabStone();
                        DbgLog.msg("10435 runServoEvents: Ran grabStone");
                        break;
                    case ServoEvent.SE_DROP_STONE:
                        liftSystem.dropStone();
                        DbgLog.msg("10435 runServoEvents: Ran dropStone");
                        break;
                    case ServoEvent.SE_EXTEND_HLIFT_FAR:
                        liftSystem.setHLiftPos(1);
                        DbgLog.msg("10435 runServoEvents: Ran setHLiftPos(1)");
                        break;
                    case ServoEvent.SE_EXTEND_HLIFT_CLOSE:
                        liftSystem.setHLiftPos(.75);
                        DbgLog.msg("10435 runServoEvents: Ran setHLiftPos(.75)");
                        break;
                    case ServoEvent.SE_ROTATE_STONE_180:
                        liftSystem.setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_UP);
                        DbgLog.msg("10435 runServoEvents: Ran setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_UP)");
                        break;
                    case ServoEvent.SE_RETRACT_HLIFT:
                        liftSystem.setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_DOWN);
                        liftSystem.setHLiftPos(GlobalPositions.MIN_HLIFT_POS);
                        DbgLog.msg("10435 runServoEvents: Ran setStoneSpinnerPos(GlobalPositions.STONE_SPINNER_DOWN) and setHLiftPos(GlobalPositions.MIN_HLIFT_POS");
                        break;
                    case ServoEvent.SE_INTAKE_ON:
                        intake.on();
                        DbgLog.msg("10435 runServoEvents: Ran intake.on()");
                        break;
                    case ServoEvent.SE_INTAKE_ON_SLOW:
                        intake.onSlow();
                        DbgLog.msg("10435 runServoEvents: Ran intake.onSlow()");
                        break;
                    case ServoEvent.SE_INTAKE_OFF:
                        intake.off();
                        DbgLog.msg("10435 runServoEvents: Ran intake.off()");
                        break;
                    case ServoEvent.SE_LEFT_CLAW_CLOSE:
                        sideGrabbers.clampLeftClaw();
                        DbgLog.msg("10435 runServoEvents: Ran clampLeftClaw()");
                        break;
                    case ServoEvent.SE_LEFT_CLAW_OPEN:
                        sideGrabbers.openLeftClaw();
                        DbgLog.msg("10435 runServoEvents: Ran openLeftClaw()");
                        break;
                    case ServoEvent.SE_LEFT_CLAW_LOWER:
                        sideGrabbers.lowerLeftClaw();
                        DbgLog.msg("10435 runServoEvents: Ran lowerLeftClaw()");
                        break;
                    case ServoEvent.SE_LEFT_CLAW_RAISE:
                        sideGrabbers.raiseLeftClaw();
                        DbgLog.msg("10435 runServoEvents: Ran raiseLeftClaw()");
                        break;
                    case ServoEvent.SE_LEFT_CLAW_PIVOT_PLACE:
                        sideGrabbers.placeLeftClaw();
                        DbgLog.msg("10435 runServoEvents: Ran placeLeftClaw()");
                        break;
                    case ServoEvent.SE_LEFT_CLAW_PIVOT_PLACE_HIGH:
                        sideGrabbers.placeLeftClawHigh();
                        DbgLog.msg("10435 runServoEvents: Ran placeLeftClawHigh()");
                        break;
                    case ServoEvent.SE_RIGHT_CLAW_CLOSE:
                        sideGrabbers.clampRightClaw();
                        DbgLog.msg("10435 runServoEvents: Ran clampRightClaw()");
                        break;
                    case ServoEvent.SE_RIGHT_CLAW_OPEN:
                        sideGrabbers.openRightClaw();
                        DbgLog.msg("10435 runServoEvents: Ran openRightClaw()");
                        break;
                    case ServoEvent.SE_RIGHT_CLAW_LOWER:
                        sideGrabbers.lowerRightClaw();
                        DbgLog.msg("10435 runServoEvents: Ran lowerRightClaw()");
                        break;
                    case ServoEvent.SE_RIGHT_CLAW_RAISE:
                        sideGrabbers.raiseRightClaw();
                        DbgLog.msg("10435 runServoEvents: Ran raiseRightClaw()");
                        break;
                    case ServoEvent.SE_RIGHT_CLAW_PIVOT_PLACE:
                        sideGrabbers.placeRightClaw();
                        DbgLog.msg("10435 runServoEvents: Ran placeRightClaw()");
                        break;
                    case ServoEvent.SE_RIGHT_CLAW_PIVOT_PLACE_HIGH:
                        sideGrabbers.placeRightClawHigh();
                        DbgLog.msg("10435 runServoEvents: Ran placeRightClawHigh()");
                        break;
                    default:
                        DbgLog.msg("10435 runServoEvents: Case not found for eventType " + eventType);
                        break;
                }
                event.setEventCompleted(true);
                int sleepMs = event.getSleepAfter();
                if (sleepMs > 0) {
                    sleep(sleepMs);
                }
            }
        }

    }

    private void logServoEvents() {

        int numEvents = servoEventManager.getEvents().size();

        for (int j = 0; j < numEvents; j++) {
            try {
                ServoEvent event = servoEventManager.getEvents().get(j);
                DbgLog.msg("10435 logServoEvents: "
                        + "   Pose event num:" + String.format("%2d", event.getPose2dEventNumber())
                        + "   Event Type:" + String.format("%3d", event.getEventType())
                        + "   Completed:" + String.format("%5b", event.getEventCompleted())
                        + "   X Trigger:" + String.format("%5.1f", event.getXTrigger())
                        + "   Y Trigger:" + String.format("%5.1f", event.getYTrigger())
                        + "   Sleep:" + String.format("%4d", event.getSleepAfter())
                        + " --j:" + String.format("%3d", j)
                );
            } catch (Exception e) {
                DbgLog.msg("10435 logServoEvents: no event number j:" + j + " numEvents:" + numEvents);
            }
        }
    }

}
