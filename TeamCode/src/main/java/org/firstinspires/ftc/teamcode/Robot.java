package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.FoundationGrabbers;
import org.firstinspires.ftc.teamcode.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.LiftSystem;
import org.firstinspires.ftc.teamcode.Hardware.Odometers;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.openftc.revextensions2.ExpansionHubMotor;

abstract public class Robot extends LinearOpMode {
    public DriveTrain driveTrain;
    public Intake intake;
    public FoundationGrabbers grabbers;
    public Odometers odometers;
    public VuforiaStuff vuforiaStuff;
    public LiftSystem liftSystem;
    public IMU imu;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";
    BNO055IMU gyro;

    double gsPreviousSpeed;
    double gsPreviousXInches;
    double gsPreviousYInches;
    double gsPreviousTime;

    boolean gsFirstRun = true;
    ElapsedTime gsSpeedTimer = new ElapsedTime();

    public SampleMecanumDriveBase drive;

    public void roboInit() {
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rr = hardwareMap.dcMotor.get("rr");
        DcMotor intakeL = hardwareMap.dcMotor.get("il");
        DcMotor intakeR = hardwareMap.dcMotor.get("ir");
        Servo hLift = hardwareMap.servo.get("hl");
        Servo stoneGrabber = hardwareMap.servo.get("sg");
        Servo stoneSpinner = hardwareMap.servo.get("ss");
        Servo hookL = hardwareMap.servo.get("hkl");
        Servo hookR = hardwareMap.servo.get("hkr");
        ExpansionHubMotor leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "il");
        ExpansionHubMotor rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "ir");
        ExpansionHubMotor frontEncoder = hardwareMap.get(ExpansionHubMotor.class,"rr");
        Servo capstonePost = hardwareMap.servo.get("cp");

        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        drive = new SampleMecanumDriveREVOptimized(hardwareMap);

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        imu = new IMU(gyro);
        imu.initialize();

        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_UP);
        stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        hookL.setPosition(GlobalPositions.HOOKL_UP);
        hookR.setPosition(GlobalPositions.HOOKR_UP);
        capstonePost.setPosition(GlobalPositions.CAPSTONE_START);

        vuforiaStuff = new VuforiaStuff(vuforia);
        driveTrain = new DriveTrain(lf, rf, lr, rr);
        intake = new Intake(intakeL, intakeR);
        grabbers = new FoundationGrabbers(hookL, hookR);
        odometers = new Odometers(frontEncoder, leftEncoder, rightEncoder);
        liftSystem = new LiftSystem(hLift, stoneGrabber, stoneSpinner);
        grabbers.up();

        waitForStart();
    }

    public void driveToPoint(double xInches, double yInches, double heading, double speedModifier) {

        DbgLog.msg(
                "10435-starting driveToPoint"
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
                "10435 driveToPoint"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        while ((distanceRemaining > 1 || currentSpeed > 3) && opModeIsActive() && timeoutTimer.seconds() < .75) {

            liftSystem.runLift();

            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 25) / 100;

            double speedIncrease = .15;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI / 4;

            double adjustment = imu.headingAdjustment(heading);

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
                            + " distanceToX:" + distanceToX
                            + " distanceToY:" + distanceToY
                            + " Wheel Power:" + wheelPower
                            + " Distance remaining:" + distanceRemaining
                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
                            + " currentSpeed:" + currentSpeed
                            + " ajustment:" + adjustment
                            + " current heading:" + imu.readCurrentHeading()
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
    }

    public void driveToPoint3(double xInches, double yInches, double heading, double maxWheelPower, double xCorrectFactor) {

        DbgLog.msg(
                "10435-starting driveToPoint"
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

        /*
        if (xCorrectFactor == 0) {
            xCorrectFactor = 1;
        }
        */

        DbgLog.msg(
                "10435 driveToPoint"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > 1 || currentSpeed > 7 /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1 ) {

            liftSystem.runLift();

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop){
                if (currentSpeed > 30){
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = imu.headingAdjustment(heading);

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
            if (xCorrectFactor == 0){
                distanceToX = 0;
            }else {
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
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217*Math.pow(currentSpeed,2)) + 0.00093*Math.pow(currentSpeed,3) - (1.23e-05*Math.pow(currentSpeed,4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > 1) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            DbgLog.msg(
                    "10435 driveToPoint"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " distanceToX:" + distanceToX
                            + " distanceToY:" + distanceToY
                            + " Wheel Power:" + wheelPower
                            + " Distance remaining:" + distanceRemaining
                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
                            + " currentSpeed:" + currentSpeed
                            + " adjustment:" + adjustment
                            + " current heading:" + imu.readCurrentHeading()
                            + " distance to stop:" + distanceToStop
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
                "10435-starting driveToPoint"
                        + " X:" + xInches
                        + " Y:" + yInches
                        + " Heading:" + heading
                        + " maxWheelPower:" + maxWheelPower
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
                "10435 driveToPoint"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        ElapsedTime timeoutTimer = new ElapsedTime();

        while ((distanceRemaining > 1 || currentSpeed > 7 /*|| degreesToTurn > .5*/) && opModeIsActive() && timeoutTimer.seconds() < 1 ) {

            liftSystem.runLift();

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            if (stop){
                if (currentSpeed > 30){
                    wheelPower = .08;
                } else {
                    wheelPower = .04;
                }
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = imu.headingAdjustment(heading);

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
            if (xCorrectFactor == 0){
                distanceToX = 0;
            }else {
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
            distanceToStop = 0.994 + 0.0111 * currentSpeed - (0.00217*Math.pow(currentSpeed,2)) + 0.00093*Math.pow(currentSpeed,3) - (1.23e-05*Math.pow(currentSpeed,4));

            if (distanceRemaining <= distanceToStop && distanceRemaining > 1) {
                stop = true;
                speedIncrease = .01;
            } else {
                stop = false;
            }

            if (distanceRemaining < spinnerUpDistance && spinnerUpDistance > 0) {
                liftSystem.stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
                spinnerUpDistance = 0;
            }

            DbgLog.msg(
                    "10435 driveToPoint3Extend"
                            + " XPos:" + currentXInches
                            + " YPos:" + currentYInches
                            + " distanceToX:" + distanceToX
                            + " distanceToY:" + distanceToY
                            + " Wheel Power:" + wheelPower
                            + " Distance remaining:" + distanceRemaining
                            + " angleradianDegrees:" + Math.toDegrees(angleRadians + Math.PI / 4)
                            + " currentSpeed:" + currentSpeed
                            + " adjustment:" + adjustment
                            + " current heading:" + imu.readCurrentHeading()
                            + " distance to stop:" + distanceToStop
                            + " stop" + stop
            );
        }
        driveTrain.applyPower(0, 0, 0, 0);
        telemetry.addData("distance remaining: ", distanceToY);
        telemetry.addData("Power distance: ", yInches);
        telemetry.addData("Highest Speed: ", highestCurrentSpeed);
        telemetry.update();
    }

    public void driveToPoint2(double xInches, double yInches, double heading, double speedModifier, double xCorrectFactor) {

        DbgLog.msg(
                "10435-starting driveToPoint"
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
                "10435 driveToPoint"
                        + " XPos:" + Double.toString(currentXInches)
                        + " YPos:" + Double.toString(currentYInches)
                        + " Wheel Power:" + Double.toString(wheelPower)
                        + " Distance remaining:" + Double.toString(distanceRemaining)
        );

        if (xCorrectFactor == 0) {
            xCorrectFactor = 1;
        }

        while (distanceRemaining > 1 && opModeIsActive() && timeoutTimer.seconds() < 1) {

            liftSystem.runLift();

            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 30) / 100;

            double speedIncrease = .05;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX * xCorrectFactor) - Math.PI / 4;

            double adjustment = imu.headingAdjustment(heading);

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
                + " Target Heading:" + target_heading
                + " Speed modifier:" + speedModifier
         );

        currentHeading = imu.readCurrentHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .5 && opModeIsActive() && timeoutTimer.seconds() < 2) {  // 11/21 changed from .5 to .3

            liftSystem.runLift();

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

            currentHeading = imu.readCurrentHeading();

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
                + " currentHeading:" + currentHeading
                + " Odometer XInches:" + currentXInches
                + " Odometer YInches:" + currentYInches
        );

        telemetry.addData("Heading: ", currentHeading);
        telemetry.addData("Odometer XInches: ", currentXInches);
        telemetry.addData("Odometer YInches: ", currentYInches);
        telemetry.update();

    } // end of turn_to_heading

    public void turn_to_heading2(double target_heading, double speedModifier, double xInches, double yInches) {
        boolean goRight;
        double currentHeading;
        double degreesToTurn;
        double wheelPower;
        double prevHeading = 0;
        ElapsedTime timeoutTimer = new ElapsedTime();

        double wheel_encoder_ticks = 2400;
        double wheel_diameter = 2.3622;  // size of wheels
        double ticks_per_inch = wheel_encoder_ticks / (wheel_diameter * Math.PI);

        double currentXInches;
        double currentYInches;
        double startXPos = odometers.getXPos();
        double startYPos = odometers.getYPos();

        DbgLog.msg("10435 Starting TURN_TO_HEADING"
                + " Target Heading:" + target_heading
                + " Speed modifier:" + speedModifier
        );

        currentHeading = imu.readCurrentHeading();
        degreesToTurn = Math.abs(target_heading - currentHeading);

        goRight = target_heading > currentHeading;

        if (degreesToTurn > 180) {
            goRight = !goRight;
            degreesToTurn = 360 - degreesToTurn;
        }

        timeoutTimer.reset();
        prevHeading = currentHeading;
        while (degreesToTurn > .3 && opModeIsActive() && timeoutTimer.seconds() < 2) {  // 11/21 changed from .5 to .3

            liftSystem.runLift();

            if (speedModifier < 0) {
                wheelPower = (Math.pow((degreesToTurn + 25) / -speedModifier, 2.2) + 15) / 100;
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

            //currentXInches = (odometers.getXPos() - startXPos) / ticks_per_inch;
            currentYInches = (odometers.getYPos() - startYPos) / ticks_per_inch;
            double inchesOff = currentYInches - yInches;
            double wheelPowerAdjust = 0; //= (Math.pow(Math.abs(inchesOff) / 15, 3) + 0) / 100;
            if (inchesOff > .2) {
                wheelPowerAdjust = -.05;
            } else if (inchesOff < -.2) {
                wheelPowerAdjust = .05;
            }

            driveTrain.applyPower(-wheelPower - wheelPowerAdjust, wheelPower - wheelPowerAdjust, -wheelPower - wheelPowerAdjust, wheelPower - wheelPowerAdjust);

            currentHeading = imu.readCurrentHeading();

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
                + " currentHeading:" + currentHeading
                + " Odometer XInches:" + currentXInches
                + " Odometer YInches:" + currentYInches
        );

        telemetry.addData("Heading: ", currentHeading);
        telemetry.addData("Odometer XInches: ", currentXInches);
        telemetry.addData("Odometer YInches: ", currentYInches);
        telemetry.update();

    } // end of turn_to_heading2

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
            DbgLog.msg("10435 getspeed: " + returnSpeed + " inches/sec");
        }

        return returnSpeed; //inches per second
    }
}
