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

    boolean gsFirstRun = true;
    ElapsedTime gsSpeedTimer = new ElapsedTime();

    public void roboInit() {
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rr = hardwareMap.dcMotor.get("rr");
        DcMotor intakeL = hardwareMap.dcMotor.get("il");
        DcMotor intakeR = hardwareMap.dcMotor.get("ir");
        DcMotor vLift = hardwareMap.dcMotor.get("vl");
        DcMotor hLift = hardwareMap.dcMotor.get("hl");
        Servo stoneGrabber = hardwareMap.servo.get("sg");
        Servo stoneSpinner = hardwareMap.servo.get("ss");
        Servo hookL = hardwareMap.servo.get("hkl");
        Servo hookR = hardwareMap.servo.get("hkr");
        DcMotor xOdom = hardwareMap.dcMotor.get("ir");
        DcMotor yOdom = hardwareMap.dcMotor.get("il");
        Servo capstonePost = hardwareMap.servo.get("cp");

        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        imu = new IMU(gyro);
        imu.initialize();

        stoneGrabber.setPosition(0);
        stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        //hookL.setPosition(1);
        //hookR.setPosition(0);
        capstonePost.setPosition(1);

        vuforiaStuff = new VuforiaStuff(vuforia);
        driveTrain = new DriveTrain(lf, rf, lr, rr);
        intake = new Intake(intakeL, intakeR);
        grabbers = new FoundationGrabbers(hookL, hookR);
        odometers = new Odometers(xOdom, yOdom);
        liftSystem = new LiftSystem(vLift,hLift,stoneGrabber,stoneSpinner);
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

        while (distanceRemaining > 1 && opModeIsActive() && timeoutTimer.seconds() < 1) {

            maxWheelPower = (Math.pow(distanceRemaining / speedModifier, 3) + 30) / 100;

            double speedIncrease = .05;

            wheelPower += speedIncrease;
            if (Math.abs(wheelPower) > Math.abs(maxWheelPower)) {
                wheelPower = maxWheelPower;
            }

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI/4;

            double adjustment = -imu.headingAdjustment(heading);

            lfPower = wheelPower * Math.cos(angleRadians) + adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) - adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) + adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) - adjustment;

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

            if (Math.abs(currentSpeed) > .5){
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
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;
        double prevheading = 0;
        ElapsedTime timeouttimer = new ElapsedTime();

        DbgLog.msg("10435 Starting TURN_TO_HEADING");
        current_heading = imu.readCurrentHeading();
        degrees_to_turn = Math.abs(target_heading - current_heading);

        go_right = target_heading > current_heading;

        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeouttimer.reset();
        prevheading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && timeouttimer.seconds() < 2) {

            if (speedModifier != 0){
                wheel_power = (Math.pow((degrees_to_turn) / speedModifier, 4) + 35) / 100;
            } else {
                wheel_power = (Math.pow((degrees_to_turn) / 30, 4) + 15) / 100;
            }

            if (go_right) {
                wheel_power = -wheel_power;
            }

            driveTrain.applyPower(-wheel_power, wheel_power, -wheel_power, wheel_power);

            current_heading = imu.readCurrentHeading();

            degrees_to_turn = Math.abs(target_heading - current_heading);       // Calculate how far is remaining to turn

            go_right = target_heading > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prevheading) > 1) {
                timeouttimer.reset();
                prevheading = current_heading;
            }

        }

        driveTrain.applyPower(0, 0, 0, 0);

    } // end of turn_to_heading

    private double getSpeed(double xInches, double yInches) {
        double distanceFromPrevPoint;
        double timerInterval = .1;  //seconds
        double returnSpeed;

        if (gsFirstRun) {
            gsPreviousXInches = xInches;
            gsPreviousYInches = yInches;
            gsSpeedTimer.reset();
            gsPreviousSpeed = 0;
            returnSpeed = gsPreviousSpeed;
            gsFirstRun = false;
        }

        if (gsSpeedTimer.seconds() >= timerInterval) {
            gsSpeedTimer.reset();
            distanceFromPrevPoint = Math.sqrt(Math.pow((xInches - gsPreviousXInches), 2) + Math.pow((yInches - gsPreviousYInches), 2));
            returnSpeed = distanceFromPrevPoint * 1.0/timerInterval;
            gsPreviousSpeed = returnSpeed;
            gsPreviousXInches = xInches;
            gsPreviousYInches = yInches;
            DbgLog.msg("10435 getspeed: " + returnSpeed + " inches/sec");
        } else {
            returnSpeed = gsPreviousSpeed; // Too early to check new speed, so return the previous speed
        }

        return returnSpeed; //inches per second
    }

}
