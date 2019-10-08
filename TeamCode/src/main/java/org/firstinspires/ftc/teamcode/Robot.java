package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.PID;
import org.firstinspires.ftc.teamcode.Autonomous.RoboPosition;
import org.firstinspires.ftc.teamcode.Autonomous.VuforiaStuff;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.IMU;
import org.firstinspires.ftc.teamcode.Hardware.Odometers;

import static org.firstinspires.ftc.teamcode.Autonomous.RoboPosition.currentXPos;
import static org.firstinspires.ftc.teamcode.Autonomous.RoboPosition.currentYPos;

abstract public class Robot extends LinearOpMode {
    public DriveTrain driveTrain;
    public Odometers odometers;
    public VuforiaStuff vuforiaStuff;
    public TFObjectDetector tfod;
    public IMU imu;
    private VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = "AWaEPBn/////AAAAGWa1VK57tkUipP01PNk9ghlRuxjK1Oh1pmbHuRnpaJI0vi57dpbnIkpee7J1pQ2RIivfEFrobqblxS3dKUjRo52NMJab6Me2Yhz7ejs5SDn4G5dheW5enRNWmRBsL1n+9ica/nVjG8xvGc1bOBRsIeZyL3EZ2tKSJ407BRgMwNOmaLPBle1jxqAE+eLSoYsz/FuC1GD8c4S3luDm9Utsy/dM1W4dw0hDJFc+lve9tBKGBX0ggj6lpo9GUrTC8t19YJg58jsIXO/DiF09a5jlrTeB2LK+GndUDEGyZA1mS3yAR6aIBeDYnFw+79mVFIkTPk8wv3HIQfzoggCu0AwWJBVUVjkDxJOWfzCGjaHylZlo";
    BNO055IMU gyro;

    public void roboInit() {
        /*
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor lr = hardwareMap.dcMotor.get("lr");
        DcMotor rr = hardwareMap.dcMotor.get("rr");
        DcMotor xOdom = hardwareMap.dcMotor.get("lf");
        DcMotor yOdom = hardwareMap.dcMotor.get("rf");
         */
        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.maxWebcamAspectRatio = 640/480;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        imu = new IMU(gyro);
        imu.initialize();

        vuforiaStuff = new VuforiaStuff(vuforia);
        //driveTrain = new DriveTrain(lf, rf, lr, rr);
        //odometers = new Odometers(xOdom, yOdom);

        if (tfod != null) {
            tfod.activate();
        }

        waitForStart();
    }

    public void driveToPoint(double x, double y, double heading) {
        PID pid = new PID(x, y, .006, .006, .005);

        RoboPosition.currentXPos = odometers.getXPos();
        RoboPosition.currentYPos = odometers.getYPOs();
        double distanceToX = x - RoboPosition.currentXPos;
        double distanceToY = y - RoboPosition.currentYPos;

        double[] vector = pid.calcErrors(distanceToX, distanceToY);
        double lfPower;
        double rfPower;
        double lrPower;
        double rrPower;

        //while (vector[0] != 0 && vector[1] != 0) {
        while (Math.abs(distanceToX) > 200 || Math.abs(distanceToY) > 200 && opModeIsActive()) {
            RoboPosition.currentXPos = odometers.getXPos();
            RoboPosition.currentYPos = odometers.getYPOs();
            RoboPosition.currentHeading = imu.readCurrentHeading();

            distanceToX = x - RoboPosition.currentXPos;
            distanceToY = y - RoboPosition.currentYPos;

            //vector = pid.calcErrors(distanceToX, distanceToY);
            //double wheelPower = Math.hypot(vector[0], vector[1]);
            double wheelPower = .2;

            double angleRadians;
            angleRadians = Math.atan2(distanceToY, distanceToX) - Math.PI/4;

            double adjustment = -imu.headingAdjustment(heading);

            lfPower = wheelPower * Math.cos(angleRadians) + adjustment;
            rfPower = wheelPower * Math.sin(angleRadians) - adjustment;
            lrPower = wheelPower * Math.sin(angleRadians) + adjustment;
            rrPower = wheelPower * Math.cos(angleRadians) - adjustment;

            driveTrain.applyPower(lfPower, rfPower, lrPower, rrPower);
            telemetry.addData("XPos", currentXPos);
            telemetry.addData("YPOs", currentYPos);
            telemetry.addData("distancetoX", distanceToX);
            telemetry.addData("distancetoY", distanceToY);
            telemetry.addData("andleradians", angleRadians);
            telemetry.addData("angleradianDegrees", Math.toDegrees(angleRadians));
            telemetry.update();

        }
        driveTrain.applyPower(0, 0, 0, 0);
    }

}
