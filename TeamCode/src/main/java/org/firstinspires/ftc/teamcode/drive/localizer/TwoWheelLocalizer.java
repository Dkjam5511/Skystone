package org.firstinspires.ftc.teamcode.drive.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class TwoWheelLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2400;
    public static double WHEEL_RADIUS = (2.3622/2); // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel

    private ExpansionHubMotor xOdom, yOdom;
    private ExpansionHubEx hub, hub2;
    private BNO055IMU imu;

    public TwoWheelLocalizer(HardwareMap hardwareMap, BNO055IMU imu) {
        super(Arrays.asList(
                new Pose2d(-5.5, -.75, 0), // yOdom
                new Pose2d(4.75, 1, Math.toRadians(90)) // xOdom
        ));

        this.imu = imu;
        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 4");
        xOdom = hardwareMap.get(ExpansionHubMotor.class,"ir");
        yOdom = hardwareMap.get(ExpansionHubMotor.class,"vl2");

        xOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        yOdom.setDirection(DcMotor.Direction.REVERSE);
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = hub.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        return Arrays.asList(
                encoderTicksToInches(bulkData2.getMotorCurrentPosition(yOdom)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(xOdom))
        );
    }

    @NonNull
    public List<Double> getWheelVelocities() {
        RevBulkData bulkData = hub.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        return Arrays.asList(
                encoderTicksToInches(bulkData2.getMotorVelocity(yOdom)),
                encoderTicksToInches(bulkData.getMotorVelocity(xOdom))
        );

    }

    @Override
    public double getHeading() {
        return -imu.getAngularOrientation().firstAngle;
    }
}
