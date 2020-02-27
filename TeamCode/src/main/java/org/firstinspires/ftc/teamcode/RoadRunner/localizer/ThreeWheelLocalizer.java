package org.firstinspires.ftc.teamcode.RoadRunner.localizer;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

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
public class ThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2400;
    public static double WHEEL_RADIUS = (2.3622/2); // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 14.85; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -6; // in; offset of the lateral wheel

    private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    private ExpansionHubEx hub, hub2;

    public ThreeWheelLocalizer(HardwareMap hardwareMap) {
        super(
        Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        hub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        hub2 = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 1");


        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "il");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "ir");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class,"vl2");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                encoderTicksToInches(bulkData2.getMotorCurrentPosition(leftEncoder)),
                encoderTicksToInches(-bulkData.getMotorCurrentPosition(rightEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(frontEncoder))
        );
    }

    public List<Double> getWheelVelocities(){
        RevBulkData bulkData = hub.getBulkInputData();
        RevBulkData bulkData2 = hub2.getBulkInputData();

        return Arrays.asList(
                encoderTicksToInches(bulkData2.getMotorVelocity(leftEncoder)),
                encoderTicksToInches(-bulkData.getMotorVelocity(rightEncoder)),
                encoderTicksToInches(bulkData.getMotorVelocity(frontEncoder))
        );
    }
}