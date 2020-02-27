package org.firstinspires.ftc.teamcode.RoadRunner.mecanum;

import static org.firstinspires.ftc.teamcode.RoadRunner.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.RoadRunner.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.RoadRunner.DriveConstants.getMotorVelocityF;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.RoadRunner.localizer.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.openftc.revextensions2.ExpansionHubMotor;

/*
 * Optimized mecanum drive implementation for REV E xHs. The time savings may significantly improve
 * trajectory following performance with moderate additional complexity.
 */
public class SampleMecanumDriveREVOptimized extends SampleMecanumDriveBase {
    private ExpansionHubMotor lf, lr, rr, rf;
    private List<ExpansionHubMotor> motors;
    private BNO055IMU imu;

    private ThreeWheelLocalizer localizer;

    public SampleMecanumDriveREVOptimized(HardwareMap hardwareMap) {
        super();

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        lf = hardwareMap.get(ExpansionHubMotor.class, "lf");
        lr = hardwareMap.get(ExpansionHubMotor.class, "lr");
        rr = hardwareMap.get(ExpansionHubMotor.class, "rr");
        rf = hardwareMap.get(ExpansionHubMotor.class, "rf");

        motors = Arrays.asList(lf, lr, rr, rf);

        for (ExpansionHubMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);

        localizer = new ThreeWheelLocalizer(hardwareMap);
        setLocalizer(localizer);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = lf.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (ExpansionHubMotor motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions;

        wheelPositions = localizer.getWheelPositions();

        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities;

        wheelVelocities = localizer.getWheelVelocities();

        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        lf.setPower(v);
        lr.setPower(v1);
        rr.setPower(v2);
        rf.setPower(v3);
    }

    @Override
    protected double getRawExternalHeading() {
        return 0;
    }
}
