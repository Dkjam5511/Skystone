package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp10435 extends OpMode {
    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;
    DcMotor intakeL;
    DcMotor intakeR;
    DcMotor vLift;
    DcMotor vLift2;
    Servo hLift;
    Servo stoneGrabber;
    Servo stoneSpinner;
    Servo hookL;
    Servo hookR;
    Servo capstonePost;
    int stoneLevel = 0;
    int reverse = 1;
    final static int stoneTickHeight = 110;
    final static int firstStoneGap = 50;
    int liftTargetTicks;
    int prevVLiftTicks;
    int vLiftSpeed;
    int prevHLiftTicks;
    int hLiftSpeed;
    int vLiftTicks;
    int hLiftTicks;
    int vTickCorrection;
    int hTickCorrection;
    int extraLiftSauce = 0;
    int capstoneStage = 1;
    double hLiftPos = 0;
    double powerCorrection;
    double liftStickPower;
    boolean lifting = false;
    boolean vManualLift = false;
    boolean hManualLift = false;
    boolean intakeOn = false;
    boolean extending = false;
    boolean retracting = false;
    boolean releasing = false;
    boolean reversing = false;
    boolean vAtIntakePos = true;
    boolean hAtIntakePos = true;
    boolean vLiftFirstRun = true;
    boolean hLiftFirstRun = true;
    boolean deployingCapstone = false;

    ElapsedTime aTimer = new ElapsedTime();
    ElapsedTime xTimer = new ElapsedTime();
    ElapsedTime bTimer = new ElapsedTime();
    ElapsedTime aTimer2 = new ElapsedTime();
    ElapsedTime xTimer2 = new ElapsedTime();
    ElapsedTime bTimer2 = new ElapsedTime();
    ElapsedTime yTimer = new ElapsedTime();
    ElapsedTime downTimer = new ElapsedTime();
    ElapsedTime upTimer = new ElapsedTime();
    ElapsedTime dropTimer = new ElapsedTime();
    ElapsedTime vLiftSpeedTimer = new ElapsedTime();
    ElapsedTime hLiftTimer = new ElapsedTime();
    ElapsedTime liftSlamTimer = new ElapsedTime();
    ElapsedTime liftManualTimer = new ElapsedTime();

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        intakeL = hardwareMap.dcMotor.get("il");
        intakeR = hardwareMap.dcMotor.get("ir");
        vLift = hardwareMap.dcMotor.get("vl");
        vLift2 = hardwareMap.dcMotor.get("vl2");
        hLift = hardwareMap.servo.get("hl");
        stoneGrabber = hardwareMap.servo.get("sg");
        stoneSpinner = hardwareMap.servo.get("ss");
        hookL = hardwareMap.servo.get("hkl");
        hookR = hardwareMap.servo.get("hkr");
        capstonePost = hardwareMap.servo.get("cp");


        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        vLift2.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vLift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_UP);
        stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        hookL.setPosition(GlobalPositions.HOOKL_UP);
        hookR.setPosition(GlobalPositions.HOOKR_UP);
        capstonePost.setPosition(GlobalPositions.CAPSTONE_START);

    }

    @Override
    public void loop() {
        //Driving
        double leftstickx = 0;
        double leftsticky = 0;
        double rightstickx = 0;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;
        double dpadpower = .25;

        if (gamepad1.dpad_up) {
            leftsticky = dpadpower;
        } else if (gamepad1.dpad_right) {
            leftstickx = dpadpower;
        } else if (gamepad1.dpad_down) {
            leftsticky = -dpadpower;
        } else if (gamepad1.dpad_left) {
            leftstickx = -dpadpower;
        } else {
            leftstickx = gamepad1.left_stick_x;
            leftsticky = -gamepad1.left_stick_y;
            rightstickx = gamepad1.right_stick_x;
        }
        wheelpower = Math.hypot(leftstickx, leftsticky);
        stickangleradians = Math.atan2(leftsticky, leftstickx);

        stickangleradians = stickangleradians - Math.PI / 4; //adjust by 45 degrees

        rightX = rightstickx * .5 * reverse;
        leftfrontpower = (wheelpower * Math.cos(stickangleradians) + rightX);
        rightfrontpower = (wheelpower * Math.sin(stickangleradians) - rightX);
        leftrearpower = (wheelpower * Math.sin(stickangleradians) + rightX);
        rightrearpower = (wheelpower * Math.cos(stickangleradians) - rightX);

        lf.setPower(leftfrontpower);
        rf.setPower(rightfrontpower);
        lr.setPower(leftrearpower);
        rr.setPower(rightrearpower);


        //Control Maps
        if (gamepad1.a && aTimer2.seconds() > .2) {
            reversing = !reversing;
            aTimer2.reset();
        }

        if (reversing) {
            reverse = -1;
        } else {
            reverse = 1;
        }

        if (gamepad1.right_bumper) {
            intakeOn = true;
        }
        if (gamepad1.left_bumper) {
            intakeOn = false;
        }

        if (intakeOn) {
            if ((gamepad1.right_trigger == 1)) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
            } else {
                intakeL.setPower(.75);
                intakeR.setPower(.75);
            }
        } else {
            if (gamepad1.right_trigger == 1) {
                intakeL.setPower(-1);
                intakeR.setPower(-1);
            } else {
                intakeL.setPower(0);
                intakeR.setPower(0);
            }
        }

        if (gamepad2.right_trigger >= .75) {
            stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_DOWN);
        }

        if (gamepad2.left_trigger >= .75) {
            stoneGrabber.setPosition(GlobalPositions.STONE_GRABBER_UP);
            releasing = true;
            dropTimer.reset();
        }

        if (gamepad2.dpad_left) {
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_LEFT);
        }
        if (gamepad2.dpad_up) {
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
        }
        if (gamepad2.dpad_right) {
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_RIGHT);
        }
        if (gamepad2.dpad_down) {
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
        }

        if (gamepad2.a && aTimer.seconds() > .2) {
            stoneLevel++;
            lifting = true;
            vManualLift = false;
            hManualLift = false;
            aTimer.reset();
        }

        if (gamepad2.x && xTimer.seconds() > .2) {
            lifting = true;
            vManualLift = false;
            hManualLift = false;

            xTimer.reset();
        }

        if (gamepad2.y && yTimer.seconds() > .2) {
            lifting = false;
            vLiftFirstRun = true;
            hLiftFirstRun = true;
            vManualLift = false;
            hManualLift = false;
            retracting = true;
            extending = false;
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            yTimer.reset();
        }

        if (gamepad2.b && bTimer.seconds() > .2) {
            stoneLevel = 0;
            lifting = false;
            vLiftFirstRun = true;
            hLiftFirstRun = true;
            vManualLift = false;
            hManualLift = false;
            extending = false;
            retracting = true;
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            bTimer.reset();
        }

        if (gamepad2.right_bumper && upTimer.seconds() > .3) {
            stoneLevel++;
            vManualLift = false;
            hManualLift = false;
            upTimer.reset();
        }

        if (gamepad2.left_bumper && downTimer.seconds() > .3) {
            stoneLevel--;
            vManualLift = false;
            hManualLift = false;
            downTimer.reset();
        }

        if (gamepad2.start) {
            extending = true;
            hLiftTimer.reset();
            deployingCapstone = false;
        }

        if (gamepad2.back) {
            extending = true;
            hLiftTimer.reset();
            deployingCapstone = true;
        }

        if (gamepad1.x && xTimer2.seconds() > .2) { //down
            hookL.setPosition(GlobalPositions.HOOKL_DOWN);
            hookR.setPosition(GlobalPositions.HOOKR_DOWN);
            xTimer2.reset();
        }

        if (gamepad1.b && bTimer2.seconds() > .2) { //up
            hookL.setPosition(GlobalPositions.HOOKL_UP);
            hookR.setPosition(GlobalPositions.HOOKR_UP);
            bTimer2.reset();
        }

        if (gamepad1.y) {
            hookL.setPosition(GlobalPositions.HOOKL_READY);
            hookR.setPosition(GlobalPositions.HOOKR_READY);
        }

        if (gamepad1.start) {
            capstonePost.setPosition(GlobalPositions.CAPSTONE_START);
        }

        if (stoneLevel > 11) {
            stoneLevel = 11;
        }

        if (stoneLevel < 0) {
            stoneLevel = 0;
        }

        //Vertical Lift
        vLiftTicks = vLift.getCurrentPosition() - vTickCorrection;

        if (stoneLevel > 0) {
            liftTargetTicks = stoneLevel * stoneTickHeight + firstStoneGap + extraLiftSauce;
        } else {
            liftTargetTicks = 0;
        }
        if (lifting) {
            vAtIntakePos = false;

            if (Math.abs(gamepad2.left_stick_y) > .03) {
                vManualLift = true;
            }

            if (vManualLift) {
                if (releasing && vLiftTicks < liftTargetTicks + 30) {
                    if (dropTimer.seconds() > .25) {
                        extraLiftSauce = 30;
                        vLift.setPower(1);
                        vLift2.setPower(1);
                    }
                } else {
                    releasing = false;
                    if (Math.abs(gamepad2.left_stick_y) > .03){
                        liftStickPower = gamepad2.left_stick_y;

                        if (liftStickPower > 0){
                            liftStickPower = liftStickPower + .2;
                            if (vLiftFirstRun) {
                                prevVLiftTicks = vLiftTicks;
                                vLiftSpeedTimer.reset();
                            } else if (vLiftSpeedTimer.seconds() > .1) {
                                vLiftSpeed = vLiftTicks - prevVLiftTicks;
                                prevVLiftTicks = vLiftTicks;
                                vLiftSpeedTimer.reset();
                            }
                            if (Math.abs(vLiftSpeed) > 1){
                                liftStickPower = liftStickPower*.6;
                            }
                        }

                        vLift.setPower(-liftStickPower);
                        vLift2.setPower(-liftStickPower);
                    } else {
                        vLift.setPower(.01);
                        vLift2.setPower(.01);
                    }
                }
            } else {
                powerCorrection = (Math.pow((Math.abs(liftTargetTicks - vLiftTicks) + 5) / 15, 2) + 3) / 100;

                if (vLiftTicks < liftTargetTicks - 10) {
                    vLift.setPower(powerCorrection);
                    vLift2.setPower(powerCorrection);
                } else if (vLiftTicks > liftTargetTicks + 10) {
                    vLift.setPower(-powerCorrection/2);
                    vLift2.setPower(-powerCorrection/2);
                } else {
                    vLift.setPower(.01);
                    vLift2.setPower(.01);
                }
            }
        } else {
            if (vLiftTicks > 10 && !vAtIntakePos) {
                if (vLiftTicks > 7 * stoneTickHeight + firstStoneGap){
                    vLift.setPower(-1);
                    vLift2.setPower(-1);
                }else {
                    vLift.setPower(-.8);
                    vLift2.setPower(-.8);
                }
                if (vLiftFirstRun) {
                    prevVLiftTicks = vLiftTicks;
                    vLiftSpeedTimer.reset();
                } else if (vLiftSpeedTimer.seconds() > .1) {
                    vLiftSpeed = vLiftTicks - prevVLiftTicks;
                    prevVLiftTicks = vLiftTicks;
                    vLiftSpeedTimer.reset();
                    if (vLiftSpeed == 0) {
                        vAtIntakePos = true;
                    }
                }
                vLiftFirstRun = false;
                liftSlamTimer.reset();

            } else {
                vAtIntakePos = true;
                extraLiftSauce = 0;
                if (liftSlamTimer.seconds() < .5) {
                    vLift.setPower(-1);
                    vLift2.setPower(-1);
                } else {
                    vLift.setPower(0);
                    vLift2.setPower(0);
                    vTickCorrection = vLift.getCurrentPosition();
                }
            }

        }

        //Horizontal Lift
        if (extending) {
            if (hLiftTimer.seconds() < .75) {
                if (deployingCapstone) {
                    hLift.setPosition(1);
                    hLiftPos = 1;
                } else {
                    hLift.setPosition(.8);
                    hLiftPos = .8;
                }
            } else if (hLiftTimer.seconds() < 1.25 && deployingCapstone){
                stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_CAPSTONE);
                hLift.setPosition(.6);
                hLiftPos = .6;
                capstonePost.setPosition(1);
            } else{
                extending = false;
            }
        } else if (retracting) {
            hLift.setPosition(GlobalPositions.MIN_HLIFT_POS);
            hLiftPos = GlobalPositions.MIN_HLIFT_POS;
            retracting = false;
            hAtIntakePos = true;
        } else {
            hLiftPos += -gamepad2.right_stick_y / 100;

            if (hLiftPos > GlobalPositions.MAX_HLIFT_POS){
                hLiftPos = GlobalPositions.MAX_HLIFT_POS;
            }
            if (hLiftPos < GlobalPositions.MIN_HLIFT_POS) {
                hLiftPos = GlobalPositions.MIN_HLIFT_POS;
            }
            hLift.setPosition(hLiftPos);
        }


        /*
        Old horizontal Lift
        hLiftTicks = hLiftEncoder.getCurrentPosition() - hTickCorrection;

        if (extending) {
            hAtIntakePos = false;

            if (hLiftTicks < 7000 && !deployingCapstone) {
                hLift.setPower(GlobalPositions.HLIFT_FORWARD_SPEED);
                if (hLiftTicks > 6500) {
                    stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
                }
            } else if (deployingCapstone) {
                if (capstoneStage == 1){
                    if (hLiftTicks < 8000){ // Extend Out hLift
                        hLift.setPower(GlobalPositions.HLIFT_FORWARD_SPEED);
                    } else {
                        capstoneStage = 2;
                    }
                    if (hLiftTicks > 7500) {
                        stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_CAPSTONE); // Once hlift is extended a certain ammount move servos into place
                        capstonePost.setPosition(1);
                        capstoneTimer.reset();
                    }
                } else if (capstoneStage == 2){
                    if (hLiftTicks > 7000) { // Move hLift to capstone position
                        if(capstoneTimer.seconds() > .5) {
                            hLift.setPower(GlobalPositions.HLIFT_REVERSE_SPEED);
                        } else {
                            hLift.setPower(0);
                        }
                    } else {
                        capstoneStage = 3;
                    }
                } else if (capstoneStage == 3){ //Exit "deployingCapstone"
                    hLift.setPower(0);
                    extending = false;
                }
            } else {
                extending = false;
            }
        } else if (retracting && !hAtIntakePos) {
            hLift.setPower(GlobalPositions.HLIFT_REVERSE_SPEED);
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            if (hLiftFirstRun) {
                prevHLiftTicks = hLiftTicks;
                hLiftSpeedTimer.reset();
            } else if (hLiftSpeedTimer.seconds() > .1) {
                hLiftSpeed = hLiftTicks - prevHLiftTicks;
                prevHLiftTicks = hLiftTicks;
                hLiftSpeedTimer.reset();
                if (hLiftSpeed > -20) {
                    hAtIntakePos = true;
                    hTickCorrection = hLiftEncoder.getCurrentPosition();
                }
            }
            hLiftFirstRun = false;
        } else{
            if (Math.abs(gamepad2.right_stick_y) > .1) {
                hManualLift = true;
            }

            if (hLiftTicks > 200 && hAtIntakePos && !hManualLift) {
                hLift.setPower(GlobalPositions.HLIFT_REVERSE_SPEED / 2);
            } else {
                hLift.setPower(-gamepad2.right_stick_y);
            }
            extending = false;
            retracting = false;
            deployingCapstone = false;
            capstoneStage = 1;
        }
*/
        telemetry.addData("Stone Level: ", stoneLevel);
        telemetry.addData("VLift Encoder: ", vLiftTicks);
        telemetry.addData("VLift Target: ", liftTargetTicks);
        telemetry.addData("VLift Tick Correction", vTickCorrection);
        telemetry.addData("VLift Stick Power", liftStickPower);
        telemetry.addData("Power Correction", powerCorrection);
        telemetry.addData("Manual Lift Mode", vManualLift);
        telemetry.addData("VLift Speed", vLiftSpeed);
        telemetry.update();
    }
}