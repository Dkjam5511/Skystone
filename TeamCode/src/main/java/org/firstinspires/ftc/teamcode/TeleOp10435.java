package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    DcMotor hLift;
    Servo stoneGrabber;
    Servo stoneSpinner;
    Servo hookL;
    Servo hookR;
    Servo capstonePost;
    int stoneLevel = 0;
    int reverse = 1;
    final static int stoneTickHeight = 500;
    final static int firstStoneGap = 300;
    int liftTargetTicks;
    int prevVLiftTicks;
    int vLiftSpeed;
    int prevHLiftTicks;
    int hLiftSpeed;
    int vLiftTicks;
    int hLiftTicks;
    int vTickCorrection;
    int hTickCorrection;
    int capstoneStage = 1;
    boolean lifting = false;
    boolean manualLift = false;
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
    ElapsedTime hLiftSpeedTimer = new ElapsedTime();
    ElapsedTime capstoneTimer = new ElapsedTime();

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lr = hardwareMap.dcMotor.get("lr");
        rr = hardwareMap.dcMotor.get("rr");
        intakeL = hardwareMap.dcMotor.get("il");
        intakeR = hardwareMap.dcMotor.get("ir");
        vLift = hardwareMap.dcMotor.get("vl");
        hLift = hardwareMap.dcMotor.get("hl");
        stoneGrabber = hardwareMap.servo.get("sg");
        stoneSpinner = hardwareMap.servo.get("ss");
        hookL = hardwareMap.servo.get("hkl");
        hookR = hardwareMap.servo.get("hkr");
        capstonePost = hardwareMap.servo.get("cp");


        rf.setDirection(DcMotor.Direction.REVERSE);
        rr.setDirection(DcMotor.Direction.REVERSE);
        vLift.setDirection(DcMotorSimple.Direction.REVERSE);
        hLift.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        vLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stoneGrabber.setPosition(0);
        stoneSpinner.setPosition(.68);
        hookL.setPosition(GlobalPositions.HOOKL_UP);
        hookR.setPosition(GlobalPositions.HOOKR_UP);
        capstonePost.setPosition(1);

    }

    @Override
    public void loop() {
        //Driving
        double leftstickx;
        double leftsticky;
        double rightstickx;
        double wheelpower;
        double stickangleradians;
        double rightX;
        double leftfrontpower;
        double rightfrontpower;
        double leftrearpower;
        double rightrearpower;

        leftstickx = gamepad1.left_stick_x;
        leftsticky = -gamepad1.left_stick_y;
        rightstickx = gamepad1.right_stick_x;

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
            stoneGrabber.setPosition(.56);
        }

        if (gamepad2.left_trigger >= .75) {
            stoneGrabber.setPosition(0);
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
            manualLift = false;
            aTimer.reset();
        }

        if (gamepad2.x && xTimer.seconds() > .2) {
            lifting = true;
            manualLift = false;
            xTimer.reset();
        }

        if (gamepad2.y && yTimer.seconds() > .2) {
            lifting = false;
            vLiftFirstRun = true;
            hLiftFirstRun = true;
            manualLift = false;
            retracting = true;
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            yTimer.reset();
        }

        if (gamepad2.b && bTimer.seconds() > .2) {
            stoneLevel = 0;
            lifting = false;
            vLiftFirstRun = true;
            hLiftFirstRun = true;
            manualLift = false;
            retracting = true;
            stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_DOWN);
            bTimer.reset();
        }

        if (gamepad2.right_bumper && upTimer.seconds() > .3) {
            stoneLevel++;
            manualLift = false;
            upTimer.reset();
        }

        if (gamepad2.left_bumper && downTimer.seconds() > .3) {
            stoneLevel--;
            manualLift = false;
            downTimer.reset();
        }

        if (gamepad2.start) {
            extending = true;
            deployingCapstone = false;
        }

        if (gamepad2.back) {
            extending = true;
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

        if (gamepad1.y){
            hookL.setPosition(GlobalPositions.HOOKL_READY);
            hookR.setPosition(GlobalPositions.HOOKR_READY);
        }

        if (stoneLevel > 9) {
            stoneLevel = 9;
        }

        if (stoneLevel < 0) {
            stoneLevel = 0;
        }

        //Vertical Lift
        vLiftTicks = vLift.getCurrentPosition() - vTickCorrection;

        if (stoneLevel > 0) {
            liftTargetTicks = stoneLevel * stoneTickHeight + firstStoneGap;
        } else {
            liftTargetTicks = 0;
        }
        if (lifting) {
            vAtIntakePos = false;

            if (Math.abs(gamepad2.left_stick_y) > .1) {
                manualLift = true;
            }

            if (manualLift) {
                if (releasing && vLiftTicks < stoneLevel * stoneTickHeight + 300) {
                    if (dropTimer.seconds() > .25) {
                        vLift.setPower(1);
                    }
                } else {
                    releasing = false;
                    vLift.setPower(-gamepad2.left_stick_y);
                }
            } else {
                if (vLiftTicks < liftTargetTicks - 50) {
                    vLift.setPower(1);
                } else if (vLiftTicks > liftTargetTicks + 50) {
                    vLift.setPower(-1);
                } else {
                    vLift.setPower(0);
                }
            }
        } else {
            if (vLiftTicks > 60 && !vAtIntakePos) {
                vLift.setPower(-1);
                if (vLiftFirstRun) {
                    prevVLiftTicks = vLiftTicks;
                    vLiftSpeedTimer.reset();
                } else if (vLiftSpeedTimer.seconds() > .1) {
                    vLiftSpeed = vLiftTicks - prevVLiftTicks;
                    prevVLiftTicks = vLiftTicks;
                    vLiftSpeedTimer.reset();
                    if (vLiftSpeed > -20) {
                        vAtIntakePos = true;
                        vTickCorrection = vLift.getCurrentPosition();
                    }
                }
                vLiftFirstRun = false;
            } else {
                vAtIntakePos = true;
                vLift.setPower(0);
            }
        }

        //Horizontal Lift
        hLiftTicks = hLift.getCurrentPosition();
        
        if (extending) {

            hAtIntakePos = false;

            if (hLiftTicks < 2000 && !deployingCapstone) {
                hLift.setPower(1);
                if (hLiftTicks > 1700) {
                    stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_UP);
                }
            } else if (deployingCapstone) {
                if (capstoneStage == 1){
                    if (hLiftTicks < 2000){ // Extend Out hLift
                        hLift.setPower(1);
                    } else {
                        capstoneStage = 2;
                    }
                    if (hLiftTicks > 1900) {
                        stoneSpinner.setPosition(GlobalPositions.STONE_SPINNER_LEFT); // Once hlift is extended a certain ammount move servos into place
                        capstonePost.setPosition(.5);
                        capstoneTimer.reset();
                    }
                } else if (capstoneStage == 2){
                    if (hLiftTicks > 1350) { // Move hLift to capstone position
                        if(capstoneTimer.seconds() > .5) {
                            hLift.setPower(-1);
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
            hLift.setPower(-1);
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
                    hTickCorrection = hLift.getCurrentPosition();
                }
            }
            hLiftFirstRun = false;
        } else {
            extending = false;
            retracting = false;
            deployingCapstone = false;
            capstoneStage = 1;
            hLift.setPower(-gamepad2.right_stick_y);
        }

        telemetry.addData("Stone Level: ", stoneLevel);
        telemetry.addData("VLift Encoder: ", vLiftTicks);
        telemetry.addData("HLift Encoder: ", hLiftTicks);
        telemetry.addData("hLiftSpeed: ", hLiftSpeed);
        telemetry.addData(" Vertical AtIntakePos", vAtIntakePos);
        telemetry.addData(" Horizontal AtIntakePos", hAtIntakePos);
        telemetry.addData("hTickCorrection", hTickCorrection);
        telemetry.update();
    }
}