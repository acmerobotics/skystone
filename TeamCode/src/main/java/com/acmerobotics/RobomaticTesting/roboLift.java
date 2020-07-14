package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.acmerobotics.robomatic.util.PIDController;


@Config

public class roboLift extends Subsystem {
    // init motors
    public DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;

    // init digital device
    private DigitalChannel bottomHallEffect;

    ////////////// P control ///////////////

    public int error;

    public int setPoint;

    public static double P = 2;
    public static double I = 0;
    public static double D = 0;

    public PIDController pidController;

    //////////////////////////////////

    public double blockHeight = 5;
    public double foundationHeight = 2;
    public double extraHeight = 0.5; // will get height greater than target so it doesn't run into it
    public static int startHeight = 1475; // 1528
    public static int bottomPosition = 0;

    public  boolean stringTightened = false;
    public boolean bottomSet = false;

    //////////////////////
    public int blockPosition = 0;

    public static int blockEncoderHeight = 1070; //1560

    private int radius = 1;
    private int TICKS_PER_REV = 280;

    private int maxHeight = 5200;

    public double liftPower = 0.5;

    public int stateb = 0;

    private enum mode{
        RUN_TO,
        BOTTOM,

    }

    private mode liftMode = mode.RUN_TO;


    public roboLift(Robot robot){
        super("Lift");

        liftMotor1 = robot.getMotor("liftMotor1");
        liftMotor2 = robot.getMotor("liftMotor2");

        bottomHallEffect = robot.getDigitalChannel("bottomHallEffect");

        pidController = new PIDController(P, I, D);
    }


    @Override
    public void update(Canvas overlay){
        telemetryData.addData("m1 current pos", liftMotor1.getCurrentPosition());
        telemetryData.addData("m2 current pos", liftMotor2.getCurrentPosition());

        telemetryData.addData("target position", setPoint);

        telemetryData.addData("atBottom", isAtBottom());

        telemetryData.addData("bottom set", bottomSet);

        switch (liftMode){
            case RUN_TO:
                if (setPoint > maxHeight){
                    setPoint = maxHeight;
                }

                error = setPoint - liftMotor1.getCurrentPosition();

                pidController.setOutputBounds(-1, 1);

                double correction = pidController.update(error);

                liftMotor1.setPower(correction);
                liftMotor2.setPower(correction);

                break;

            case BOTTOM:

                boolean isAtBottom = isAtBottom();
                if(bottomSet == false && stringTightened == true) {

                    switch (stateb){

                        case 0:
                            if (!isAtBottom) {

                                liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                                liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                                liftMotor1.setPower(-0.1);
                                liftMotor2.setPower(-0.1);
                            }
                            else{
                                stateb++;
                            }
                            break;

                        case 1:
                            liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                            liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                            stateb++;

                        case 2:
                            bottomPosition = -100;
                            bottomSet = true;
                            stateb++;

                    }
                }

        }
    }


    public void init(){

        liftMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void resetEncoder(){
        // motor's
        // current encoder position is set as the zero position

        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runTo(int position){
        setPoint = position;

        liftMode = mode.RUN_TO;
    }


    public void runToBlocks(int position){

        blockPosition = (blockEncoderHeight * position);// + startHeight;

        setPoint = blockPosition;

        liftMode = mode.RUN_TO;
    }


    public void runToIncrement(int position){
        setPoint = liftMotor1.getCurrentPosition() + position; //make 50 to 150

        liftMode = mode.RUN_TO;
    }


    public void goToStartHeight(){
        setPoint = startHeight;

        liftMode = mode.RUN_TO;
    }


    public void tightenLiftString(){
        int tightPosition = 50;
        if(stringTightened == false) {
            setPoint = tightPosition;

            liftMode = mode.RUN_TO;

            if (liftMotor1.getCurrentPosition() >= (tightPosition - 10)) {
                stringTightened = true;
            }
        }
    }


    public void goToBottom(){
        liftMode = mode.BOTTOM;
    }


    public boolean isAtBottom() {
        // state needs to be invereted because the raw value is the opposite of what is expected
        boolean state = bottomHallEffect.getState();
        boolean inverseState = !state;

        return inverseState; // is at bottom
    }

}
