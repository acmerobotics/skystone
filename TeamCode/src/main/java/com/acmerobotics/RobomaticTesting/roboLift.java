package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

    public static double Pcoefficient = 0.001;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(5, 0.055, 0, 0, MotorControlAlgorithm.LegacyPID);

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

    public roboLift(Robot robot){
        super("Lift");

        liftMotor1 = robot.getMotor("liftMotor1");
        liftMotor2 = robot.getMotor("liftMotor2");

        bottomHallEffect = robot.getDigitalChannel("bottomHallEffect");
    }


    @Override
    public void update(Canvas overlay){
        telemetryData.addData("m1 current pos", liftMotor1.getCurrentPosition());
        telemetryData.addData("m2 current pos", liftMotor2.getCurrentPosition());

        telemetryData.addData("target position", setPoint);

        telemetryData.addData("atBottom", isAtBottom());
    }


    public void init(){

        liftMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void resetEncoder(){
        // motor's
        // current encoder position is set as the zero position

        //todo add similar method to Robomatic

        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void runTo(int position, double power) {

        //todo add similar method to Robomatic

        setPID(coefficients);

        //@Override
        if (position >= maxHeight){
            position = 4599;
        }

        liftMotor1.setTargetPosition(position);
        liftMotor2.setTargetPosition(position);

        liftMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        liftMotor1.setPower(power);
        liftMotor2.setPower(power);
    }


    public void runToBlocks(int position, double power){

        blockPosition = (blockEncoderHeight * position);// + startHeight;

        if (blockPosition >= maxHeight){
            blockPosition = 4599;
        }

        setPosition(blockPosition);
    }


    public void runToIncrement(int position){
        int targetPosition = liftMotor1.getCurrentPosition() + position; //make 50 to 150

        setPosition(targetPosition);
    }


    public void goToStartHeight(){
        setPosition(startHeight);
    }

    public void tightenLiftString(){
        int tightPosition = 50;
        if(stringTightened == false) {
            runTo(tightPosition, 0.5);

            if (liftMotor1.getCurrentPosition() >= (tightPosition - 10)) {
                stringTightened = true;
            }
        }
    }


    public void goToBottom(){
        boolean isAtBottom = isAtBottom();
        if(bottomSet == false && stringTightened == true) {

            switch (stateb){

                case 0:
                    if (!isAtBottom) {

                        liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                        liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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


    public boolean isAtBottom() {
        boolean state = bottomHallEffect.getState();
        boolean inverseState = false;

        if (state == true) {
            inverseState = false;
        }
        if (state == false) {
            inverseState = true;
        }

        return inverseState; // is at bottom
    }


    ////////////////////// P control /////////////////////////////

    public void setPID(PIDFCoefficients pidfCoefficients) {
        //will set the pid coefficients

        //todo add similar method to Robomatic

        // should only set the pid coefficients if they are different from the current ones
        if (pidfCoefficients != coefficients) {
            liftMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
            liftMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
            coefficients = pidfCoefficients;
        }
    }


    private void updateError(){
        error = setPoint - liftMotor1.getCurrentPosition();
    }


    private void setUp(){
        liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void PController(){
        updateError();

        setUp();

        double P = Pcoefficient * error;

        if (P > 1){
            P = 1;
        }

        if(P < -1){
            P = -1;
        }

        liftMotor1.setPower(P);
        liftMotor2.setPower(P);
    }


    public void setPosition(int target){

        if (target >= maxHeight){
            target = 5199;
        }

        setPoint = target;
    }
}
