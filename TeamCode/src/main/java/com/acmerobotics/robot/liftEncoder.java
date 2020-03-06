package com.acmerobotics.robot;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

@Config
public class liftEncoder {
    public DcMotorEx liftMotor1, liftMotor2;
    private DigitalChannel bottomHallEffect;



    ////////////// P control ///////////////

    public int error;

    public int setPoint;

    public static double Pcoefficient = 0.0009;

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

    private int maxHeight = 4600;

    public double liftPower = 0.5;

    private int stateb = 0;

    public enum Mode{
        BLOCKS,
        BOTTOM,
        DIRECT
    }

    public Mode mode;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(5, 0.055, 0, 0, MotorControlAlgorithm.LegacyPID);


    public liftEncoder(HardwareMap hardwareMap){
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        bottomHallEffect = hardwareMap.digitalChannel.get("bottomHallEffect");

        liftMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        liftMotor2.setDirection(DcMotorEx.Direction.FORWARD);
    }


    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(){

        liftMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        liftMotor1.setTargetPosition(0);
        liftMotor2.setTargetPosition(0);

        liftMotor1.setPower(0);
        liftMotor2.setPower(0);

        liftMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void runTo(int position, double power) {

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

        runTo(targetPosition, 0.5);
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
                        runTo(-100, 0.5);

                        if (liftMotor1.getCurrentPosition() <= -90){
                            stateb++;
                        }
                        break;

                    case 3:
                        liftMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        liftMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        bottomPosition = 0;
                        bottomSet = true;
                        stateb++;
            }
        }
    }


    ////////// encoder math, inches to encoder ticks ///////////////

    public double blocksToTotalHeight(int blocks){
        double height = (blocks * blockHeight) + foundationHeight + extraHeight;
        return (height);
    }


    public int inchesToTicks(int blocks){

        double targetHeight = blocksToTotalHeight(blocks);

        int ticks = (int) ((targetHeight * TICKS_PER_REV) / (Math.PI * radius * 2));

        return ticks;
    }


    /////////////////////// other methods //////////////////////////

    private void setMode(Mode mode){
        this.mode = mode;
    }

    public boolean isAtBottom(){
        boolean state = bottomHallEffect.getState();
        boolean inverseState = false;

        if (state == true){
            inverseState = false;
        }
        if (state == false){
            inverseState = true;
        }

        return inverseState; // is at bottom
    }

    /////////////////////////////////////////////////


    public void setPID(){//PIDFCoefficients coefficients){
        liftMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);
        liftMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, coefficients);
    }


    ////////////////////// P control /////////////////////////////

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
            target = 4599;
        }

        setPoint = target;
    }

    /////////////////////////////////////////////////
}
