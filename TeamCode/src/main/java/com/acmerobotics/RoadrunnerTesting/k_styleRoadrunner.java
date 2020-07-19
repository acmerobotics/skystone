package com.acmerobotics.RoadrunnerTesting;

import com.acmerobotics.RobomaticTesting.roboLift;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

// Roadrunner can be used slightly differently but each variation should end with the same result.
// r_style uses the roadrunner PIDF controller, is very declarative, and simple because everything
// is hidden inside the roadrunner methods. In k_style the robomatic PID controller is used this results
// is less being hidden from the user so the user has to write more code and show more steps to obtain
// a r_style equivalant. For example in k_style the robomatic PID controller doesn't have an opetion
// for feedforward like the roadrunner PID controller so the calculation has to be done by the user
// (the calculation should be the same as the one in the roadrunner controller). Also the PID steps
// have to be done manually in k_style while they are automaticly performed in r_syte.

// Note: all of the lift measurements are based on ticks on the motor not inches
// this means the velocity and acceleration have to also be measured in ticks.
// We need to change future iteration to inches because it is more logical and
// easier to interpret

public class k_styleRoadrunner extends Subsystem {
    // init motors
    public DcMotorEx liftMotor1;
    private DcMotorEx liftMotor2;

    // init digital device
    private DigitalChannel bottomHallEffect;

    ////////////// P control ///////////////

    public double error;

    public double setPoint;

    public static double P = 2;
    public static double I = 0;
    public static double D = 0;

    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;
    public static double K_g = 0;
    public static double G = K_g * 368;

    public static PIDCoefficients COEFFICIENTS = new PIDCoefficients(P, I, D);

    public PIDController controller;

    public static double MAX_V = 0;
    public static double MAX_A = 0;
    public static double MAX_J = 0;

    private MotionProfile profile;

    ElapsedTime time;

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

    private double currentLiftPower = 0;

    public int stateb = 0;

    private enum mode{
        RUN_TO,
        HOLD_POSITION,
        BOTTOM,

    }

    private mode liftMode = mode.RUN_TO;


    public k_styleRoadrunner(Robot robot){
        super("KLift");

        liftMotor1 = robot.getMotor("liftMotor1");
        liftMotor2 = robot.getMotor("liftMotor2");

        bottomHallEffect = robot.getDigitalChannel("bottomHallEffect");

        controller = new PIDController(P, I, D);
    }


    @Override
    public void update(Canvas overlay){
        telemetryData.addData("m1 current pos", liftMotor1.getCurrentPosition());
        telemetryData.addData("m2 current pos", liftMotor2.getCurrentPosition());

        telemetryData.addData("target position", setPoint);

        telemetryData.addData("error", error);

        telemetryData.addData("atBottom", isAtBottom());

        telemetryData.addData("bottom set", bottomSet);

        switch (liftMode){
            case RUN_TO:
                if (setPoint > maxHeight){
                    setPoint = maxHeight;
                }

                double t = time.milliseconds() / 1000; // get time since last goToPos (last time reset) used

                // if the time since goToPos was used is greater than the duration of the motion profile
                // then the profile has reached its position and ther is no longer a need to continue using
                // motion profile instead the position can be held by PID alone in HOLD_POSITION
                if (t > profile.duration()) {
                    liftMode = mode.HOLD_POSITION;
                    setPoint = profile.end().getX(); // position at the end of motion profile
                    currentLiftPower = liftMotor1.getPower();
                    return;
                }

                MotionState target = profile.get(t); // motion state at current time

                error = target.getX() - getPosition(); // motion state position - current position

                double correction = controller.update(error);

                double feedforward = getFeedforward(target);

                liftMotor1.setPower(correction + feedforward);
                liftMotor2.setPower(correction + feedforward);

                break;


            case HOLD_POSITION:
                // hold a position without motion profile just PID
                // motion profiling isn't needed to hold a position. All that is required is the power
                // it took to barely reach the position plus work from a PID controller to maintain and
                // hold that position.

                error = setPoint - getPosition();

                correction = controller.update(error);

                liftMotor1.setPower(correction + currentLiftPower); // current liftPower should work
                        // as a feedforward to keep the arm stable and make the PID just correct small errors.
                liftMotor2.setPower(correction + currentLiftPower);

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


    private double getPosition(){
        return liftMotor1.getCurrentPosition();
    }

    public void goToPosition(double targetPosition){
        MotionState start;

        if (profile != null){ // if a profile exists make its start state the current motion state
            start = profile.get(time.seconds());
        }
        else{ // if a profile doesn't already exist crete a start motion state
            start = new MotionState(getPosition(), 0, 0, 0); // made
        }

        // create a motion profile with max v, a, j and a start and end motion state  at the current position
        profile = MotionProfileGenerator.generateSimpleMotionProfile( // create motion profile
                start, // start motion state
                new MotionState(targetPosition, 0, 0, 0), // create end motion state
                MAX_V,
                MAX_A,
                MAX_J
        );
        time.reset(); // start counting the time since this reset or since goToPosition was called
    }


    private double getFeedforward(MotionState state){
        // feed forward calculation

        // this feed forward calculation is nearly identical to the one used in the readrunner PIDF controller
        double feedforward = K_V * state.getV() + K_A * state.getA() + K_g; // mass could also be included with the acc
                                                                            // to account for inertia but I left it out for now

        // K_static is added to the feed forward when the target is far
        if (Math.abs(feedforward) > 1e-6){ // the 1e-6 is 0.000001 (1e-4 is also used), road runner labels the number as EPSILON
                                // I have a loose definition of Epsilon its basically how close you want to get to something.
                                // My guess is that the closer you approach your target you will the less you will need to
                                // worry about K_static represents static friction (static friction is the force needed to
                                // move an object, it is the friction that works while there is no movement.) so once we
                                // reach the target we won't need to worry about static friction because we won't be moving
                                // any more.
            feedforward += Math.copySign(K_STATIC, feedforward); // K_static needs to have the same sign as the feedforward
        }

        return feedforward;
    }


    public void runTo(int position){
        setPoint = position;

        goToPosition(setPoint);

        liftMode = k_styleRoadrunner.mode.RUN_TO;
    }


    public void runToBlocks(int position){

        blockPosition = (blockEncoderHeight * position);// + startHeight;

        setPoint = blockPosition;

        goToPosition(setPoint);

        liftMode = k_styleRoadrunner.mode.RUN_TO;
    }


    public void runToIncrement(int position){
        setPoint = liftMotor1.getCurrentPosition() + position; //make 50 to 150

        goToPosition(setPoint);

        liftMode = k_styleRoadrunner.mode.RUN_TO;
    }


    public void goToStartHeight(){
        setPoint = startHeight;

        goToPosition(setPoint);

        liftMode = k_styleRoadrunner.mode.RUN_TO;
    }


    public void tightenLiftString(){
        int tightPosition = 50;
        if(stringTightened == false) {
            setPoint = tightPosition;

            liftMode = k_styleRoadrunner.mode.RUN_TO;

            if (liftMotor1.getCurrentPosition() >= (tightPosition - 10)) {
                stringTightened = true;
            }
        }
    }


    public void goToBottom(){
        liftMode = k_styleRoadrunner.mode.BOTTOM;
    }


    public boolean isAtBottom() {
        // state needs to be invereted because the raw value is the opposite of what is expected
        boolean state = bottomHallEffect.getState();
        boolean inverseState = !state;

        return inverseState; // is at bottom
    }

}
