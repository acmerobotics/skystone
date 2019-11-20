package com.acmerobotics.robot;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.opmodes.TeleOp;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Lift {

    private DcMotorEx liftMotor;
    private MotionProfile profile;
    private DigitalChannel bottomHallEffect;
    private double startTime;

    private double targetPosition;


    public static double K_V = 1;
    public static double K_A = 1;
    public static double K_STATIC = 1;
    public static double G = 1;
    public static double P = 1;
    public static double I = 1;
    public static double D = 1;
    public static double V = 1;
    public static double A = 1;
    public static double J = 1;
    public static double RADIUS = 0.5;

    public static double MAX_LIFT_HIEGHT = 14;

    public static double LIFT_HEIGHT = 15;
    public static double LIFT_INTAKE = 0;
    public static double LIFT_RELOCATION = 0;
    public static double LIFT_BOTTOM = 0;
    public static double INCREMENT = 5;
    public static double BASE_HEIGHT = 2.5;

    public static double MASS_ARM = 0;
    public static double MASS_FRAME = 0;


    public static double CALIBRATE_V = 0;

    public double offset;
    public double placingHeight = 0;

    private boolean calibrated = false;

    private PIDController pidController;

    private enum LiftMode{
        DRIVER_CONTROLLED,
        FIND_BOTTOM,
        HOLD_POSITION,
        RUN_TO_POSITION
    }

    private LiftMode liftMode = LiftMode.DRIVER_CONTROLLED;

    public Lift(HardwareMap hardwareMap){

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        bottomHallEffect = hardwareMap.digitalChannel.get("bottomHallEffect");

        liftMotor.setPower(0);

        pidController = new PIDController(P, I, D);

        liftMode = LiftMode.HOLD_POSITION;


    }

    public double getPosition(){
        return internalGetPosition() + offset;
    }

    public double internalGetPosition(){
        return ((liftMotor.getCurrentPosition() / (liftMotor.getMotorType().getTicksPerRev())) * Math.PI * RADIUS * 2);

    }

    public void setPower(double power){
        internalSetVelocity(power);
    }

    public void setPosition(double position){
        offset = position - internalGetPosition();
    }

    public void update(TelemetryPacket packet){
        packet.put("lift mode", liftMode.toString());
        packet.put("position", getPosition());

        switch (liftMode){

            case HOLD_POSITION:
                double error = getPosition() - targetPosition;
                double correction = pidController.update(error);
                packet.put("error", error);
                internalSetVelocity(-correction);
                packet.put("lift correction", -correction);

                break;


            case RUN_TO_POSITION:
                pidController = new PIDController(P, I, D);
               /* double t = (System.currentTimeMillis() - startTime) / 1000.0;
                if (profile == null){
                    return;
                }
                MotionState target = profile.get(t);

                error = getPosition() - target.getX();//the set point (target.getX) is set to the position that is
                                                        // reached at the end of the motion profiles journey.
                packet.put("error", error);
                correction = pidController.update(error);
                double feedForward = getFeedForward(target, getMass());
                internalSetVelocity(feedForward - correction);

                if (t > profile.duration()){
                    packet.put("complete", true);
                    liftMode = LiftMode.HOLD_POSITION;
                    targetPosition = profile.end().getX();
                    return;
                }*/

                break;


            case FIND_BOTTOM:
                internalSetVelocity(CALIBRATE_V);
                if(isAtBottom()){
                    liftMode = LiftMode.HOLD_POSITION;
                    calibrated = true;
                    setPower(0);
                    targetPosition = 0;
                    setPosition(0);
                    pidController = new PIDController(P, I, D);
                }
                break;


        }

    }

    public void goToPosition(double position){
        internalSetVelocity(1);
        /*pidController = new PIDController(P, I, D);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0, 0, 0),
                V, A, J
        );
        startTime = System.currentTimeMillis(); */
        liftMode = LiftMode.RUN_TO_POSITION;
    }

    public void internalSetVelocity(double v){
        liftMotor.setPower(v);
    }

    public double getOffset(){
        return offset;
    }

    public void goToBottom() {
        goToPosition(LIFT_BOTTOM);
        liftMode = LiftMode.FIND_BOTTOM;


    }

//    public void relocationPosition(){
//        goToPosition(LIFT_RELOCATION);
//        liftMode = LiftMode.RUN_TO_POSITION;
//
//    }
//
//    public void intakePosition(){
//
//        goToPosition(LIFT_INTAKE);
//        liftMode = LiftMode.RUN_TO_POSITION;
//    }

    public void moveTo(int blocks){
        if(getPosition() < MAX_LIFT_HIEGHT){
            double position = setLiftIncrement((blocks));
            goToPosition(position);
            liftMode = LiftMode.RUN_TO_POSITION;
        } else {
            liftMode = LiftMode.HOLD_POSITION;
        }
    }

    public boolean isAtBottom(){
        return bottomHallEffect.getState();
    }

    public double setLiftIncrement(int blocks){
        placingHeight = BASE_HEIGHT + (INCREMENT * blocks);
        return placingHeight;
    }

    private double getFeedForward(MotionState state, double mass){
        double ff = K_V * state.getV() + K_A * mass * (state.getA() - G);
        if(Math.abs(ff) > 1e-4){
            ff += Math.copySign(K_STATIC, ff);
        }

        return ff;
    }

    public double getMass(){
        double mass = MASS_ARM;
        if(!isAtBottom()){
            mass += MASS_FRAME;
        }

        return mass;
    }

}
