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
    private DcMotorEx armMotor;
    private MotionProfile profile;
    DigitalChannel bottomHallEffect;
    private double startTime;


    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;
    public static double G = 0;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double V = 0;
    public static double A = 0;
    public static double J = 0;
    public static double RADIUS = 0;

    public static double LIFT_HEIGHT = 0;
    public static double LIFT_INTAKE = 0;
    public static double LIFT_RELOCATION = 0;
    public static double LIFT_START = 0;

    public static double ARM_START = 0;
    public static double ARM_INTAKE = 0;
    public static double ARM_RELOCATION = 0;

    public static double CALIBRATE_V = 0;

    public double offset;

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
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        bottomHallEffect = hardwareMap.digitalChannel.get("bottomHallEffect");

        pidController = new PIDController(P, I, D);

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

            case RUN_TO_POSITION:


            case FIND_BOTTOM:
                internalSetVelocity(CALIBRATE_V);
                if(isAtBottom()){
                    liftMode = LiftMode.HOLD_POSITION;
                    calibrated = true;
                    internalSetVelocity(0);
                    setPosition(0);
                    pidController = new PIDController(P, I, D);
                }
                break;




        }

    }

    public void goToPosition(double position){
        pidController = new PIDController(P, I, D);
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(0, 0, 0, 0),
                new MotionState(0, 0, 0, 0),
                V, A, J
        );
        startTime = System.currentTimeMillis();

    }

    public void internalSetVelocity(double v){
        liftMotor.setPower(v);
    }

    public double getOffset(){
        return offset;
    }

    public void startPosition(){


    }


    public void relocationPosition(){
        goToPosition(LIFT_RELOCATION);

    }

    public void intakePosition(){

    }

    public boolean isAtBottom(){
        return bottomHallEffect.getState();
    }

}
