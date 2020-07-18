package com.acmerobotics.util;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RoadrunnerSample {

    // variable constants
    public static double K_V = 0;
    public static double K_A = 0;
    public static double K_STATIC = 0;
    public static double K_g = 0;
    public static double G = K_STATIC * 368;

    public static PIDCoefficients COEFFICIENTS = new PIDCoefficients(
            0,
            0,
            0
    );

    public static double MAX_V = 0;
    public static double MAX_A = 0;
    public static double MAX_J = 0;

    public static final double RADIUS = 1;
    public static final double TICKS_PER_REVOLUTION = 100;

    private MotionProfile profile;
    private PIDFController controller;

    ElapsedTime time;

    private DcMotor motor;


    public void Lift (HardwareMap map){
        motor = map.get(DcMotor.class, "liftMotor");
        time = new ElapsedTime();
        controller = new PIDFController(COEFFICIENTS, K_V, K_A, K_STATIC, (x, v) -> G);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goToPosition(0);
    }

    private double getPosition(){
        double rotations = motor.getCurrentPosition() / TICKS_PER_REVOLUTION;
        return rotations * 2 * Math.PI * RADIUS;
    }

    public void goToPosition(double targetPosition){
        MotionState start;

        if (profile != null){
            start = profile.get(time.seconds());
        }
        else{
            start = new MotionState(getPosition(), 0, 0, 0);
        }
        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                start,
                new MotionState(targetPosition, 0, 0, 0),
                MAX_V,
                MAX_A,
                MAX_J
        );
        time.reset();
    }

    public void update (){
        MotionState target = profile.get(time.seconds());

        controller.setTargetPosition(target.getX());
        controller.setTargetVelocity(target.getV());
        controller.setTargetAcceleration(target.getA());

        motor.setPower(controller.update(getPosition()));
    }

}
