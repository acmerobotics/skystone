package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class PlacingArm {

    //TODO talk about moving this into the lift code. Might be worth it. Maybe idk

    public static double ARM_LENGTH = 15.375;

    public static double ARM_ZERO = 0;
    public static double ARM_INTAKE = 0;
    public static double ARM_RELOCATION = 0;

    public static double RADIUS = 0;

    private DcMotorEx armMotor;

    private PIDController pidController;

    public double offset;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    public PlacingArm(HardwareMap hardwareMap){

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        pidController = new PIDController(P, I, D);

    }


    public double getPosition(){
        return internalGetPosition() + offset;

    }

    public double internalGetPosition(){
        return ((armMotor.getCurrentPosition() / (armMotor.getMotorType().getTicksPerRev())) * Math.PI * RADIUS * 2);

    }

    public void setPower(double power){
        internalSetVelocity(power);

    }

    public void setPosition(double position){
        offset = position - internalGetPosition();

    }

    public void internalSetVelocity(double v){
        armMotor.setPower(v);

    }


    public void armZeroPosition(){
        setPosition(ARM_ZERO);
        setPower(1);
        pidController = new PIDController(P, I, D);

    }

    public void armIntakePosition(){
        setPosition(ARM_INTAKE);
        setPower(1);
        pidController = new PIDController(P, I, D);
    }

    public void armRelocationPosition(){
        setPosition(ARM_RELOCATION);
        setPower(1);
        pidController = new PIDController(P, I, D);

    }


}
