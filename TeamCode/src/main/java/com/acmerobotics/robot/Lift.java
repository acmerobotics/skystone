package com.acmerobotics.robot;

import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    private DcMotorEx liftMotor1, liftMotor2;
    private DcMotorEx armMotor;

    public static double G = 0;
    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    private PIDController pidController;

    private enum LiftMode{
        INTAKE_POS,
        RELOCATION_POS,
        DRIVER_CONTROLLED,
        LOWERING,
        FIND_BOTTOM,

    }

    private LiftMode liftMode = LiftMode.DRIVER_CONTROLLED;

    public Lift(HardwareMap hardwareMap){

        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        pidController = new PIDController(P, I, D);

    }
}
