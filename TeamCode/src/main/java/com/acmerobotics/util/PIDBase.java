package com.acmerobotics.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/***

This is a FTC SDK PID Base for a single motor of a subsystem. This system will reduce the lines of code
that have to be written each season ,for motors using the SDK PID, because it puts repetitive SDK structures
together and makes use of general purpose methods.


 The methods that would go in a subsystem class are the Motor Setup, Run-methods, and unit of measurement converters.
 The rest, the private methods, are background methods the add some sort of functionality or structure to the other methods.

 ***/


public class PIDBase {

    // TODO investigate static var for PID and using the Dashboard in multiple objs. Will the changing a var affect the same var in all obj?
    // Need to be careful with the static variables bc they are dependent on the class not an obj.
    // I am wondering if the value are changed, then will the new values change across all obj change
    // If they do change, then use the static variables during tuning with the dashboard but with one
    // PIDBase being used at a time of variable changes will be constantly overwritten.
    public static double P = 10;
    public static double I = 0.05;
    public static double D = 0;
    public static double F = 0;

    public PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F, MotorControlAlgorithm.LegacyPID);

    private DcMotorEx motor;

    private double PIDMotorPower = 1;

    public PIDBase(){
    }

    //////////////////////////////////////// Setup /////////////////////////////////////////////////

    // Motor Setup

    public void addMotor(DcMotorEx motor){
        this.motor = motor;
    }

    public void resetEncoder(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // PID Setup

    private void setPID(){
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,pidfCoefficients);
    }

    private void setPIDRunAt(){
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    ///////////////////////////////////// Run-methods //////////////////////////////////////////////

    public void runTo(int position){
        setPID();

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setTargetPosition(position);

        motor.setPower(PIDMotorPower);
    }

    public void runtAt(double power){
        setPIDRunAt();

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setPower(power);
    }

    ///////////////////////// ticks and unit of measurement converters /////////////////////////////

    // (I can probably add this to its own class because, although useful, it does not fit the PIDBase title (tuConverter).

    // unit used depends on the units of the diameter given. Meaning unit could be anything (in, cm, and more).

    public double ticksToUnits(int ticksPerRev, int diameter, int ticks){
        double circumference = diameter * 3.14;

        double ticksPerU = (ticks * circumference) / ticksPerRev;

        return ticksPerU;
    }

    public int unitsToTicks(int ticksPerRev, int diameter, double units){
        double circumference = diameter * 3.14;

        double UnitPerT = (ticksPerRev * units) / circumference;

        return (int) (Math.round(UnitPerT));
    }

}
