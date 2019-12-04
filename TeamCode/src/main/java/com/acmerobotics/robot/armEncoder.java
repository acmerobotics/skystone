package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class armEncoder {

    public DcMotorEx armMotor;

    //TODO///////////// NEED REAL VALUES FOR VARIABLES, ALL OF THESE ARE FOR TESTING ONLY //////////

    private static double TICKS_PER_REV = 280;

    private double ARM_MOTOR_DIAMETER = 1;

    private int gearRatio = 2;

    public double testAngle = 10;

    public int testEncoderPosition = 0;

    //^^^^^^^^^^^^^^^^^^^^used in encoder math^^^^^^^^^^^//

    public int testPosition1 = 220;

    public String initialized = "";
    public String reset = "";
    public String leave = "";
    public String runningTo = "running to position";
    public String positionReached = "position reached";


    //^^^^^^^^^^^^^^^used in encoder test^^^^^^^^^^^^^^^//

    public double thePower = 0.95;

    private PIDFCoefficients motorCoefficients;


    //^^^^^^^^^^^^general variables^^^^^^^^^^^^^^^^^^^^//


    public void armEncoder(){
    }

    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(HardwareMap hardwareMap){
        // motor added to hardware map, arm floats to init position and
        // is set to 0 power and is held there by RUN_USING_ENCODER

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initialized = "initialized";
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        reset = "encoders stopped and reset";
    }


    public void leaveReset(){
        // motor mode is set to RUN_USING_ENCODER to get motor out of STOP_AND_RESET mode
        // motor will just continue to hold a power of 0

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leave = "left stop and reset encoder";
    }


    public void runTo(int position, double power) {
        // target position is set and the motor is set to run to that position and a set speed/ power
        // target position is held with pid

        position = armMotor.getCurrentPosition() + position;

        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //setPID(motorCoefficients);

        armMotor.setPower(power);
    }


    public void stopMotors(){
        // motor is stopped and its power is set to 0 so it will just fall to its init position and
        // hold that position
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPID(PIDFCoefficients pidfCoefficients){
        //will set the pid coefficients, it is likely that only p will need to be changed for now
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }


    ///////////////////////////^ end of encoder setup and main methods ^//////////////////////////////








    ///////////////////////////// math to get ticks to move at x angle /////////////////////////////

    public double ticksPerInch(){
        // number of ticks per inch based on the arm gear circumference

        return TICKS_PER_REV/ Math.PI * ARM_MOTOR_DIAMETER;
        // (to test if this is really ticks per inch: take the number that results from the equation above (x) and
        // multiply it by the circumference, this number will tell you the total number of ticks around the circumference.)
    }


    public double arcLength(double angle ){
        // get the length of the arc, in inches, of the arm motor gear based on an angle

        return (angle/ 360) * (Math.PI * ARM_MOTOR_DIAMETER);
        //     ^portion of circumference          ^circumference
    }


    public int arcInchesToTicks(double angle){
        // get ticks of arc

        double ticksPerInch = ticksPerInch();

        double arcLength = arcLength(angle);

        int ticksForArc = (int) ticksPerInch * (int) arcLength;

        return ticksForArc; // ticks to get to the arc length, arc is based on given angle. (result is for motor gear)
    }


    public int ToArmGearTicks(double angle){
        // convert motor gear ticks to the ticks needed to get the same result on the arm gear, this is done
        // by multipling the motor gear ticks by the 2 because the arm to motor gear ratio is 2:1

        int motorGearTicks = arcInchesToTicks(angle);

        int armGearTicks = motorGearTicks * gearRatio;

        return armGearTicks;
    }


    public int setEncoderTicks(double angle){
        //final step of encoder math, by now the ticks of the arc have been found for the motor gear
        // and this has been converted to find the ticks needed for the arm gear which is 2 times
        // more than the motor gear arc tick count.

        return ToArmGearTicks(angle);
    }


    public void encoderRunTo(double angle){
        int position = setEncoderTicks(angle);

        testEncoderPosition = position;/////////////////////////////////// used for a telemetry test

        runTo(position, thePower);
    }

    //////////////////////////////////////^ end of encoder math ^/////////////////////////////////////
}
