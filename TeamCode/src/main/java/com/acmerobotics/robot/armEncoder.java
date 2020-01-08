package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class armEncoder {

    public DcMotorEx armMotor;
    public Servo rotationServo;
    public Servo handServo;

    //TODO///////////// Find the needed angles //////////

    private double TICKS_PER_REV = 280;

    private double ARM_MOTOR_DIAMETER = 1;

    private int gearRatio = 2;

    public double testAngle = 10;

    //^^^^^^^^^^^^^^^^^^^^used in encoder math^^^^^^^^^^^//

    public double rotateCenter = 140/255;


    public static double P = 30;
    public static double I = 4; // 7
    public static double D = 0.25;
    public static double F = 0;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F, MotorControlAlgorithm.LegacyPID);

    private double handOpenPos = 0.78;
    private double handClosePos = 0.08;

    //0.47


    //^^^^^^^^^^^^general variables^^^^^^^^^^^^^^^^^^^^//


    public armEncoder(HardwareMap hardwareMap){

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        //rotationServo = hardwareMap.get(Servo.class, "rotationServo");
        handServo = hardwareMap.get(Servo.class, "handServo");

    }

    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(){
        // motor added to hardware map, arm floats to init position and
        // is set to 0 power and is held there by RUN_USING_ENCODER

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        armMotor.setPower(0);
        armMotor.setTargetPosition(0);

        //setPID(coefficients)

        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //rotationServo.setPosition(rotateCenter);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void leaveReset(){
        // motor mode is set to RUN_USING_ENCODER to get motor out of STOP_AND_RESET mode
        // motor will just continue to hold a power of 0

        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }


    public void runTo(int position) {
        // target position is set and the motor is set to run to that position and a set speed/ power
        // target position is held with pid

        setPID(coefficients);

        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armMotor.setPower(1);
    }


    public void setPID(PIDFCoefficients pidfCoefficients){
        //will set the pid coefficients
        armMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
    }

    public void setHand(String position){
        // take in close or open then set servo position accordingly

        if (position.equals("open")){
            //open hand
            handServo.setPosition(handOpenPos);
        }

        if (position.equals("close")){
            //close hand
            handServo.setPosition(handClosePos);
        }
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


    public double arcInchesToTicks(double angle){
        // get ticks of arc

        double ticksPerInch = ticksPerInch();

        double arcLength = arcLength(angle);

        double ticksForArc = ticksPerInch * arcLength;

        return ticksForArc; // ticks to get to the arc length, arc is based on given angle. (result is for motor gear)
    }


    public double toArmGearTicks(double angle){
        // convert motor gear ticks to the ticks needed to get the same result on the arm gear, this is done
        // by multipling the motor gear ticks by the 2 because the arm to motor gear ratio is 2:1

        double motorGearTicks = arcInchesToTicks(angle);

        double armGearTicks = motorGearTicks * gearRatio;

        return armGearTicks;
    }


    public int setEncoderTicks(double angle){
        //final step of encoder math, by now the ticks of the arc have been found for the motor gear
        // and this has been converted to find the ticks needed for the arm gear which is 2 times
        // more than the motor gear arc tick count.

        return (int) toArmGearTicks(angle); //TODO find out how to round up before converting to int
    }


    public void encoderRunTo(double angle){

        int position = setEncoderTicks(angle);

        runTo(position);
    }



    //////////////////////////////////////^ end of encoder math ^/////////////////////////////////////
}
