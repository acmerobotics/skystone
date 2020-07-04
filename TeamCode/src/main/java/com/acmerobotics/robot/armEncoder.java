package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class armEncoder {

    public DcMotorEx armMotor;
    public Servo handServo;

    private double TICKS_PER_REV_MOTOR = 560;

    private double TICKS_PER_REV_GEAR = TICKS_PER_REV_MOTOR * 2;

    private double TICKs_PER_DEGREE = TICKS_PER_REV_GEAR / 360;

    private double ARM_MOTOR_DIAMETER = 1;

    public static double armPower = 1;

    private int gearRatio = 2;

    //^^^^^^^^^^^^^^^^^^^^ used in encoder math ^^^^^^^^^^^^^^^^^^^//
    private double armLength = 14;

    //^^^^^^^^^^^^^^^^^^^^ used in math ^^^^^^^^^^^^^^^^^^^^^^^^^^^//

    private double ticksPerRev = 4000;
    private double error;

    public static double Pcoefficient = 0;
    public static double Icoefficient = 0;

    public double loopCount = 0;

    public double adder = 1;

    public double Ii = 0;

    public static double maxL = 0;

    //^^^^^^^^^^^^^^^^^^^^^^ used in p controller ^^^^^^^^^^^^^^^^^//


    public static double P = 20; // 12
    public static double I = 0.25; // 0.5
    public static double D = 0;
    public static double F = 0;

    public static PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F, MotorControlAlgorithm.LegacyPID);


    private double handOpenPos = 0.59;
    private double handClosePos = 0.1;


    //^^^^^^^^^^^^general variables^^^^^^^^^^^^^^^^^^^^//


    public armEncoder(HardwareMap hardwareMap){

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        handServo = hardwareMap.get(Servo.class, "handServo");

    }

    ////////////////////////////// encoder setup and main methods //////////////////////////////////

    public void init(){
        // motor added to hardware map, arm floats to init position and
        // is set to 0 power and is held there by RUN_USING_ENCODER

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        armMotor.setPower(0);
        armMotor.setTargetPosition(0);

        //setPID(coefficients)

        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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

        armMotor.setPower(armPower);
    }


    public void setPID(PIDFCoefficients pidfCoefficients){
        //will set the pid coefficients

        // should only set the pid coefficients if they are different from the current ones
        if (pidfCoefficients != coefficients) {
            armMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
            coefficients = pidfCoefficients;
        }
    }

    public void setHand(String position){

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



    ////////////////////////// hand inches from ground math code ////////////////////////////

    public double hypotenuseOfPath(double desiredInches){
        double hypotenuse = Math.pow(armLength, 2) + Math.pow(desiredInches, 2);
        return hypotenuse;
    }

    public double armAngleOfPath(double hypotenuse){

        double angle = (Math.pow(hypotenuse, 2) - Math.pow(armLength, 2) - Math.pow(armLength, 2)
                / (-2 * armLength *  armLength));  // law of cosine solving for angle

        angle = Math.acos(angle);

        return angle;

    }


    public double angleToTicks(double angle){
        double ticks = angle * TICKs_PER_DEGREE;

        return ticks;
    }


    public double moveTo(double inches){  /// final method with all math concluded
        double hypotenuse = hypotenuseOfPath(inches);

        double angle = armAngleOfPath(hypotenuse);

        double ticks = angleToTicks(angle);

        return ticks;
    }


    //////////////////////////^ end of hand inches from ground math code ^////////////////////////////





    ///////////////////////////// degree angle math code (discontinued) /////////////////////////////

    public int heightToEncoder(double height){

        /*
         * This is the new and final version ot the angle math code. A triangle is created with the
         * users desired arm height. The angle is found of that triangle then translated to encoder
         * ticks.
         */

        double degreesPerTick = TICKS_PER_REV_MOTOR / 360;

        double armAngle = Math.ceil( 2 * Math.asin((height/2) / armLength)); // calculate angle arm needs to move

        double motorAngle = armAngle * 2; // calculate motor angle based on arm and motor gear ratio.

        double ticks = motorAngle * degreesPerTick; // translate the angle to ticks

        return (int) ticks;
    }

    private double ticksPerInch(){
        // number of ticks per inch based on the arm gear circumference

        return TICKS_PER_REV_GEAR/ Math.PI * ARM_MOTOR_DIAMETER;
        // (to test if this is really ticks per inch: take the number that results from the equation above (x) and
        // multiply it by the circumference, this number will tell you the total number of ticks around the circumference.)
    }


    private double arcLength(double angle ){
        // get the length of the arc, in inches, of the arm motor gear based on an angle

        return (angle/ 360) * (Math.PI * ARM_MOTOR_DIAMETER);
        //     ^portion of circumference          ^circumference
    }


    private double arcInchesToTicks(double angle){
        // get ticks of arc

        double ticksPerInch = ticksPerInch();

        double arcLength = arcLength(angle);

        double ticksForArc = ticksPerInch * arcLength;

        return ticksForArc; // ticks to get to the arc length, arc is based on given angle. (result is for motor gear)
    }


    private double toArmGearTicks(double angle){
        // convert motor gear ticks to the ticks needed to get the same result on the arm gear, this is done
        // by multipling the motor gear ticks by the 2 because the arm to motor gear ratio is 2:1

        double motorGearTicks = arcInchesToTicks(angle);

        double armGearTicks = motorGearTicks * gearRatio;

        return armGearTicks;
    }


    private int setEncoderTicks(double angle){
        //final step of encoder math, by now the ticks of the arc have been found for the motor gear
        // and this has been converted to find the ticks needed for the arm gear which is 2 times
        // more than the motor gear arc tick count.

        return (int) toArmGearTicks(angle);
    }


    public void encoderRunTo(double angle){

        int position = setEncoderTicks(angle);

        runTo(position);
    }

    //////////////////////////////////////^ degree angle math code math ^/////////////////////////////////////


    ///////////////////////////////////// P controller ////////////////////////////////////////////////

    public void updateError(double target){
        error = target - armMotor.getCurrentPosition();
    }

    public void updateLoopCount(){
        loopCount += adder;
    }

    public void controller(double target){
        updateError(target);

        if (error == 0){
            loopCount = 0;
        }

        double P = error * Pcoefficient;

        double L = error * loopCount * Icoefficient;
        Ii = Ii + L;

        if (Ii > maxL){
            Ii = maxL;
        }

        if (Ii < -maxL){
            Ii = -maxL;
        }

        double output = P + I;

        armMotor.setPower(output);
    }
}
