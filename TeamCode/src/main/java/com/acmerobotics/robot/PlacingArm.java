package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.robot.Lift;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;


@Config
public class PlacingArm {

    //TODO add feedforward method and add feedforward to RUN_TO_POSITION

    //ToDo find the vales to all the empty variables

    public static double ARM_LENGTH = 15.375;

    public static double ARM_INIT;
    public static double ARM_INTAKE;
    public static double ARM_RELOCATION;

    public static double ARM_MASS = 0;

    public static double initAngle;
    public static double intakeAngle;
    public static double relocationAngle;
    public static double restingAngle = 0; //angle where motor is doing nothing and the arm is resting

    public static double wantInitAngle = 0; //find angle
    public static double wantIntakeAngle = 0; //find angle
    public static double wantRelocationAngle = 20;

    public double targetPosition;

    private static final double TICK_COUNT_PER_REVOLUTION = 200;

    private static final double DIAMETER_OF_MOTOR_GEAR = 1;
    private static final double TICKS_PER_INCH_OF_MOTOR_GEAR = TICK_COUNT_PER_REVOLUTION/ DIAMETER_OF_MOTOR_GEAR * Math.PI;

    private static final double DIAMETER_OF_ARM_GEAR = 2;
    private static final double TICKS_PER_INCH_OF_ARM_GEAR = TICK_COUNT_PER_REVOLUTION/ DIAMETER_OF_ARM_GEAR * Math.PI; //figure out if drive gear reduction is needed

    public boolean Here = false;

    private DcMotorEx armMotor;
    private Servo handServo;

    private double handOpenPos = 0; //add angle position at which hand will open
    private double handClosePos = 0; // add angle position at which hand will close

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double G = 0;


    public PlacingArm(HardwareMap hardwareMap){

        armMotor = hardwareMap.get(DcMotorEx.class, "Arm Motor");
        handServo = hardwareMap.get(Servo.class, "hand Servo");


        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    public void resetEncoder(){
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


    }

    public double checkEncoder(){
        return armMotor.getCurrentPosition();

    }

    public void setMotorEncoders(double angle) {
        int moveMotorTo = armMotor.getCurrentPosition() + convertToTicks(angle);
        int moveGearTo = moveMotorTo *2;//////////////////////////////////////////////////////////
        armMotor.setTargetPosition(moveGearTo);///////////////////////////////////////////////////
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        targetPosition = moveGearTo;///////////////////////////////////////////////////////////////
    }

    public int convertToTicks(double angle){
        double distance = getArmWheelArc(angle);

        int numToTicks = (int)((distance) * TICKS_PER_INCH_OF_MOTOR_GEAR);
        return numToTicks;
    }

    public void goToPosition(double angle){

        //TODO check math, look into the 2:1 gear ratio and how it affect the encoder tick count

        armMotor.setPower(.75);

        setMotorEncoders(angle);
        if (armMotor.getCurrentPosition() == targetPosition){
            Here = true;

        }

    }



    public void armRelocationPosition(){
        //Arm angle is set to 90 degrees. Motion profiling and pid are initialized. Arm moves to relocation position

        relocationAngle = getActualAngle(wantRelocationAngle, restingAngle);

        //ARM_RELOCATION = getRadianLen(relocationAngle, ARM_LENGTH);

        goToPosition(relocationAngle);

    }

    public double getRadianLen(double angle, double radius){
        // returns radian length (arm movement curve length)

        double i =  (angle/360) * 2 * Math.PI * radius;
        return i;
    }

    public double getArmWheelArc(double angle){
        double arcLength = getRadianLen(angle, (DIAMETER_OF_ARM_GEAR/2));
        return arcLength;
    }

    public void setServo(String position){
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

    public double getActualAngle(double angle, double restingAngle){
        // angle is the angle you want the arm to be placed when the lift is angle 0. a is the angle that will actual work with goToPosition
        // look at "Actual Angel" paper for details.
        double a = angle - restingAngle;
        return a;
    }

}