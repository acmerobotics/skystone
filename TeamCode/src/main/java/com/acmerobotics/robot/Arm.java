package com.acmerobotics.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public DcMotorEx armMotor;
    private Servo handServo;
    private Servo rotationServo;

    private double handOpenPos = 0.78;
    private double handClosePos = 0.33;

    private double rotateRight = 1;
    private double rotateLeft = 0;
    private double rotateCenter = 0.53;

    private static final double TICK_COUNT_PER_REVOLUTION = 280;

    private static final double DIAMETER_OF_MOTOR_GEAR = 1;
    private static final double TICKS_PER_INCH_OF_MOTOR_GEAR = TICK_COUNT_PER_REVOLUTION/ DIAMETER_OF_MOTOR_GEAR * Math.PI;

    private static final double DIAMETER_OF_ARM_GEAR = 2;

    private int ratioMultiplier = 2;

    private double restingAngle = 0;

    public double position0 = 20; //test position
    public double position1 = 45; //test position

    public int targetPosition = 0;


    public Arm(){
    }

    public void init(HardwareMap hardwareMap){
        //armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        //handServo = hardwareMap.get(Servo.class, "handServo");
        //.rotationServo = hardwareMap.get(Servo.class, "rotationServo");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        setVelocity(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotationServo.setPosition(rotateCenter);

    }

    public void resetEncoder(){
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void leaveStopAndResetEncoder(){
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double checkEncoder(){
        return armMotor.getCurrentPosition();
    }

    public void stopEncoder(){
        armMotor.setPower(0);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void goToPosition(int position){
        if(position == 0) {
            setMotorEncoders(position0);

        }

        if(position == 1){
            setMotorEncoders(position1);
        }
    }


    public int convertToTicks(double angle){
        double distance = getRadianLen(angle, (DIAMETER_OF_MOTOR_GEAR));

        int TicksToMoveMotorGearByAngle = (int)((distance) * TICKS_PER_INCH_OF_MOTOR_GEAR);
        int TicksToMoveArmGearByAngle = TicksToMoveMotorGearByAngle * ratioMultiplier;

        int numToTicks = TicksToMoveArmGearByAngle;
        return numToTicks;
    }


    public void setMotorEncoders(double angle) {
//        int moveMotorTo = armMotor.getCurrentPosition() + convertToTicks(angle);
//        int moveGearTo = moveMotorTo;
//        targetPosition = moveGearTo;

        targetPosition = armMotor.getCurrentPosition() + 100;

        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(.95);

    }

    public void setVelocity(double power){
        armMotor.setPower(power);
    }


    public double getRadianLen(double angle, double diameter){
        // returns radian length (arm movement curve length)

        double i =  (angle/360) * Math.PI * diameter;
        return i;
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

//    public void setRotationServo(String position){
//        if (position.equals("right")){
//            //right
//            rotationServo.setPosition(rotateRight);
//        }
//
//        if (position.equals("left")){
//            //left
//            rotationServo.setPosition(rotateLeft);
//        }
//
//        if (position.equals("center")){
//            //center
//            rotationServo.setPosition(rotateCenter);
//        }
//    }

    public double getActualAngle(double angle, double restingAngle){
        // angle is the angle you want the arm to be placed when the lift is angle 0. a is the angle that will actual work with goToPosition
        // look at "Actual Angel" paper for details.
        double a = angle - restingAngle;
        return a;
    }

    public double setMotorPower(double power){
        return power * -0.80;
    }

    public void stopMotor(){
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }
}
