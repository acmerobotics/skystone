package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.hardware.CachingDcMotorEx;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class roboArm extends Subsystem {

    // initialize motors
    public DcMotorEx armMotor;
    private Servo handServo;

    // PID var
    public static double P = 20; // 12
    public static double I = 0.25; // 0.5
    public static double D = 0;
    public static double F = 0;
    public static double armPower = 1;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(P, I, D, F, MotorControlAlgorithm.LegacyPID);

    // handServo pos
    private double handOpenPos = 0.59;
    private double handClosePos = 0.1;

    public roboArm(Robot robot){
        super("Arm"); // calls Subsystem constructor

        // creates obj
        armMotor = robot.getMotor("armMotor");
        handServo = robot.getServo("handServo");
    }


    @Override
    public void update(Canvas overlay){
        // telemetry will be sent to dashboard as TelemetryPackets and the data wil have "Arm" prefix
        telemetryData.addData("current pos", armMotor.getCurrentPosition());
        telemetryData.addData("target pos", armMotor.getTargetPosition());
    }


    public void init(){
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        setPID(coefficients);

        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void runTo(int position) {
        // target position is set and the motor is set to run to that position and a set power
        // target position is held with pid

        setPID(coefficients);

        armMotor.setTargetPosition(position);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        armMotor.setPower(armPower);
    }


    public void setPID(PIDFCoefficients pidfCoefficients){
        //will set the pid coefficients

        //todo add similar method to Robomatic

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


}
