package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.hardware.CachingDcMotorEx;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.robomatic.util.PIDController;
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

    //PID
    private PIDController pidController;
    public double error;

    // PID var
    private static double P = 20; // 12
    private static double I = 0.25; // 0.5
    private static double D = 0;

    // handServo pos
    private double handOpenPos;
    private double handClosePos;

    // arm
    private int armPosition;

    private enum State {
        UNKNOWN,
        OPEN,
        CLOSE,
        GO_TO_POS
    }

    private State state = State.UNKNOWN;

    public roboArm(Robot robot){
        super("Arm"); // calls Subsystem constructor

        pidController = new PIDController();

        // creates obj
        armMotor = robot.getMotor("armMotor");
        handServo = robot.getServo("handServo");
    }


    @Override
    public void update(Canvas overlay){
        // telemetry will be sent to dashboard as TelemetryPackets and the data wil have "Arm" prefix
        telemetryData.addData("current pos", armMotor.getCurrentPosition());
        telemetryData.addData("target pos", armMotor.getTargetPosition());

        switch (state) {

            case UNKNOWN:

                pidController = new PIDController(P, I, D);

                double target;
                double correction;


                break;

            case OPEN:

                handServo.setPosition(handOpenPos);


                break;


            case CLOSE:

                handServo.setPosition(handClosePos);

                break;


            case GO_TO_POS:

                pidController = new PIDController(P, I, D);

                target = armMotor.getCurrentPosition() + armPosition;
                error = target - armMotor.getCurrentPosition();

                correction = pidController.update(error);

                for (int i= 0; i < 4; i++) {
                    armMotor.setPower(correction);
                }

                break;
        }

    }


    public void init(){
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        pidController = new PIDController(P, I, D);

        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void resetEncoder(){
        // motor's current encoder position is set as the zero position

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void runTo(int position) {
        armPosition = position;
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        state = State.GO_TO_POS;
    }


    public void openHand() {
        handOpenPos = 0.59;
        state = State.OPEN;

    }

    public void closeHand(){
        handClosePos = 0.1;
        state = State.CLOSE;


    }


}
