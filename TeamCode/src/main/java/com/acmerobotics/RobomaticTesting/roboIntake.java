package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config

public class roboIntake extends Subsystem {

    private DcMotorEx intakeMotor;
    private Servo leftServo, rightServo;


    private double leftOpen;
    private double leftClose;

    private double rightOpen;
    private double rightClose;

    private double leftFullyOpen;
    private double rightFullyOpen;

    private double intakePower;

    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;
    private boolean isFullyOpen = false;



    private enum State {
        UNKNOWN,
        OPEN_AND_CLOSE,
        RUN_INTAKE,
        FULLY_OPEN
    }

    private State state = State.UNKNOWN;

    public roboIntake(Robot robot) {
        super("Intake");

        intakeMotor = robot.getMotor("intakeMotor");

        leftServo = robot.getServo("leftServo");
        rightServo = robot.getServo("rightServo");
    }

    @Override
    public void update(Canvas overlay) {

        switch (state){

            //figure out how to make it so the intake wheel don't get stuck

            case UNKNOWN:

                //umm yeah idk what to put here. I guess there doesn't really need to be anything

                break;

            case OPEN_AND_CLOSE:

                if (isLeftBumperPressed == false) {

                    isLeftBumperPressed = true;

                    if (isRightOpen == false) {

                        rightServo.setPosition(rightOpen);
                        isRightOpen = true;

                    } else {

                        rightServo.setPosition(rightClose);
                        isRightOpen = false;
                        isFullyOpen = false;
                    }

                }


                if (!isFullyOpen && isLeftBumperPressed) {

                    isLeftBumperPressed = false;

                    if (isRightOpen == true){

                        leftServo.setPosition(leftOpen);
                        isLeftOpen = true;

                    } else {

                        leftServo.setPosition(leftClose);
                        isLeftOpen = false;

                    }
                }

                break;

            case FULLY_OPEN:

                if (isRightBumperPressed == false) {

                    isRightBumperPressed = true;

                    if(isFullyOpen == false) {

                        isFullyOpen = true;

                        rightServo.setPosition(rightFullyOpen);
                        leftServo.setPosition(leftFullyOpen);

                            } else {

                                isFullyOpen = false;

                                rightServo.setPosition(rightOpen);
                                leftServo.setPosition(leftOpen);
                            }

                } else {

                    isRightBumperPressed = false;
                }


                break;


            case RUN_INTAKE:

                intakeMotor.setPower(intakePower);


                break;

        }



    }

    public void setIntakePower(double power) {
        intakePower = power;
        state = State.RUN_INTAKE;
    }

    public void openAndClose(){
        leftOpen = 0.7;
        rightOpen = 0.353;
        leftClose = 0.99;
        rightClose = 0.001;
        state = State.OPEN_AND_CLOSE;
    }

    public void fullyOpen(){
        leftFullyOpen = 0.47;
        rightFullyOpen = 0.57;
        state = State.FULLY_OPEN;
    }

    public void leftFullyOpen() {
        leftFullyOpen = 0.47;
        state = State.FULLY_OPEN;
    }

    public void rightFullyOpen() {
        rightFullyOpen = 0.57;
        state = State.FULLY_OPEN;
    }
}
