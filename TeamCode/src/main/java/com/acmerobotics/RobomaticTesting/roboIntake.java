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
    public Servo leftServo, rightServo;


    private double leftOpen = 0.7;
    private double leftClose = 0.99;

    private double rightOpen = 0.353;
    private double rightClose = 0.001;

    public double LfullyOpen = 0.47;
    private double RfullyOpen = 0.57;

    private enum State {
        UNKNOWN,
        OPEN,
        REVERSE_INTAKE,
        FULLY_OPEN,
        CLOSED
    }

    private State state = State.UNKNOWN;

    public roboIntake(Robot robot){
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

                setIntakePower(0);

                break;

            case OPEN:

                leftOpen();

                try {
                    Thread.sleep(1000);

                } catch(InterruptedException ex) {

                    Thread.currentThread().interrupt();  //this is the  best way I could think to get
                                                        //wheels to close without hitting each other
                                                        //i suppose I could have had two different states
                                                        //and then use thread.sleep() in teleop,
                                                        // but thats not as concise.
                }

                rightOpen();
                setIntakePower(1);

                break;

            case REVERSE_INTAKE:

                leftOpen();
                try {
                    Thread.sleep(1000);

                } catch(InterruptedException ex) {

                    Thread.currentThread().interrupt();
                }

                rightOpen();
                setIntakePower(-1);


                break;

            case FULLY_OPEN:

                leftFullyOpen();
                rightFullyOpen();


                break;

            case CLOSED:

                setIntakePower(0);
                leftClose();

                try {
                    Thread.sleep(1000);

                } catch(InterruptedException ex) {

                    Thread.currentThread().interrupt();
                }

                rightClose();


                break;

        }



    }

    public void setIntakePower(double intakePower) {
        intakeMotor.setPower(intakePower);
    }

    public void leftOpen() {
        leftServo.setPosition(leftOpen);
    }

    public void rightOpen() {
        rightServo.setPosition(rightOpen);
    }

    public void leftClose() {
        leftServo.setPosition(leftClose);
    }

    public void rightClose() {
        rightServo.setPosition(rightClose);
    }

    public void leftFullyOpen() {
        leftServo.setPosition(LfullyOpen);
    }

    public void rightFullyOpen() {
        rightServo.setPosition(RfullyOpen);
    }
}
