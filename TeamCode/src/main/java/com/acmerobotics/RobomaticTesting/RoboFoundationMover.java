package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoboFoundationMover extends Subsystem {

    private Servo servo;

    private double grabbingPos;
    private double storingPos;

    public enum State {
        GRAB,
        STORE,
        UNKNOWN
    }

    public State state = State.UNKNOWN;


    public RoboFoundationMover(roboRobot robot){
        super("foundationMover");

        servo = robot.getServo("foundationServo");
    }

    @Override
    public void update(Canvas fieldOverlay) {

        switch (state) {

            case UNKNOWN:


                break;

            case GRAB:

                servo.setPosition(grabbingPos);

                break;

            case STORE:

                servo.setPosition(storingPos);

                break;

        }

    }

    public void grab(){
        grabbingPos  = 0.19;
        state = State.GRAB;

    }

    public void store(){
        storingPos = 0.6;
        state = State.STORE;

    }

}
