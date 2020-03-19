package com.acmerobotics.RobomaticTesting.IndividualTests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.util.PIDBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// NOTE: pid testing through the dashboard will occur through the PIDBase instance for each motor

public class runToTest extends Subsystem {

    public DcMotorEx motor1;
    public DcMotorEx motor2;

    private PIDBase motor1PID = new PIDBase();
    private PIDBase motor2PID = new PIDBase();

    public runToTest(Robot robot){
        super("runToTest");

        // drive motors
        motor1 = robot.getMotor("motor1");
        motor2 = robot.getMotor("motor2");

        // each motor has to be added to its own PID Base so they can independently interact with each other
        motor1PID.addMotor(motor1);
        motor2PID.addMotor(motor2);
    }

    @Override
    public void update(Canvas overlay){
        telemetryData.addData("current pos", motor1.getCurrentPosition());
        telemetryData.addData("target pos", motor1.getTargetPosition());
    }


    public void init(){
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void motor1MoveInches(){
        // turn m1 on rev

        int target = motor1PID.unitsToTicks(560, 4, 12.56); // unit is in inches because diameter is in inches

        motor1PID.runTo(target);
    }

    public void moveBoth(){
        // turn m1 and m2 each 2 rev

        motor1PID.runTo(1200);
        motor2PID.runTo(1200);
    }
}
