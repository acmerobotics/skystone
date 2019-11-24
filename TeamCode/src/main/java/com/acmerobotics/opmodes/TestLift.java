package com.acmerobotics.opmodes;

import com.acmerobotics.robot.BurlingameLift;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Lift Testing")
public class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BurlingameLift lift = new BurlingameLift(hardwareMap);

        lift.init();
        lift.resetEncoder();

        waitForStart();

        while(!isStopRequested()){

            lift.driverControlled(gamepad1.left_stick_y);

            if(gamepad1.y){
                lift.goToBottom();
            }

            if(gamepad1.b){
                lift.goToIntake();
            }

            telemetry.addData("encoder pos", lift.checkEncoder());
            telemetry.update();

        }

    }
}
