package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Intake;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Intake Test")
public class TestIntake extends LinearOpMode {


    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;


    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);


        waitForStart();

        while (!isStopRequested()){

            if(gamepad1.left_bumper){

                if (isLeftBumperPressed == false){

                    isLeftBumperPressed = true;

                    if (isLeftOpen == false){
                        intake.leftOpen();
                        isLeftOpen = true;

                    } else {
                        isLeftOpen = false;
                        intake.leftClose();

                    }

                }


            } else {

                isLeftBumperPressed = false;
            }

            if(gamepad1.right_bumper){

                if(isRightBumperPressed == false){
                    isRightBumperPressed = true;

                    if (isRightOpen == false) {
                        intake.rightOpen();
                        isRightOpen = true;

                    } else {
                        isRightOpen = false;
                        intake.rightClose();
                    }

                }


            } else {

                isRightBumperPressed = false;
            }

            intake.setIntakePower(gamepad1.left_stick_y);



        }
    }
}
