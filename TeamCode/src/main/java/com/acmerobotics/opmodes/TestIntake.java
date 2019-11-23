package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Intake;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Intake Test")
public class TestIntake extends LinearOpMode {

    public boolean isXPressed = false;
    public boolean isBPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);


        waitForStart();

        while (!isStopRequested()){

            if(gamepad1.left_bumper){
                isXPressed = true;

            } else if (isXPressed) {

                intake.leftOpen();

                Thread.sleep(500);

                intake.rightOpen();
            }

            if(gamepad1.b){
                isBPressed = true;

            } else if (isBPressed) {

                intake.leftClose();

                Thread.sleep(500);

                intake.rightClose();
            }

            intake.setIntakePower(gamepad1.left_stick_y);



        }
    }
}
