package com.acmerobotics.opmodes;

import com.acmerobotics.robot.CapstonePlacer;
import com.acmerobotics.robot.Intake;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="intakeTest")
public class intakeTest extends LinearOpMode {

    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;
    private boolean isFullyOpen = false;

    private int state = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        Intake intake = new Intake(hardwareMap);
        CapstonePlacer capstonePlacer = new CapstonePlacer(hardwareMap);


        waitForStart();


        while(!isStopRequested()){

            if (gamepad1.left_trigger > 0) {

                intake.setIntakePower(-1);


            } else if (gamepad1.right_trigger > 0){

                intake.setIntakePower(1);

            } else {

                intake.setIntakePower(0);

            }

            if (gamepad1.left_bumper) {

                if (isLeftBumperPressed == false) {

                    isLeftBumperPressed = true;

                    if (isRightOpen == false) {

                        intake.rightOpen();
                        state = 1;
                        isRightOpen = true;

                    } else {

                        intake.rightClose();
                        state = 2;
                        isRightOpen = false;
                        isFullyOpen = false;
                    }

                }


            } else if (!gamepad1.left_bumper && !isFullyOpen && isLeftBumperPressed) {

                isLeftBumperPressed = false;

                if (isRightOpen == true){

                    intake.leftOpen();
                    state = 3;
                    isLeftOpen = true;

                } else {
                    intake.leftClose();
                    state = 4;
                    isLeftOpen = false;

                }
            }

            if (gamepad1.right_bumper) {

                if (isRightBumperPressed == false) {

                    isRightBumperPressed = true;

                    if(isFullyOpen == false) {

                        isFullyOpen = true;

                        intake.rightFullyOpen();
                        intake.leftFullyOpen();
                        state = 5;

                    } else {

                        isFullyOpen = false;

                        intake.rightOpen();
                        intake.leftOpen();
                        state = 6;
                    }
                }


            } else {

                isRightBumperPressed = false;
            }


            if (gamepad1.dpad_down){
                capstonePlacer.moveToPlace();
            }

            if (gamepad1.dpad_up) {
                capstonePlacer.moveToStore();
            }

            telemetry.addData("state", state);
            telemetry.update();



        }

    }
}
