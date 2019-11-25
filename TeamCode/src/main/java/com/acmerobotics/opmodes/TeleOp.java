package com.acmerobotics.opmodes;

import com.acmerobotics.robot.BurlingameLift;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.robot.Lift;
import com.acmerobotics.robot.ArmSimple;
import com.acmerobotics.util.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;
    private boolean isUpDown = false;
    private boolean isDownDown = false;
    private boolean isLeftDown = false;
    private boolean isRightDown = false;
    private boolean isYDown = false;
    private boolean isYDown2 = false;

    public double thePower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap);
        ////////////////////////////////////////Lift lift = new Lift(hardwareMap);
        ArmSimple arm = new ArmSimple(hardwareMap);
       BurlingameLift lift = new BurlingameLift(hardwareMap);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        arm.init();
        lift.init();
        lift.resetEncoder();

        waitForStart();


        while (!isStopRequested()){

            ////////////////////// gamepad1   /////////////////////////////

            drive.setPower(new Vector2d(gamepad1.left_stick_y,- gamepad1.left_stick_x), gamepad1.right_stick_x);

            if (gamepad1.a){
                foundationMover.moveToGrab();
            }

            if (gamepad1.b){
                foundationMover.moveToStore();
            }

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

            intake.setIntakePower(-gamepad1.left_trigger);
            intake.setIntakePower(gamepad1.right_trigger);

            /*
            if (gamepad1.y){
                isYDown = true;
            } else if (isYDown) {
                intake.leftOpenAllWay();
                intake.rightOpenAllWay();
            }
            */



            ///////////////////// gamepad2   ///////////////////////////
            if (gamepad2.dpad_up){
                isUpDown = true;

            } else if (isUpDown){
                lift.goToIntake();
                isUpDown = false;
            }

            if (gamepad2.dpad_down){
                isDownDown = true;

            } else if (isDownDown){
                lift.goToBottom();
                isDownDown = false;
            }

            if (gamepad2.dpad_right) {
                isRightDown = true;

            } else if (isRightDown) {
                lift.adjustLiftUp();
                isRightDown = false;
            }

            if (gamepad2.dpad_left) {
                isLeftDown = true;

            } else if (isLeftDown) {
                lift.adjustLiftDown();
                isLeftDown = false;
            }


            if (gamepad2.x){
                thePower = arm.armMotor.getPower();
                arm.armMotor.setPower(thePower);
            }

            else {
                arm.setMotorPower(gamepad2.left_stick_y);
            }

            if (gamepad2.right_bumper){
                arm.setHand("close");
            }

            if (gamepad2.left_bumper){
                arm.setHand("open");
            }

            telemetry.addData("the power", thePower);
            telemetry.update();


        }

    }

}

