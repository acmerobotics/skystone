package com.acmerobotics.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ArmSimple;
import com.acmerobotics.robot.BurlingameLift;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.util.JoystickTransform;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;

    public double thePower = 0;

    public double incrementUp = 0.5;
    public double incrementDown = 0.25;

    public boolean stickUp = false;
    public boolean stickDown = false;

    public boolean incrementLock = false;

    private StickyGamepad stickyGamepad1, stickyGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        //Lift lift = new Lift(hardwareMap);
        ArmSimple arm = new ArmSimple(hardwareMap);
        BurlingameLift lift = new BurlingameLift(hardwareMap);
        Drive drive = new Drive(hardwareMap);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        JoystickTransform transform = new JoystickTransform();

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        arm.init();
        lift.init();
        lift.resetEncoder();

        waitForStart();


        while (!isStopRequested()){


            ////////////////////// gamepad1   /////////////////////////////

            //drive
            Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
            drive.setPower(v);

            //foundation mover
            if (stickyGamepad1.a){
                foundationMover.moveToGrab();
            }

            if (stickyGamepad1.b){
                foundationMover.moveToStore();
            }

            //intake
            if(stickyGamepad1.x){
                intake.leftClose();
            }

            if(stickyGamepad1.y){
                intake.rightClose();
            }

            if(gamepad1.left_bumper){

                if (isLeftBumperPressed == false){

                    isLeftBumperPressed = true;

                    if (isLeftOpen == false){
                        intake.leftOpen();
                        isLeftOpen = true;

                    } else {
                        isLeftOpen = false;
                        intake.leftOpenAllWay();

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
                        intake.rightOpenAllWay();
                    }

                }


            } else {

                isRightBumperPressed = false;
            }

            intake.setIntakePower(-gamepad1.left_trigger);
            intake.setIntakePower(gamepad1.right_trigger);


            ///////////////////// gamepad2   ///////////////////////////

            //lift
            if (stickyGamepad2.dpad_up) {
                lift.goToIntake();
            }

            if(stickyGamepad2.dpad_down) {
                lift.goToBottom();
            }

            if(stickyGamepad2.dpad_right) {
                lift.adjustLiftUp();
            }

            if(stickyGamepad2.dpad_left) {
                lift.adjustLiftDown();
            }

            //arm
            if (gamepad2.x){
                thePower = arm.armMotor.getPower();
                arm.armMotor.setPower(arm.stablePower);
            }


            else
            {
                if ((-gamepad2.left_stick_y) > 0.5) {
                    stickUp = true;
                    stickDown = false;
                    //arm.armMotor.setPower(0.4);

                }

                if ((-gamepad2.left_stick_y) < -0.5){
                    stickDown = true;
                    stickUp = false;
                    //arm.armMotor.setPower(0.1);
                }

                if (stickUp && !incrementLock) {
                    arm.armMotor.setPower(arm.armMotor.getPower() + incrementUp);
                    stickUp = false;
                    stickDown = false;
                    incrementLock = true;
                }

                if (stickDown && !incrementLock){
                    arm.armMotor.setPower(arm.armMotor.getPower() - incrementDown);
                    stickDown = false;
                    stickUp = false;
                    incrementLock = true;
                }

                if (gamepad2.left_stick_y == 0){
                    incrementLock = false;
                    stickUp = false;
                    stickDown = false;
                }
            }


            if (stickyGamepad2.right_bumper){
                arm.setHand("close");
            }

            if (stickyGamepad2.left_bumper){
                arm.setHand("open");
            }

            //update
            stickyGamepad1.update();
            stickyGamepad2.update();


        }

    }

}

