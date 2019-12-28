package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.robot.ArmSimple;
import com.acmerobotics.robot.BurlingameLift;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.robot.Lift;
import com.acmerobotics.robot.ArmSimple;
import com.acmerobotics.util.JoystickTransform;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
@Config
public class TeleOp extends LinearOpMode {

    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;
    private boolean isUpDown = false;
    private boolean isDownDown = false;
    private boolean isLeftDown = false;
    private boolean isRightDown = false;

    public double thePower = 0;

    public double incrementUp = 0.5;
    public double incrementDown = 0.25;

    public boolean stickUp = false;
    public boolean stickDown = false;

    public boolean incrementLock = false;

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

        StickyGamepad stickyGamepad;

        arm.init();
        lift.init();
        lift.resetEncoder();

        waitForStart();


        while (!isStopRequested()){


            ////////////////////// gamepad1   /////////////////////////////

            Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
            drive.setPower(v);

            //drive.setPower(new Vector2d(gamepad1.left_stick_y, -gamepad1.left_stick_x), gamepad1.right_stick_x);

            if (gamepad1.a){
                foundationMover.moveToGrab();
            }

            if (gamepad1.b){
                foundationMover.moveToStore();
            }

            if(gamepad1.x){
                intake.leftClose();
            }

            if(gamepad1.y){
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


            if (gamepad2.right_bumper){
                arm.setHand("close");
            }

            if (gamepad2.left_bumper){
                arm.setHand("open");
            }


        }

    }

}

