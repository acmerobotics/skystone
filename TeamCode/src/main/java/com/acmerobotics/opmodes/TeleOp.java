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
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.util.JoystickTransform;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        armEncoder arm = new armEncoder(hardwareMap);
        BurlingameLift lift = new BurlingameLift(hardwareMap);
        Drive drive = new Drive(hardwareMap);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        JoystickTransform transform = new JoystickTransform();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        StickyGamepad stickyGamepad;

        arm.init();
        lift.init();

        arm.resetEncoder();
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


            /////////////////////////// ARM //////////////////////////

            if (gamepad2.a){
                arm.runTo(arm.grabPosition);
            }

            if (gamepad2.x){
                arm.runTo(arm.underBridge);
            }

            if (gamepad2.y){
                arm.runTo(arm.liftPosition);
            }

            if (gamepad2.b){
                arm.runTo(arm.allTheWayPosition);
            }

            ///////////////////////////////////////////////////////////////

            ////////////////////////////// HAND ////////////////////////////


            if (gamepad2.right_bumper){
                arm.setHand("open");
            }

            if (gamepad2.left_bumper){
                arm.setHand("close");
            }

            ///////////////////////////////////////////////////////////////

            dashboardTelemetry.addData("target position", arm.armMotor.getTargetPosition());
            dashboardTelemetry.addData("current position", arm.armMotor.getCurrentPosition());
            dashboardTelemetry.addData("pid", arm.armMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));

            dashboardTelemetry.update();

        }

    }

}

