package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.roboRobot;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name="roboTeleOp")
public class roboTeleOp extends LinearOpMode {

    public boolean isLeftBumperPressed = false;
    public boolean isLeftOpen = false;
    public boolean isRightBumperPressed = false;
    public boolean isRightOpen = false;
    private boolean isFullyOpen = false;
    private boolean isIntakeReady = false;

    public boolean isRightTriggerPressed = false;
    public boolean isLeftTriggerPressed = false;

    private boolean isDpadUp = false;
    private boolean isDpadDown = false;

    private boolean isYPressed = false;

    private boolean armReady = false;


    private int blocks = 0;

    public static int foundation = 155; // 165
    public static int underB = 110; // 135


    public static int oneExtraBlock = 220;
    public static int twoExtraBlock = 250;

    private boolean timeReset = false;

    public int extraBlocks = 0;

    @Override
    public void runOpMode(){

        roboRobot robot = new roboRobot(this);

        ElapsedTime time = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        robot.arm.resetEncoder();

        while (true) {

            if (!robot.lift.bottomSet) {
                robot.arm.runTo(100);

                robot.lift.tightenLiftString();

                robot.lift.goToBottom();

            } else {
                break;
            }

            telemetry.addData("current position ", robot.lift.liftMotor1.getCurrentPosition());
            telemetry.addData("target position ", robot.lift.setPoint);

            telemetry.addLine();

            telemetry.addData("bottom set", robot.lift.bottomSet);

            telemetry.addLine();

            telemetry.addData("step", 0);

            telemetry.update();
        }

        robot.lift.resetEncoder();

        robot.arm.runTo(100); // gets arm out of the intake's way

        while (true) {

            if (robot.arm.armMotor.getCurrentPosition() > 80){

                if (timeReset == false){
                    time.reset();
                    timeReset = true;
                }

                robot.intake.rightFullyOpen();
                isRightOpen = true;

                if (time.seconds() > 1.25) {
                    robot.intake.leftFullyOpen();
                    isLeftOpen = true;
                    isFullyOpen = true;
                    timeReset = false;
                    break;
                }
            }

            telemetry.addData("step", 1);

            telemetry.update();

        }


        while(true){
            robot.lift.PController();

            if (timeReset == false){
                time.reset();
                timeReset = true;
            }

            if (time.seconds() > 1.25) {

                robot.lift.goToStartHeight(); // raise lift so arm is ready for blocks coming in from intake

                robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.armMotor.setPower(0.08); // arm goes to place where the 0 position will be

                if (robot.lift.liftMotor1.getCurrentPosition() >= (robot.lift.setPoint - 50)) {

                    if(!armReady) {
                        robot.arm.resetEncoder();
                        robot.arm.setHand("open");
                        armReady = true;
                    }

                    break;
                }
            }

            telemetry.addData("current position ", robot.lift.liftMotor1.getCurrentPosition());
            telemetry.addData("target position ", robot.lift.setPoint);

            telemetry.addLine();

            telemetry.addData("step", 2);

            telemetry.update();
        }

        waitForStart();

        while(!isStopRequested()){
            robot.lift.PController();

            //////////////////////////////////// gamepad1   //////////////////////////////////////////

            Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            robot.drive.setPower(v);

            if (gamepad1.left_bumper) {

                if (isLeftBumperPressed == false) {

                    isLeftBumperPressed = true;

                    if (isRightOpen == false) {

                        robot.intake.rightOpen();
                        isRightOpen = true;

                    } else {

                        robot.intake.rightClose();
                        isRightOpen = false;
                        isFullyOpen = false;
                    }

                }


            } else if (!gamepad1.left_bumper && !isFullyOpen && isLeftBumperPressed) {

                isLeftBumperPressed = false;

                if (isRightOpen == true){

                    robot.intake.leftOpen();
                    isLeftOpen = true;

                } else {
                    robot.intake.leftClose();
                    isLeftOpen = false;

                }
            }

            if (gamepad1.right_bumper) {

                if (isRightBumperPressed == false) {

                    isRightBumperPressed = true;

                    if(isFullyOpen == false) {

                        isFullyOpen = true;

                        robot.intake.rightFullyOpen();
                        robot.intake.leftFullyOpen();

                    } else {

                        isFullyOpen = false;

                        robot.intake.rightOpen();
                        robot.intake.leftOpen();
                    }
                }


            } else {

                isRightBumperPressed = false;
            }


            if (gamepad1.left_trigger > 0) {

                robot.intake.setIntakePower(-1);


            } else if (gamepad1.right_trigger > 0){

                robot.intake.setIntakePower(1);

            } else {

                robot.intake.setIntakePower(0);

            }

            /////////////////////////////////////// gamepad2   /////////////////////////////////////////////


            ////////////////////// Main Lift Code ///////////////////////
            if (gamepad2.dpad_up) {
                if (isDpadUp == false) {

                    isDpadUp = true;

                    isDpadDown = false;

                    blocks += 1;
                    //lift.setPosition(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);
                    robot.lift.runToBlocks(blocks, robot.lift.liftPower);
                }
            }

            else{
                isDpadUp = false; // allows runTo to be used after every press
            }


            if (gamepad2.dpad_down) {

                if (isDpadDown == false) {

                    isDpadDown = true;

                    isDpadUp = false;

                    blocks -= 1;
                    //lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);
                    robot.lift.runToBlocks(blocks, robot.lift.liftPower);
                }
            }

            else{
                isDpadDown = false;
            }


            ////////////////////// Lift Error Prevention /////////////////////////


            if (gamepad2.right_trigger > 0){
                if (!isRightTriggerPressed){
                    isRightTriggerPressed = true;
                    robot.lift.runToIncrement(200);
                }
            }

            else{
                isRightTriggerPressed = false;
            }


            if (gamepad2.left_trigger > 0){
                if (!isLeftTriggerPressed){
                    isLeftTriggerPressed = true;
                    robot.lift.runToIncrement(-200);
                }
            }

            else{
                isLeftTriggerPressed = false;
            }

            //////////////////////// ARM //////////////////////////

            if (gamepad2.a){
                //starting height, arm at rest (at hard stop)

                //hand will grab block

                robot.lift.setPosition(liftEncoder.startHeight);

                robot.arm.runTo(8);

                robot.arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.arm.armMotor.setPower(0.08);
            }


            if (gamepad2.x){
                // lift goes to bottom, arm moves to a position where it is greater than the foundation (2 in) and 1 in above
                // the foundation so it can block a block

                // allows robot to go under bridge

                int blockLifted = foundation;

                robot.arm.runTo(blockLifted);
                robot.lift.setPosition(liftEncoder.bottomPosition); // using bottom position instead of 0
            }


            if (gamepad2.b){

                robot.arm.runTo(underB);
            }

            if (gamepad2.y){
                if (!isYPressed) {
                    isYPressed = true;

                    if (extraBlocks == 0) {
                        robot.arm.runTo(oneExtraBlock);
                        extraBlocks = 1;
                    }

                    else if (extraBlocks == 1){
                        robot.arm.runTo(twoExtraBlock);
                        extraBlocks = 0;
                    }
                }
            }

            else{
                isYPressed = false;
            }


            ////////////////////////////// HAND ////////////////////////////


            if (gamepad2.right_bumper){
                robot.arm.setHand("open");
            }

            if (gamepad2.left_bumper){
                robot.arm.setHand("close");
            }


            ////////////////////////// Telemetry //////////////////////////////

            telemetry.addData("bottom set", robot.lift.bottomSet);

            telemetry.addData("blocks", blocks);

            telemetry.addLine();

            telemetry.addData("lift position" ,robot.lift.liftMotor1.getCurrentPosition());
            telemetry.addData("lift target", robot.lift.setPoint);

            telemetry.addLine();

            telemetry.addData("lift power", robot.lift.liftMotor1.getPower());

            telemetry.addLine();

            telemetry.addData("heading", robot.drive.getAngle());

            telemetry.update();

        }

    }
}
