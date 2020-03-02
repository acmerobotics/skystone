package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.liftEncoder;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.acmerobotics.robot.Intake;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.util.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.robomatic.util.StickyGamepad;
import com.acmerobotics.util.JoystickTransform;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
@Config

public class TeleOp extends LinearOpMode {

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

    private boolean is1YPressed = false;
    private boolean slowDrive = false;

    private boolean isYPressed = false;

    private boolean armReady = false;


    private int blocks = 0;

    public static int foundation = 165;
    public static int lower = 145;

    public static int oneExtraBlock = 220;
    public static int twoExtraBlock = 250;

    private boolean timeReset = false;

    public int extraBlocks = 0;


    private StickyGamepad stickyGamepad1, stickyGamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap, true);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        JoystickTransform transform = new JoystickTransform();
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        ElapsedTime time = new ElapsedTime();
        
        /////////////////////////////////FtcDashboard dashboard = FtcDashboard.getInstance();
        /////////////////////////////Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (true) {
            if (!lift.bottomSet) {
                arm.runTo(110);

                lift.tightenLiftString();

                lift.goToBottom();

            } else {
                break;
            }
        }

        lift.resetEncoder();

        arm.runTo(130); // gets arm out of the intake's way

        while (true) {

            if (arm.armMotor.getCurrentPosition() > 100) {

                if (timeReset == false){
                    time.reset();
                    timeReset = true;
                }

                intake.rightFullyOpen();
                isRightOpen = true;

                if (time.seconds() > 1.25) {
                    intake.leftFullyOpen();
                    isLeftOpen = true;
                    isFullyOpen = true;
                    timeReset = false;
                    break;
                }
            }

        }


        while(true){
            if (timeReset == false){
                time.reset();
                timeReset = true;
            }

            if (time.seconds() > 1.25) {

                lift.goToStartHeight(); // raise lift so arm is ready for blocks coming in from intake

                arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.armMotor.setPower(0.08); // arm goes to place where the 0 position will be
                break;
            }
        }

        waitForStart();

            if(!armReady) {
                    arm.resetEncoder();
                    arm.setHand("open");
                    armReady = true;
            }

        while (!isStopRequested()){
            time.reset();

            lift.setPID();

            //////////////////////////////////// gamepad1   //////////////////////////////////////////

            if(gamepad1.y) {

                if(!is1YPressed) {
                    is1YPressed = true;

                    if (slowDrive == false) {
                        slowDrive = true;
                    }

                    else if (slowDrive == true){
                        slowDrive = false;
                    }
                }
            }

            else{
                is1YPressed = false;
            }


            if(slowDrive == false){

                Pose2d v = transform.transform(new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
                drive.setPower(v);
            }

            if(slowDrive == true){

                Pose2d v = new Pose2d(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
                drive.setSlowPower(v);
            }


            //drive.setPower(new Vector2d(gamepad1.left_stick_y, -gamepad1.left_stick_x), gamepad1.right_stick_x);

            if (gamepad1.a){
                foundationMover.moveToGrab();
            }

            if (gamepad1.b){
                foundationMover.moveToStore();
            }

            if (gamepad1.left_bumper) {

                if (isLeftBumperPressed == false) {

                    isLeftBumperPressed = true;

                    if (isRightOpen == false) {

                        intake.rightOpen();
                        isRightOpen = true;

                    } else {

                        intake.rightClose();
                        isRightOpen = false;
                        isFullyOpen = false;
                    }

                }


            } else if (!gamepad1.left_bumper && !isFullyOpen && isLeftBumperPressed) {

                isLeftBumperPressed = false;

                if (isRightOpen == true){

                    intake.leftOpen();
                    isLeftOpen = true;

                } else {
                    intake.leftClose();
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

                    } else {

                        isFullyOpen = false;

                        intake.rightOpen();
                        intake.leftOpen();
                    }
                }


            } else {

                isRightBumperPressed = false;
            }




            if (gamepad1.left_trigger > 0) {

                intake.setIntakePower(-1);


            } else {

                intake.setIntakePower(0);
            }

            if (gamepad1.right_trigger > 0){

                intake.setIntakePower(1);

            } else {

                intake.setIntakePower(0);
            }


            /////////////////////////////////////// gamepad2   /////////////////////////////////////////////


            ////////////////////// Main Lift Code ///////////////////////
            if (gamepad2.dpad_up) {
                if (isDpadUp == false) {

                    isDpadUp = true;

                    isDpadDown = false;

                    blocks += 1;
                    //lift.runTo(blocks, lift.liftPower, liftEncoder.Mode.BLOCKS);
                    lift.runToBlocks(blocks, lift.liftPower);
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
                    lift.runToBlocks(blocks, lift.liftPower);
                }
            }

            else{
                isDpadDown = false;
            }


            ////////////////////// Lift Error Prevention /////////////////////////


            if (gamepad2.right_trigger > 0){
                if (!isRightTriggerPressed){
                    isRightTriggerPressed = true;
                    lift.runToIncrement(150);
                }
            }

            else{
                isRightTriggerPressed = false;
            }


            if (gamepad2.left_trigger > 0){
                if (!isLeftTriggerPressed){
                    isLeftTriggerPressed = true;
                    lift.runToIncrement(-150);
                }
            }

            else{
                isLeftTriggerPressed = false;
            }


            /////////////////////// Lift Grab Capstone //////////////////////////

            if (gamepad2.back){
                arm.runTo(8);
                lift.runTo(1340, 1);
            }

            ///////////////////////////////////////////////////////////////



            //////////////////////// ARM //////////////////////////


            if (gamepad2.a){
                //starting height, arm at rest (at hard stop)

                //hand will grab block

                lift.runTo(liftEncoder.startHeight, lift.liftPower);

                arm.runTo(8);

                arm.armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                arm.armMotor.setPower(0.08);
            }


            if (gamepad2.x){
                // lift goes to bottom, arm moves to a position where it is greater than the foundation (2 in) and 1 in above
                // the foundation so it can block a block

                // allows robot to go under bridge

                int blockLifted = foundation;

                arm.runTo(blockLifted);
                lift.runTo(liftEncoder.bottomPosition, lift.liftPower); // using bottom position instead of 0
            }


            if (gamepad2.b){
                // arm moves down 1 in. to place block on foundation

                // hand releases block

                int blockPlaced = lower;

                arm.runTo(blockPlaced);
            }

            if (gamepad2.y){
                if (!isYPressed) {
                    isYPressed = true;

                    if (extraBlocks == 0) {
                        arm.runTo(oneExtraBlock);
                        extraBlocks = 1;
                    }

                    else if (extraBlocks == 1){
                        arm.runTo(twoExtraBlock);
                        extraBlocks = 0;
                    }
                }
            }

            else{
                isYPressed = false;
            }


            ////////////////////////////// HAND ////////////////////////////


            if (gamepad2.right_bumper){
                arm.setHand("open");
            }

            if (gamepad2.left_bumper){
                arm.setHand("close");
            }


            ////////////////////////// Telemetry //////////////////////////////

            telemetry.addData("blocks", blocks);

            telemetry.addData("isBusy", arm.armMotor.isBusy());
            telemetry.addData("current pos", arm.armMotor.getCurrentPosition());

            telemetry.addData("1/2 speed drive", slowDrive);

            telemetry.update();


        }
    }
}
