package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.FoundationMover;
import com.acmerobotics.robot.Lift;
import com.acmerobotics.robot.PlacingArm;
import com.acmerobotics.util.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {


    public int liftPotentialValue = 0;
    //Telemetry telemetry;



    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        PlacingArm arm = new PlacingArm(hardwareMap);
        FoundationMover foundationMover = new FoundationMover(hardwareMap);


        while (!isStopRequested()){

            ////////////////////// gamepad1   /////////////////////////////

            drive.setPower(new Vector2d(gamepad1.left_stick_y,- gamepad1.left_stick_x), gamepad1.right_stick_x);

            if (gamepad1.y){
                //arm relocation
                arm.armRelocationPosition();
            }

            if (gamepad1.a){ // could possible combine with right bumper
                //arm is right on top of block
                arm.armIntakePosition();
            }

            if (gamepad1.right_bumper){
                /// servo grab block
                arm.setServo("open");
            }

            if (gamepad1.x){
                foundationMover.moveToGrab();
            }

            if (gamepad1.b){
                foundationMover.moveToStore();
            }


            ///////////////////// gamepad2   /////////////////////////////

            if (gamepad2.dpad_up){
                liftPotentialValue += 1;
            }

            if (gamepad2.dpad_down){
                liftPotentialValue -= 1;
            }

            if (gamepad2.dpad_right){
                // accept potential value and change lift value
                lift.moveTo(liftPotentialValue);
            }

            if (gamepad2.dpad_left){
                // reset height value
                lift.moveTo(0);
            }

            if (gamepad2.y){
                //arm is above block
                arm.armInitPosition();
            }

            if (gamepad2.a){
                //arm is on top block
                arm.armIntakePosition();// could possible combine with right bumper
            }

            if (gamepad2.right_bumper){
                // grab block, servo stuff
                arm.setServo("open");
            }

            //telemetry.addData("Block count ", liftPotentialValue);
        }

    }

}
