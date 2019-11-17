package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.Lift;
import com.acmerobotics.robot.PlacingArm;
import com.acmerobotics.util.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    public double liftPotentialValue;

    Telemetry telemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        PlacingArm arm = new PlacingArm(hardwareMap);


        while (!isStopRequested()){

            ////////gamepad1   ////////////////////
            drive.setPower(new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);

            //TODO check power that is going to motors (is it between -1 and 1 or -30 and 30)
            telemetry.addData("0", drive.motors[0].getPower());
            telemetry.addData("m1", drive.motors[1].getPower());
            telemetry.addData("m2", drive.motors[2].getPower());
            telemetry.addData("m3", drive.motors[3].getPower());

            if (gamepad1.y){
                arm.armRelocationPosition();
            }

            if (gamepad1.a){ // could possible combine with right bumper
                arm.armIntakePosition();
            }

            if (gamepad1.right_bumper){
                /// servo grab block
                arm.setServo("open");
            }


            ////////gamepad2   ///////////////
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
                arm.armInitPosition();
            }

            if (gamepad2.a){
                arm.armIntakePosition();// could possible combine with right bumper
            }

            if (gamepad2.right_bumper){
                // grab block, servo stuff
                arm.setServo("open");
            }

            telemetry.addData("Block count ", liftPotentialValue);
        }

    }

}
