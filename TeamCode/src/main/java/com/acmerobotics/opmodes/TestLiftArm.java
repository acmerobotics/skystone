package com.acmerobotics.opmodes;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.robot.Lift;
import com.acmerobotics.robot.PlacingArm;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="TestArm/Lift")
public class TestLiftArm extends LinearOpMode {

    public double liftPotentialValue = 0;
    TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException{

        PlacingArm arm = new PlacingArm(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        while (!isStopRequested()){

            if (gamepad1.y){
                arm.armRelocationPosition();
            }

            if (gamepad1.a){ // could possible combine with right bumper
                arm.armIntakePosition();
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
                lift.update(packet);
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

            if(gamepad2.left_bumper){
                //release block, servo stuff
                arm.setServo("open");
            }

            if (gamepad2.right_bumper){
                // grab block, servo stuff
                arm.setServo("close");
            }

            telemetry.addData("Block count: ", liftPotentialValue);
            telemetry.update();

        }


    }



}
