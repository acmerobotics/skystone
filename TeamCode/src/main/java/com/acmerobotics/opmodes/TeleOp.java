package com.acmerobotics.opmodes;

import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.Lift;
import com.acmerobotics.robot.PlacingArm;
import com.acmerobotics.util.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {

    public double x = 0;
    Telemetry telemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        //SkyStoneRobot robot = new SkyStoneRobot(this);
        Drive drive = new Drive(hardwareMap);
        Lift lift = new Lift(hardwareMap);


        while (!isStopRequested()){

            drive.setPower(new Vector2d(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);

            if (gamepad2.y){
                x += 1;
            }

            if (gamepad2.a){
                x -= 1;
            }

            if (gamepad2.x){
                lift.setLiftIncrement(x);

            }

            telemetry.addData("Placing Height", x);
        }

    }

}
