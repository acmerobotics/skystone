package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.acmerobotics.robot.armEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SkystoneGrabTest")
@Config

public class SkystoneGrabTest extends LinearOpMode{

    private boolean inStone1Position = false;

    private int ticksTraveled1 = 0;
    private int ticksTraveled2 = 0;

    private boolean stone1Detected = false;
    private boolean stone2Detected = false;

    private int stonesMoved = 0;

    private int timeToPark = 8; // minimum time needed to park
    private int passedBlocks; // amount of ticks equal to the length of block, checks if drive passed the blocks

    private String state = "atParking"; // "grabFirst"  "returnFirst" "grabSecond" "returnSecond" "grabAny"

        @Override
        public void runOpMode() throws InterruptedException {
            Drive drive = new Drive(hardwareMap, false);
            drive.Pcoefficient = 0.2; // 0.1 is default

            drive.resetEncoders();

            waitForStart();

            while(!isStopRequested()){

                drive.LgoToPosition("forward",-48, 0.28);

                // drive.IgoToPosition("right", -48, 0.28);

                telemetry.addData( "current position", drive.motors[0].getCurrentPosition());
                telemetry.addData("target position", drive.motors[0].getTargetPosition());

                telemetry.addLine();

                telemetry.addData("angle", drive.getAngle());

                telemetry.update();

            }

        }
}
