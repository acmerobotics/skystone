package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.acmerobotics.robot.armEncoder;
import com.acmerobotics.robot.liftEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SimpleSkystoneAuto")
@Config
public class SimpleSkystoneAuto extends LinearOpMode {

    private int traveled = 0;
    private boolean stoneDetected = false;
    private boolean liftDown = false;


    @Override
    public void runOpMode() throws InterruptedException {
        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
        Drive drive = new Drive(hardwareMap, false);
        armEncoder arm = new armEncoder(hardwareMap);
        liftEncoder lift = new liftEncoder(hardwareMap);

        drive.resetEncoders();

        drive.moveForwardPower = 0.3;

        waitForStart();

        while (!isStopRequested()) {
            colorSensor.HSV();

            if(!stoneDetected) {
                if (colorSensor.isSkystoneHue()) {

                    drive.stopMotors();
                    traveled = drive.motors[0].getCurrentPosition();
                    stoneDetected = true;

                } else {
                    drive.moveForward();
                    traveled = drive.motors[0].getCurrentPosition();
                }
            }

            if (stoneDetected){

                    drive.goToPosition(0, 0.3);

                    if (lift.bottomSet) {
                        drive.goToPosition(-40, 0.3);

                        if (drive.atLinearPos()) {
                            drive.stopMotors();
                        }
                    } else {
                        arm.runTo(80);

                        lift.tightenLiftString();

                        lift.goToBottom();
                    }
            }

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.update();
        }
    }
}
