package com.acmerobotics.opmodes.AutoOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.Drive;
import com.acmerobotics.robot.TheColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SimpleSkystoneAuto")
@Config
public class SimpleSkystoneAuto extends LinearOpMode {

    private int traveled = 0;
    private boolean stoneDetected = false;


    @Override
    public void runOpMode() throws InterruptedException {
        TheColorSensor colorSensor = new TheColorSensor(hardwareMap);
        Drive drive = new Drive(hardwareMap, false);
        drive.moveBackPower = -0.3;

        waitForStart();

        while (!isStopRequested()) {
            colorSensor.HSV();

            if(!stoneDetected) {
                if (colorSensor.isSkystoneSat()) {

                    drive.stopMotors();
                    traveled = drive.motors[0].getCurrentPosition();
                    stoneDetected = true;

                } else {
                    drive.moveBack();
                    traveled = drive.motors[0].getCurrentPosition();
                }
            }

            if (stoneDetected){

                double inchesTobackUp = -(drive.ticksToInches(traveled));

                drive.goToPosition(inchesTobackUp, 0.5);

                if (drive.atLinearPos()){
                    drive.stopMotors();
                }
            }

            telemetry.addData( "skystone", colorSensor.isSkystoneSat());
            telemetry.addData( "current position 0", drive.motors[0].getCurrentPosition());
            telemetry.addData("current position 1", drive.motors[0].getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("motor 1 power", drive.motors[1].getPower());
            telemetry.addData("have motors been stopped", drive.motorsStopped);
            telemetry.update();
        }
    }
}
