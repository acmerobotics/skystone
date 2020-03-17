package com.acmerobotics.RobomaticTesting.roboOpModes;

import com.acmerobotics.RobomaticTesting.roboConfig;
import com.acmerobotics.RobomaticTesting.roboRobot;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="roboAuto")
public class roboAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        roboRobot robot = new roboRobot(this);

        roboConfig config = (roboConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        robot.addTelemetry("color", config.color);
        robot.addTelemetry("delay", config.delay);
        robot.addTelemetry("starting location", config.startLocation);

        robot.runUntil(this::opModeIsActive); // will run above until start is pressed, basically a waitForStart()

        if (config.color == roboConfig.AllianceColor.RED){
            if (config.startLocation == roboConfig.StartLocation.LOADING){
                telemetry.addLine("run: a red loading zone auto ");
            }

            else if (config.startLocation == roboConfig.StartLocation.BUILDING){
                telemetry.addLine("run: a red building zone auto ");
            }

        }

        else if (config.color == roboConfig.AllianceColor.BLUE){
            if (config.startLocation == roboConfig.StartLocation.LOADING){
                telemetry.addLine("run: a blue loading zone auto ");
            }

            else if (config.startLocation == roboConfig.StartLocation.BUILDING){
                telemetry.addLine("run: a blue building zone auto ");
            }

        }

    }
}
