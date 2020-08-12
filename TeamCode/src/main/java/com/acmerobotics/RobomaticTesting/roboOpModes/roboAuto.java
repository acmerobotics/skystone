package com.acmerobotics.RobomaticTesting.roboOpModes;

import android.util.Log;

import com.acmerobotics.RobomaticTesting.roboConfig;
import com.acmerobotics.RobomaticTesting.roboRobot;
import com.acmerobotics.opmodes.AutoOpModes.BlueBridgeParkingBuildingZone;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.util.AutoTransitioner;

@Autonomous(name="roboAuto")
public class roboAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        roboRobot robot = new roboRobot(this);

        // autos
        BlueBridgeParkingBuildingZone blueBridgeParkingBuildingZone = new BlueBridgeParkingBuildingZone();

        roboConfig config = (roboConfig) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        robot.addTelemetry("color", config.color);
        robot.addTelemetry("delay", config.delay);
        robot.addTelemetry("starting location", config.startLocation);

        // automatic auto to teleOp transistor when auto is stoped
        AutoTransitioner.transitionOnStop(this, "roboTeleOp");

        waitForStart();

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
                try {
                    blueBridgeParkingBuildingZone.runOpMode(); // if done this way I have to deal with or remove InterruptedException
                }
                catch (Exception InterruptedException){
                    Log.e("roboAuto", "thread was interrupted in auto");
                }
            }

        }


        robot.update();

    }
}
