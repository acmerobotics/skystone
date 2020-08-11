package com.acmerobotics.RobomaticTesting.util;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.robomatic.config.ConfigurationLoader;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class ConfigTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        SkystoneConfiguration configuration = (SkystoneConfiguration) new ConfigurationLoader(hardwareMap.appContext).getConfig();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("alliance color", configuration.allianceColor);
        packet.put("start location", configuration.startLocation);
        packet.put("parking location", configuration.parkingLocation);
        packet.put("delay", configuration.delay);
        packet.put("get both skystones", configuration.collectBothSkystones);
        packet.put("has initialation run", configuration.hasInitializationBeenRun);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        waitForStart();

    }
}
