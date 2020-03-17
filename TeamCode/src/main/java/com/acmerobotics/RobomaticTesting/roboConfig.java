package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.robomatic.config.IntegerConfiguration;
import com.acmerobotics.robomatic.config.OpmodeConfiguration;
import com.acmerobotics.robomatic.demo.DemoConfig;

@OpmodeConfiguration
public class roboConfig {

    public enum AllianceColor {
        RED,
        BLUE
    }

    // init AllianceColor
    public AllianceColor color;

    public enum StartLocation {
        LOADING,
        BUILDING
    }

    // init StartingLocation
    public StartLocation startLocation;

    @IntegerConfiguration(max=10)
    public int delay;
}
