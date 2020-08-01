package com.acmerobotics.RobomaticTesting.util;

import com.acmerobotics.robomatic.config.IntegerConfiguration;
import com.acmerobotics.robomatic.config.OpmodeConfiguration;


public class SkystoneConfiguration {

    public enum AllianceColor {
        RED,
        BLUE
    }

    public AllianceColor allianceColor;


    public enum StartLocation {
        FOUNDATION,
        SKYSTONE,
        PARKING

    }

    public StartLocation startLocation;

    public enum ParkingLocation {
        BRIDGE,
        WALL
    }

    public ParkingLocation parkingLocation;

    public boolean bothSkystones;

   @IntegerConfiguration(max=10)
    public int delay;

}
