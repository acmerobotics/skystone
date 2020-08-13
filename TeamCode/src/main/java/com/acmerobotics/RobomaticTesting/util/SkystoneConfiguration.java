package com.acmerobotics.RobomaticTesting.util;

import com.acmerobotics.robomatic.config.IntegerConfiguration;
import com.acmerobotics.robomatic.config.OpmodeConfiguration;


public class SkystoneConfiguration {

    public enum AllianceColor {
        RED,
        BLUE
    }

    public AllianceColor allianceColor;


    public enum AutoType {
        FOUNDATION,
        SKYSTONE,
        PARKING

    }

    public AutoType autoType;

    public enum StartLocation {
        BUILDING_ZONE,
        LOADING_ZONE
    }

    public StartLocation startLocation;

    public enum ParkingLocation {
        BRIDGE,
        WALL
    }

    public ParkingLocation parkingLocation;

    public boolean collectBothSkystones;

    public boolean hasInitializationBeenRun;

   @IntegerConfiguration(max=10)
    public int delay;

}
