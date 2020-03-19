package com.acmerobotics.RobomaticTesting.IndividualTests;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;

public class ImuTest extends Subsystem {

    private double lastAngle;
    private double globalAngle;
    private CachingSensor imuSensor;

    public ImuTest(Robot robot){
        super("imuTest");

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z)); // creates BN0055-IMU-Impl, imu orientation is remapped
        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle); // gets heading
        robot.registerCachingSensor(imuSensor); // adds imu to caching sensors that will then update the heading
    }

    @Override
    public void update(Canvas overlay) {

        telemetryData.addData("heading (radians)", imuSensor.getValue());
        telemetryData.addData("heading (degrees)", getAngle());
    }


    private double convertToDegrees(double radians){
        return (180 * radians) / 3.14;
    }


    public void resetAngle(){
        // imuSensor.getValue returns an obj when it should return a float to get around this I convert the obj to a string then to a double
        // the imuSensor.getValue returns the heading in radians, setting the return value to degrees is hard unless the hard code is changed or I manually do the convergence
        // I decided to manually change the radians to degrees

        String stringValue = String.valueOf(imuSensor.getValue()); // obj to string
        double radians = Double.valueOf(stringValue); // string to double
        lastAngle = convertToDegrees(radians); // radians to degrees

        globalAngle = 0;

    }

    public double getAngle(){

        // imuSensor.getValue returns an obj when it should return a float to get around this I convert the obj to a string then to a double
        // the imuSensor.getValue returns the heading in radians, setting the return value to degrees is hard unless the hard code is changed or I manually do the convergence
        // I decided to manually change the radians to degrees

        String stringValue = String.valueOf(imuSensor.getValue()); // obj to string
        double radians = Double.valueOf(stringValue); // string to double
        double angles = convertToDegrees(radians); // radians to degrees

        double deltaAngle = angles - lastAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angles;

        return globalAngle;

    }
}
