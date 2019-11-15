package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.util.Vector2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Drive {

    //TODO figure what the heck is happening with the drive like why won't you do the things i need you to do?

    public static double AUTO_SPEED = 10;

    public static double d = 0; //distance
    public static double r = 0; //rate
    public static double t = 0; //time

    public static double MAX_V = 30;
    public static double MAX_O = 1;
    public static final double RADIUS = 2;

    public static Vector2d[] WHEEL_POSITIONS = {
            new Vector2d(6, 7.5),
            new Vector2d(-6, 7.5),
            new Vector2d(-6, -7.5),
            new Vector2d(6, -7.5)
    };

    public static Vector2d[] ROTOR_DIRECTIONS = {
      new Vector2d(1, 1).unit(),
      new Vector2d(-1, 1).unit(),
      new Vector2d(-1, -1).unit(),
      new Vector2d(1, -1).unit()
    };

    private DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu;

    private boolean turning = false;
    private double targetHeading;
    private double headingOffset;
    private double rawHeading;

    private Telemetry telemetry;

    public Drive(HardwareMap hardwareMap){
        //super("drive");

      /* motors[0] = robot.getMotor("m0");
       motors[1] = robot.getMotor("m1");
       motors[2] = robot.getMotor("m2");
       motors[3] = robot.getMotor( "m3");*/


        motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
        motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
        motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
        motors[3] = hardwareMap.get(DcMotorEx.class, "m3");

       FtcDashboard dashboard = FtcDashboard.getInstance();


       for (int i = 0; i < 4; i++){
           motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
           motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
       }

       motors[0].setDirection(DcMotorEx.Direction.FORWARD);
       motors[1].setDirection(DcMotorEx.Direction.REVERSE);
       motors[2].setDirection(DcMotorEx.Direction.FORWARD);
       motors[3].setDirection(DcMotorEx.Direction.REVERSE);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

      /* imu = robot.getRevHubImu(0);
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
       imu.initialize(parameters); */

    }

    public void setPower(Vector2d v, double omega) {

        setVelocity(v.times(MAX_V), omega * MAX_O);

    }

    // is the math wrong ?????

    public void setVelocity(Vector2d v, double omega) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.x() - omega * WHEEL_POSITIONS[i].y(),
                    v.y() + omega * WHEEL_POSITIONS[i].x());
            double wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / RADIUS;
            motors[i].setPower(wheelOmega);

        }

    }

    // ha ha this is all for auto, something that doesn't exist yet....

    //TODO make auto happens

    public double getRawHeading(){
        return rawHeading;

    }

    public double getHeading(){
        return getRawHeading() + headingOffset;
    }

    public void setHeading(double heading){
        headingOffset = heading - getRawHeading();
    }

    public void turn(double angle){
        turning = true;
        targetHeading = getHeading() + angle;

    }

//    public double getRate(double distance, double time){
//
//    }
//
//    public double getTime(){
//
//    }

    //TODO robomatic probably needs the update function ha ha ha

    public void update(){

        rawHeading = imu.getAngularOrientation().firstAngle;
    }


}