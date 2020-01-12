package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Drive {

    public static double MAX_V = 30;
    public static double MAX_O = 30;
  
    public static final double RADIUS = 2;

    public static double MOVE_FORWARD = -0.8;
    public static double MOVE_BACK = 0.8;
    public static double MOVE_RIGHT = 0.8;
    public static double MOVE_LEFT = -0.8;
    public static double omegaSpeed = 0.8;

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    private static double WHEEL_FROM_CENTER = 0; /////////////////find length of wheel from center

    private static final double TICK_COUNT = 0;
    private static final double WHEEL_DIAMETER = 2; ////////real diameter is 4
    private static final double TICKS_PER_INCH = TICK_COUNT/ WHEEL_DIAMETER * Math.PI;

    public double wheelOmega = 0;
    public int MDistance = 0;

    private ElapsedTime runtime = new ElapsedTime();
  
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    public static Vector2d[] WHEEL_POSITIONS = {
            new Vector2d(6, 7.5),
            new Vector2d(-6, 7.5),
            new Vector2d(-6, -7.5),
            new Vector2d(6, -7.5)
    };

    public static Vector2d[] ROTOR_DIRECTIONS = {
      new Vector2d(1, 1),
      new Vector2d(-1, 1),
      new Vector2d(-1, -1),
      new Vector2d(1, -1)
    };

    public DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu;

    private boolean turning = false;
    private double targetHeading;
    private double headingOffset;
    private double rawHeading;


    public Drive(HardwareMap hardwareMap){
        //super("drive");

       // motors[0] = robot.getMotor("m0");
      // motors[1] = robot.getMotor("m1");
      // motors[2] = robot.getMotor("m2");
      // motors[3] = robot.getMotor( "m3");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
        motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
        motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
        motors[3] = hardwareMap.get(DcMotorEx.class, "m3");


        motors[0].setDirection(DcMotorEx.Direction.FORWARD);
        motors[1].setDirection(DcMotorEx.Direction.REVERSE);
        motors[2].setDirection(DcMotorEx.Direction.FORWARD);
        motors[3].setDirection(DcMotorEx.Direction.REVERSE);

        for(int i=0; i<4; i++){
            motors[i].setPower(0);
        }

        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

       //imu = robot.getRevHubImu(0);
      // BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
       //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
      // parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

       //imu.initialize(parameters);
    }


    //what is this for?
    public void moveTo(int seconds){
        double x = 0;
        double y = 0;
        runtime.reset();

        Vector2d v = new Vector2d(y,x);

        if (runtime.seconds() > seconds){
            //setPower(v, 0);

    }

    }

    /*

    public void setPower(Vector2d v, double omega) {

        setVelocity(v.times(MAX_V), omega * MAX_O);

    }


    public void setVelocity(Vector2d v, double omega) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.getX() - omega * WHEEL_POSITIONS[i].getY(),
                    v.getY() + omega * WHEEL_POSITIONS[i].getX());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);

        }

    }

     */


    public void setPower(Pose2d target) {
        double v = target.vec().norm() * MAX_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * MAX_O;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        setVelocity(targetVelocity);

    }

    public void setVelocity(Pose2d v) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.getX() - v.getHeading() * WHEEL_POSITIONS[i].getY(),
                    v.getY() + v.getHeading() * WHEEL_POSITIONS[i].getX());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);


        }

    }


    /////////////////// Auto specific methods //////////////////////////////////////////////////////

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

//////////////////////// Auto specific methods end//////////////////////////////////////////////////


    //TODO robomatic probably needs the update function ha ha ha

    public void update(){

        rawHeading = imu.getAngularOrientation().firstAngle;
    }


}