package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.util.Vector2d;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Drive {

    //TODO figure what the heck is happening with the drive like why won't you do the things i need you to do?



    public static double MAX_V = 30;
    public static double MAX_O = 1;
    public static final double RADIUS = 2; // wheel radius ???????????????????????????????????????????????

    public static double MOVE_FORWARD = -0.8;
    public static double MOVE_BACK = 0.8;
    public static double MOVE_RIGHT = 0.8;
    public static double MOVE_LEFT = -0.8;
    public static double omegaSpeed = 0.8;

    private static double WHEEL_FROM_CENTER = 0; /////////////////find length of wheel from center

    private static final double TICK_COUNT = 0;
    private static final double WHEEL_DIAMETER = 2; //find real wheel diameter
    private static final double TICKS_PER_INCH = TICK_COUNT/ WHEEL_DIAMETER * Math.PI; //figure out if drive gear reduction is needed

    public double wheelOmega = 0;
    public int MDistance = 0;

    private ElapsedTime     runtime = new ElapsedTime();


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

    public DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu;

    private boolean turning = false;
    private double targetHeading;
    private double headingOffset;
    private double rawHeading;


    public Drive(HardwareMap hardwareMap){
        //super("drive");

      /* motors[0] = robot.getMotor("m0");
       motors[1] = robot.getMotor("m1");
       motors[2] = robot.getMotor("m2");
       motors[3] = robot.getMotor( "m3");*/

       FtcDashboard dashboard = FtcDashboard.getInstance();

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

      /* imu = robot.getRevHubImu(0);
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
       imu.initialize(parameters); */

    }

    public void moveTo(int seconds){
        double x = 0;
        double y = 0;
        runtime.reset();

        Vector2d v = new Vector2d(y,x);

        if (runtime.seconds() > seconds){
            setPower(v, 0);

    }


    public void setPower(Vector2d v, double omega) {

        setVelocity(v.times(MAX_V), omega * MAX_O);

    }

    // is the math wrong ?????

    public void setVelocity(Vector2d v, double omega) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.x() - omega * WHEEL_POSITIONS[i].y(), v.y() + omega * WHEEL_POSITIONS[i].x());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / RADIUS;
            motors[i].setPower(wheelOmega);// divide wheelOmega by 30? to get number between -1 and 1

        }

    }


    /////////////////// Auto specific methods //////////////////////////////////////////////////////

    public void stopAndReset(){
        for(int i=0; i<4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setPowerA(Vector2d v, double omega){
        setVelocityA(v.times(MAX_V), omega * MAX_O);
    }

    public void setVelocityA(Vector2d v, double omega){

        for (int i = 0; i < 4; i ++){
            Vector2d wheelVelocity = new Vector2d(v.x() - omega * WHEEL_POSITIONS[i].y(), v.y() + omega * WHEEL_POSITIONS[i].x());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)/RADIUS);
            motors[i].setPower(Math.abs(wheelOmega));
        }
    }

    public void setMDistance(Vector2d v, double distance) { //right now only does forward and back but after testing it add right and left
        //sets 1)direction 2)distance with direction 3)target position

        for(int i=0; i<4; i++) {
            double setDirections = v.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2);

            distance *= setDirections;

            MDistance = motors[i].getCurrentPosition() + (int) (distance * TICKS_PER_INCH);
            motors[i].setTargetPosition(MDistance);
        }

        for(int i=0; i<4; i++) {
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void stopMotors(){
        for (int i = 0; i < 4; i++) {
            if (!(motors[i].isBusy())) {

                //sets everything to zero because destination was reached
                setPowerA(new Vector2d(0, 0), 0);
                motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void moveRobotTo(String move, double distance){// distance is negative if going back

        double y = 0;
        double x = 0;

        if (move.equals("forward")) {
            y = 1;

            Vector2d v = new Vector2d(y,x); //sets vector based on move

            setMDistance(v, distance);

            setPowerA(v, 0);

            stopMotors();
        }

        if (move.equals("back")) {
            y = -1;

            Vector2d v = new Vector2d(y,x); //sets vector based on move

            setMDistance(v, distance);

            setPowerA(v, 0);

            stopMotors();
        }

        if (move.equals("right")) {
            x = 1;

            Vector2d v = new Vector2d(y,x); //sets vector based on move

            setMDistance(v, distance);

            setPowerA(v, 0);

            stopMotors();
        }

        if (move.equals("left")) {
            x = -1;

            Vector2d v = new Vector2d(y,x); //sets vector based on move

            setMDistance(v, distance);

            setPowerA(v, 0);

            stopMotors();
        }
    }

    public void turnRobotTo(String move, double angle ){
        //turns robot
        Vector2d v = new Vector2d(0,0); //sets vector based on move

        double omega = 0;
        double distance = getRadianLen(angle, WHEEL_FROM_CENTER); //find wheel's distance from center so it acts as radius

        if (move.equals("right")) {

            omega = 1 * omegaSpeed;

            setMDistance(v, distance);

            setPowerA(v, omega);

            stopMotors();
        }

        if (move.equals("left")) {

            omega = -1 * omegaSpeed;

            setMDistance(v, distance);

            setPowerA(v, omega);

            stopMotors();
        }
    }


    public double getRadianLen(double angle, double radius){
        // returns radian length (wheel movement curve length)

        double i =  (angle/360) * 2 * Math.PI * radius;
        return i;
    }

//    public double getRawHeading(){
//        return rawHeading;
//
//    }
//
//    public double getHeading(){
//        return getRawHeading() + headingOffset;
//    }
//
//    public void setHeading(double heading){
//        headingOffset = heading - getRawHeading();
//    }
//
//    public void turn(double angle){
//        turning = true;
//        targetHeading = getHeading() + angle;
//
//    }

//////////////////////// Auto specific methods end//////////////////////////////////////////////////


    //TODO robomatic probably needs the update function ha ha ha

    public void update(){

        rawHeading = imu.getAngularOrientation().firstAngle;
    }


}