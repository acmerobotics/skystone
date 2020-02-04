package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Drive {

    public static double MAX_V = 30;
    public static double MAX_O = 30;

    public static double slow_v = MAX_V/2;
    public static double slow_o = MAX_O/2;

    public double moveForwardPower = 0.5;
    public double moveBackPower = -0.5;
  
    public static final double RADIUS = 2;

    public int targetPos;

    public boolean atTargetPos = false;

    public boolean motorsStopped = false;

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    Orientation lastAngle = new Orientation();

    public double wheelOmega = 0;
    public int MDistance = 0;
    private double currentPos;
    private double error;
    private double degrees;
    private double globalAngle;

    private double grabPosition;
    private double releasePosition;

    private double ticksPerRev = 560.0;

    private ElapsedTime runtime = new ElapsedTime();
  
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    private static Vector2d[] WHEEL_POSITIONS = {
            new Vector2d(6, 7.5),
            new Vector2d(-6, 7.5),
            new Vector2d(-6, -7.5),
            new Vector2d(6, -7.5)
    };

    private static Vector2d[] ROTOR_DIRECTIONS = {
      new Vector2d(1, 1),
      new Vector2d(-1, 1),
      new Vector2d(-1, -1),
      new Vector2d(1, -1)
    };

    public DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu;
    private Servo stoneServo;

    private boolean turning = false;
    private double targetHeading;
    private double headingOffset;
    private double rawHeading;


    public Drive(HardwareMap hardwareMap, boolean inTeleOp){
        //super("drive");

       // motors[0] = robot.getMotor("m0");
      // motors[1] = robot.getMotor("m1");
      // motors[2] = robot.getMotor("m2");
      // motors[3] = robot.getMotor( "m3");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        stoneServo = hardwareMap.get(Servo.class, "stoneServo");

        motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
        motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
        motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
        motors[3] = hardwareMap.get(DcMotorEx.class, "m3");


        if(!inTeleOp){
            motors[0].setDirection(DcMotorEx.Direction.REVERSE);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.FORWARD);

        } else {

            motors[0].setDirection(DcMotorEx.Direction.FORWARD);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.REVERSE);

        }


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


    public void setSlowPower(Pose2d target) {
        double v = target.vec().norm() * slow_v;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * slow_o;

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

    public double degreesToRadians(double degrees){
        double toRadians = degrees/180 * Math.PI;
        return toRadians;
    }



    /////////////////// Auto specific methods //////////////////////////////////////////////////////

    public double getRawHeading() {
        return rawHeading;

    }

    public void setDegrees(double degrees){
        this.degrees = degrees;
    }

    public double getDegrees(){
        return degrees;
    }

    public void resetAngle(){
       lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

       globalAngle = 0;

    }

    public double getAngle(){

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngle.firstAngle;


        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angles;

        return globalAngle;

    }

    public void counterClockwise(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        motors[0].setPower(0.5);
        motors[1].setPower(0.5);
        motors[2].setPower(-0.5);
        motors[3].setPower(-0.5);

    }

    public void clockwise(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(-0.5);
        motors[1].setPower(-0.5);
        motors[2].setPower(0.5);
        motors[3].setPower(0.5);
    }

    public void strafeLeft(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(-0.5);
        motors[1].setPower(0.5);
        motors[2].setPower(-0.5);
        motors[3].setPower(0.5);
    }

    public void strafeRight(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(0.5);
        motors[1].setPower(-0.5);
        motors[2].setPower(0.5);
        motors[3].setPower(-0.5);
    }

    public void moveForward(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(moveForwardPower);
        motors[1].setPower(moveForwardPower);
        motors[2].setPower(moveForwardPower);
        motors[3].setPower(moveForwardPower);
    }

    public void moveBack(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(moveBackPower);
        motors[1].setPower(moveBackPower);
        motors[2].setPower(moveBackPower);
        motors[3].setPower(moveBackPower);
    }



    public void stopMotors(){

        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

        motorsStopped = true;

    }

    public boolean areMotorsStopped(){
        return motorsStopped;
    }

    public void setEncoders(int distance, double power){
        motors[0].setTargetPosition(distance);
        motors[3].setTargetPosition(distance);
        motors[1].setTargetPosition(distance);
        motors[2].setTargetPosition(distance);

        motors[0].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[3].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[1].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[2].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        for (int i = 0; i < 4; i++){

            motors[i].setPower(power);
        }

    }


    public void resetEncoders(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public double ticksToInches(int ticks) {
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * RADIUS * revs;

    }

    public int inchesToTicks(double inches) {
        double circumference = 2 * Math.PI * RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    public void goToPosition(double position, double power){
        setEncoders(inchesToTicks(position), power);

        targetPos = inchesToTicks(position);

    }


    public void grab(){
        stoneServo.setPosition(grabPosition);
    }

    public void release(){
        stoneServo.setPosition(releasePosition);
    }


    //TODO see if this is causing issues

    public int getCurrentPos(){
        return motors[0].getCurrentPosition();
    }

    public int getTargetPos(){
        return targetPos;
    }

    public double getCurrentAngle(){
        return globalAngle;
    }

    public boolean returnAtTargetPos(){
        return atTargetPos;
    }

    public boolean atLinearPos(){

        if(Math.abs(targetPos - getCurrentPos()) < 4){
            atTargetPos = true;
        }

        return atTargetPos;

    }


    public boolean resetLinearPos(){
        atTargetPos = false;

        return atTargetPos;
    }

//////////////////////// Auto specific methods end//////////////////////////////////////////////////

    public void update(){

        rawHeading = imu.getAngularOrientation().firstAngle;
    }


}