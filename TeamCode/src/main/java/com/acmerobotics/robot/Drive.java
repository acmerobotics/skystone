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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
public class Drive {

    private static double MAX_V = 30;
    private static double MAX_O = 30;

    public static double slow_v = MAX_V/2;
    public static double slow_o = MAX_O/2;

    public double strafePower = 0.5;

    public double turnPower = 0.5;
  
    private static final double WHEEL_RADIUS = 2;


    private int targetMotorPos;
    private boolean atTargetMotorPos = false;

    private boolean motorsStopped = false;

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    Orientation lastAngle = new Orientation();

    private double wheelOmega = 0;
    private double degrees;
    private double globalAngle;
    private double grabPosition = 0.70;
    private double releasePosition = 0.20;
    private double ticksPerRev = 560.0;

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

    public DcMotorEx omniTracker;

    private static final double trackerRadius = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double trackerTicksPerInch = (500 * 4) / (2 * trackerRadius * Math.PI);

    public double targetOmniPos;
    private boolean atTargetOmniPos = false;
    private int zeroPos;

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
            omniTracker = hardwareMap.get(DcMotorEx.class, "rightMotor");
            omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motors[0].setDirection(DcMotorEx.Direction.FORWARD);
            motors[1].setDirection(DcMotorEx.Direction.FORWARD);
            motors[2].setDirection(DcMotorEx.Direction.REVERSE);
            motors[3].setDirection(DcMotorEx.Direction.REVERSE);

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
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / WHEEL_RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);


        }

    }



    /////////////////// Auto specific methods //////////////////////////////////////////////////////
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
        motors[0].setPower(turnPower);
        motors[1].setPower(turnPower);
        motors[2].setPower(-turnPower);
        motors[3].setPower(-turnPower);

    }

    public void clockwise(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(-turnPower);
        motors[1].setPower(-turnPower);
        motors[2].setPower(turnPower);
        motors[3].setPower(turnPower);
    }

    public double getCurrentTrackerPosInches(){
        return omniEncoderTicksToInches(omniTracker.getCurrentPosition());
    }

    public int getCurrentTrackerPosTicks(){
        return omniTracker.getCurrentPosition();
    }

    public void resetEncoderOmni(){
     omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void goToStrafingPos(double distance, double power, String direction){
        setTrackingOmni(power, direction);

        targetOmniPos = distance;
    }

    private void setTrackingOmni(double power, String direction){
        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction.equals("left")){
            motors[0].setPower(-power);
            motors[1].setPower(power);
            motors[2].setPower(-power);
            motors[3].setPower(power);

        } else if (direction.equals("right")) {
            motors[0].setPower(power);
            motors[1].setPower(-power);
            motors[2].setPower(power);
            motors[3].setPower(-power);
        }

    }

    private int omniEncodersInchesToTicks(int inches) {
        double circumference = 2 * Math.PI * trackerRadius;
        return (int) Math.round(inches * (500 * 4) / circumference);
    }

    public double omniEncoderTicksToInches(int ticks){
        double revs = ticks / trackerTicksPerInch;
        return 2 * Math.PI * trackerRadius * revs;
    }

    public boolean atStrafingPos(){

        return Math.abs(getCurrentTrackerPosInches()) > Math.abs(targetOmniPos);

    }

    public boolean resetStrafingPos(){
        atTargetOmniPos = false;

        return atTargetOmniPos;
    }

    public double getTargetOmniPos(){
        return targetOmniPos;
    }

    public void moveForward(double power){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        motors[0].setPower(power);
        motors[1].setPower(power);
        motors[2].setPower(power);
        motors[3].setPower(power);
    }

    public void moveBack(double power){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        motors[0].setPower(-power);
        motors[1].setPower(-power);
        motors[2].setPower(-power);
        motors[3].setPower(-power);
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

    private void setMotorEncoders(int distance, double power){
        motors[0].setTargetPosition(distance);
        motors[1].setTargetPosition(distance);
        motors[2].setTargetPosition(distance);
        motors[3].setTargetPosition(distance);

        motors[0].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[1].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[2].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[3].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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
        return 2 * Math.PI * WHEEL_RADIUS * revs;

    }

    private int motorEncodersInchesToTicks(int inches) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    public void goToPosition(int position, double power){
        setMotorEncoders(motorEncodersInchesToTicks(position), power);

        targetMotorPos = motorEncodersInchesToTicks(position);

    }


    public void grab() {

        stoneServo.setPosition(grabPosition);
    }

    public void release() {

        stoneServo.setPosition(releasePosition);
    }


    public int getCurrentPos(){
        int motorZeroPos = motors[0].getCurrentPosition();
        int motorOnePos = motors[1].getCurrentPosition();
        int motorTwoPos = motors[2].getCurrentPosition();
        int motorThreePos = motors[3].getCurrentPosition();
        return (motorZeroPos + motorOnePos + motorTwoPos + motorThreePos) / 4;
    }

    public int getTargetMotorPos(){
        return targetMotorPos;
    }

    public double getCurrentAngle(){
        return globalAngle;
    }

    public boolean atLinearPos(){

        return Math.abs(targetMotorPos - getCurrentPos()) < 14;

    }
    ////////////////////////////////////////////////////////

    public double realTicksPerInch(int ticks){
        double D = 1.4;
        int ticksPerRev = 2000;

        return (ticks * ( (D * 3.14) / (ticksPerRev) ) );
    }


    public void IsetTrackingOmni(double power, String direction){
        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction.equals("right")){
            motors[0].setPower(-power);
            motors[1].setPower(power);
            motors[2].setPower(-power);
            motors[3].setPower(power);

        } else if (direction.equals("left")) {
            motors[0].setPower(power);
            motors[1].setPower(-power);
            motors[2].setPower(power);
            motors[3].setPower(-power);
        }
    }


    public void IgoToStrafingPos(double distance, double power, String direction){

        IsetTrackingOmni(power, direction);

        targetOmniPos = distance;
    }


    public boolean IatStrafingPos(){
        return Math.abs(realTicksPerInch(omniTracker.getCurrentPosition())) > Math.abs(targetOmniPos);
    }





//////////////////////// Auto specific methods end//////////////////////////////////////////////////

    public void update(){

        double rawHeading = imu.getAngularOrientation().firstAngle;
    }


}