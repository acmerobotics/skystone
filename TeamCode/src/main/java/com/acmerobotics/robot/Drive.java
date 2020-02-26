package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    //constants
    private static final double WHEEL_RADIUS = 2;

    private static final double TRACKER_RADIUS = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double TRACKER_TICKS_PER_INCH  = (500 * 4) / (2 * TRACKER_RADIUS * Math.PI);

    // hardware devices
    public DcMotorEx[] motors = new DcMotorEx[4];
    private BNO055IMU imu;
    private Servo stoneServo;
    private DcMotorEx omniTracker;

    // motors/motor encoders
    private static double MAX_V = 30;
    private static double MAX_O = 30;
    private static double SLOW_V = MAX_V/2;
    private static double SLOW_O = MAX_O/2;

    private double ticksPerRev = 560.0;

    private boolean motorsStopped = false;

    private int targetMotorPos;

    // misc. motor powers
    public double moveForwardPower = 0.5;
    public double moveBackPower = 0.5;
    public double strafePower = 0.5;
    public double turnPower = 0.5;


    // omni variables
    private double targetOmniPos;
    private boolean atTargetOmniPos = false;

    // imu variables
    Orientation lastAngle = new Orientation();
    private double degrees;
    private double globalAngle;
    private double grabPosition = 0.75;
    private double releasePosition = 0.20;
    private double ticksPerRev = 560.0;

    // servo variables
    private double grabPosition;
    private double releasePosition;


    // vector/pos variables
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

    private double wheelOmega = 0;

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    //hardware map
    private static final double trackerRadius = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double trackerTicksPerInch = (500 * 4) / (2 * trackerRadius * Math.PI);

    public double targetOmniPos;
    private boolean atTargetOmniPos = false;
    private int zeroPos;

    public double Pcoefficient = 0.1;
    public static double PcoefficientTurn = 0.04;

    public double error;
    public double newPower;

    public Drive(HardwareMap hardwareMap, boolean inTeleOp){

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        stoneServo = hardwareMap.get(Servo.class, "stoneServo");

        motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
        motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
        motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
        motors[3] = hardwareMap.get(DcMotorEx.class, "m3");

<<<<<<<
        stoneServo = hardwareMap.get(Servo.class, "stoneServo");
=======

>>>>>>>

        if(!inTeleOp){
            omniTracker = hardwareMap.get(DcMotorEx.class, "rightMotor");
            omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
    }

    //setting power for TeleOp
    public void setPower(Pose2d target) {
        double v = target.vec().norm() * MAX_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * MAX_O;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        setVelocity(targetVelocity);
    }


    public void setSlowPower(Pose2d target) {
        double v = target.vec().norm() * SLOW_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * SLOW_O;

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

    //Turning and Angle stuff
    public void setDegrees(double degrees){
        this.degrees = degrees;
    }

    public double getDegrees(){
        return degrees;
    }

    public double getCurrentAngle(){
        return globalAngle;
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

    public void executeAngle(){
        if(getAngle() == 0) {
            clockwise();
        }

        if(getDegrees() > 0) {

            if(getAngle() < getDegrees()){
                counterClockwise();

            } else {

                stopMotors();

            }

        } else {

            if(getAngle() > getDegrees()){
                clockwise();

            } else {

                stopMotors();
            }

        }


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


    //Strafing

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
        double circumference = 2 * Math.PI * TRACKER_RADIUS;
        return (int) Math.round(inches * (500 * 4) / circumference);
    }

    public double omniEncoderTicksToInches(int ticks){
        double revs = ticks / TRACKER_TICKS_PER_INCH;
        return 2 * Math.PI * TRACKER_RADIUS * revs;
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

<<<<<<<

=======
    public void moveForward(double power){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        motors[0].setPower(power);
        motors[1].setPower(power);
        motors[2].setPower(power);
        motors[3].setPower(power);
    }
>>>>>>>

    //regular driving (forward and backward)
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

        motors[0].setTargetPositionTolerance(150);
        motors[1].setTargetPositionTolerance(150);
        motors[2].setTargetPositionTolerance(150);
        motors[3].setTargetPositionTolerance(150);

        motors[0].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[1].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[2].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[3].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 4; i++){

            motors[i].setPower(power);
        }

    }

    public int getCurrentPos(){
        int motorZeroPos = motors[0].getCurrentPosition();
        int motorOnePos = motors[1].getCurrentPosition();
        int motorTwoPos = motors[2].getCurrentPosition();
        int motorThreePos = motors[3].getCurrentPosition();

        return (motorZeroPos + motorOnePos + motorTwoPos + motorThreePos) / 4;
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

    private int motorEncodersInchesToTicks(double inches) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    public void goToPosition(double position, double power){
        setMotorEncoders(motorEncodersInchesToTicks(position), power);

        targetMotorPos = motorEncodersInchesToTicks(position);

    }

<<<<<<<

=======

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

>>>>>>>
    public int getTargetMotorPos(){
        return targetMotorPos;
    }


    public boolean atLinearPos(){

        return Math.abs(getCurrentPos()) > Math.abs(targetMotorPos);

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
    ////////////////////////////////////////////////////////

    public void moveBack(double power){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
    public double realTicksPerInch(int ticks){
        double D = 1.4;
        int ticksPerRev = 2000;

        return (ticks * ( (D * 3.14) / (ticksPerRev) ) );
    }

        motors[0].setPower(-power);
        motors[1].setPower(-power);
        motors[2].setPower(-power);
        motors[3].setPower(-power);
    }


    //stop motors

    public void stopMotors(){
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

        motorsStopped = true;

    public double IcurrentPosition(){
        return realTicksPerInch(omniTracker.getCurrentPosition());
    }

    public boolean areMotorsStopped(){
        return motorsStopped;
    }

    public void IsetTrackingOmni(double power, String direction){
        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (direction.equals("right")){
            motors[0].setPower(-power);
            correctingPower(power, 1, "right");
            correctingPower(-power, 2, "right");
            motors[3].setPower(power);

        } else if (direction.equals("left")) {
            motors[0].setPower(power);
            correctingPower(-power, 1, "left");
            correctingPower(power, 2, "left");
            motors[3].setPower(-power);
        }
    //skystone grabber

    public void grab() {

        stoneServo.setPosition(grabPosition);
    }

    public void release() {

        stoneServo.setPosition(releasePosition);
    }





    public void IgoToStrafingPos(double distance, String direction){

        IsetTrackingOmni(strafePower, direction);

        targetOmniPos = distance;
    }


    public boolean IatStrafingPos(){
        return Math.abs(realTicksPerInch(omniTracker.getCurrentPosition())) > Math.abs(targetOmniPos);
    }





//////////////////////// Auto specific methods end//////////////////////////////////////////////////

    public void update(){

        double rawHeading = imu.getAngularOrientation().firstAngle;
    }


    ///////////////////////////////////////Angle Corrector//////////////////////////////////////////

    public void setZero(){
        resetAngle();
        setDegrees(0);
    }


    private void setError(){
        error = getAngle();
    }


    private double Pcontroller(){
        double output = Pcoefficient * error;
        return output;
    }


    private double PcontrollerTurn(){
        return PcoefficientTurn * error;
    }


    public void correctingPower(double defaultPower, int motorNum, String direction){
        setError();

        double correctionPower = Pcontroller();
        double turnCorrectionPower = PcontrollerTurn();

        double changeSign = Math.copySign(1, defaultPower); // 1 or -1

        if (defaultPower != 0) {

            if (direction.equals("right")) {
                newPower = defaultPower - (correctionPower * changeSign);
            }

            else if (direction.equals("left")) {
                newPower = defaultPower + (correctionPower * changeSign); // added instead of subtracted bc opposite adjustment to error is needed from right strafe
            }
        }

        else {
            if (motorNum == 0 || motorNum == 1) {
                newPower = defaultPower - turnCorrectionPower;
            }

            else { // motors 2 and 3
                newPower = defaultPower + turnCorrectionPower;
            }
        }

        motors[motorNum].setPower(newPower);
    }

}