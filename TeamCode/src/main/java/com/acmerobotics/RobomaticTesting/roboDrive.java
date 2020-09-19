package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.opmodes.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config

public class roboDrive extends Subsystem {

    //constants
    private static final double WHEEL_RADIUS = 2;

    private static final double TRACKER_RADIUS = DistanceUnit.INCH.fromMm(35.0 / 2.0);
    private static final double TRACKER_TICKS_PER_INCH = (500 * 4) / (2 * TRACKER_RADIUS * Math.PI);

    //hardware devices
    public DcMotorEx[] motors = new DcMotorEx[4];
    public DcMotorEx omniTracker;
    private CachingSensor imuSensor;
    private Servo stoneServo;

    // motors and motor encoder variables
    private static double MAX_V = 30;
    private static double MAX_O = 30;
    public static double SLOW_V = MAX_V/2;
    public static double SLOW_O = MAX_O/2;

    private boolean motorsStopped = false;

    private double ticksPerRev = 560.0;

    private int targetMotorPos;

    // misc. motor powers
    public double strafePower = 0.28;
    public double turnPower = 0.5;

    // omni variables
    public double targetOmniPos;
    private boolean atTargetOmniPos = false;

    // imu variables
    //Orientation lastAngle = new Orientation();
    private double lastAngle;
    private double degrees;
    private double globalAngle;

    // servo positions
    private double grabPosition = 0.75;
    private double releasePosition = 0.20;

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

    private Pose2d targetVelocity = new Pose2d(0, 0, 0);

    private double wheelOmega = 0;

    // correction variables
    public double Pcoefficient = 0.1; // 0.2
    public static double PcoefficientTurn = 0.04;

    public double error;
    public double newPower;

    private boolean inTeleOp = false;

    public roboDrive(Robot robot, LinearOpMode opMode){
        super("Drive");

        stoneServo = robot.getServo("stoneServo");

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z)); // creates BN0055-IMU-Impl, imu orientation is remapped
        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle); // gets heading
        robot.registerCachingSensor(imuSensor); // adds imu to caching sensors, will then update the heading

        for (int i=0; i<4;i++){
            motors[i] = robot.getMotor("m" + i);
        }

        // this is not the best way to determine if the GenDrive is in teleOp I will change it later
        if (String.valueOf(opMode).equals("roboTeleOp")){
            inTeleOp = true;
        }

        if(!inTeleOp){
            omniTracker = robot.getMotor("intakeMotor");
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

        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }

        //Orientation heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    @Override
    public void update(Canvas overlay){
        telemetryData.addData("m0 current pos", motors[0].getCurrentPosition());
        telemetryData.addData("m0 power", motors[0].getPower());

        telemetryData.addData("heading (radians)", imuSensor.getValue());
        telemetryData.addData("heading (degrees)", getAngle());
        telemetryData.addData("inTeleOp", inTeleOp);
    }


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

    // turning and angle stuff

    private double convertToDegrees(double radians){
        return (180 * radians) / 3.14;
    }

    public void setDegrees(double degrees){
        this.degrees = degrees;
    }

    public double getDegrees(){
        return degrees;
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

    public double getCurrentAngle(){
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

    // strafing
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

    // regular driving (forward and backward)
    private void setMotorEncoders(int distance, double power){
        motors[0].setTargetPosition(distance);
        motors[1].setTargetPosition(distance);
        motors[2].setTargetPosition(distance);
        motors[3].setTargetPosition(distance);

        motors[0].setTargetPositionTolerance(30);
        motors[1].setTargetPositionTolerance(30);
        motors[2].setTargetPositionTolerance(30);
        motors[3].setTargetPositionTolerance(30);

        motors[0].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[1].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[2].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[3].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        for (int i = 0; i < 4; i++){

            motors[i].setPower(power);
        }

    }

    public void goToPosition(double position, double power){
        setMotorEncoders(motorEncodersInchesToTicks(position), power);

        targetMotorPos = motorEncodersInchesToTicks(position);

    }

    public void LsetMotorEncoders(String direction, int distance, double power){
        motors[0].setTargetPosition(distance);
        motors[1].setTargetPosition(distance);
        motors[2].setTargetPosition(distance);
        motors[3].setTargetPosition(distance);

        motors[0].setTargetPositionTolerance(30);
        motors[1].setTargetPositionTolerance(30);
        motors[2].setTargetPositionTolerance(30);
        motors[3].setTargetPositionTolerance(30);

        motors[0].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[1].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[2].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motors[3].setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (direction.equals("forward")) {
            motors[0].setPower(power);
            motors[1].setPower(power);
            correctingPower(power, 2, "right");
            correctingPower(power, 3, "right");
        }

        if (direction.equals("back")){
            motors[0].setPower(power);
            motors[1].setPower(power);
            correctingPower(power, 2, "left");
            correctingPower(power, 3, "left");
        }
    }


    public void LgoToPosition(String direction, double position, double power){
        LsetMotorEncoders(direction, motorEncodersInchesToTicks(position), power);

        targetMotorPos = motorEncodersInchesToTicks(position);

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


    public int getTargetMotorPos(){
        return targetMotorPos;
    }

    public boolean atLinearPos(){

        return Math.abs(targetMotorPos - getCurrentPos()) < 20;

    }


    // servo
    public void grab() {

        stoneServo.setPosition(grabPosition);
    }

    public void release() {

        stoneServo.setPosition(releasePosition);
    }

    public double realTicksPerInch(int ticks){
        double D = 1.4;
        int ticksPerRev = 2000;

        return (ticks * ( (D * 3.14) / (ticksPerRev) ) );
    }


    public double IcurrentPosition(){
        return realTicksPerInch(omniTracker.getCurrentPosition());
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
    }


    public void IgoToStrafingPos(double distance, String direction){

        IsetTrackingOmni(strafePower, direction);

        targetOmniPos = distance;
    }


    public boolean IatStrafingPos(){
        return Math.abs(realTicksPerInch(omniTracker.getCurrentPosition())) > Math.abs(targetOmniPos);
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
