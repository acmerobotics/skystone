package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// TODO rename the final version of this class GenDrive or something

public class generalizedDrive extends Subsystem {

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

    private double ticksPerRev = 560.0;

    // imu variables
    //Orientation lastAngle = new Orientation();
    private double lastAngle;
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

    private PIDController pidController;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;

    // might need different coeff for turning
    public static double Pturn = 0;
    public static double Iturn = 0;
    public static double Dturn = 0;

    // event triggers

    private enum AutoMode{
        UNKNOWN,
        Y,
        STRAFE,
        TURN
    }

    private double Ytarget = 0;
    private double Xtarget = 0;
    private double turnTarget = 0;

    // should be changed if needed (in inches might change to ticks if needed)
    private double YErrorTolerance = 3;
    private double XErrorTolerance = 5;
    private double headingErrorTolerance = 5;



    private AutoMode autoMode = AutoMode.UNKNOWN;
    private AutoMode lastAutoMode = AutoMode.UNKNOWN;

    private LinearOpMode opMode;


    public generalizedDrive(Robot robot, LinearOpMode opMode){
        super("Drive");

        this.opMode = opMode;

        stoneServo = robot.getServo("stoneServo");

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z)); // creates BN0055-IMU-Impl, imu orientation is remapped
        imuSensor = new CachingSensor<>(() -> imu.getAngularOrientation().firstAngle); // gets heading
        robot.registerCachingSensor(imuSensor); // adds imu to caching sensors, will then update the heading

        pidController = new PIDController();

        for (int i=0; i<4;i++){
            motors[i] = robot.getMotor("m" + i);
        }

        if (opModeEquals("roboTeleOp") || opModeEquals("TeleOpUsingGenDrive")){ // insert a list of or statements with TeleOps that use the drive
            inTeleOp = true;
        }
        else {
            inTeleOp = false;
        }

        if(!inTeleOp){
            omniTracker = robot.getMotor("intakeMotor");
            omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motors[0].setDirection(DcMotorEx.Direction.REVERSE);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.FORWARD);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
            }

        } else {

            motors[0].setDirection(DcMotorEx.Direction.FORWARD);
            motors[1].setDirection(DcMotorEx.Direction.REVERSE);
            motors[2].setDirection(DcMotorEx.Direction.FORWARD);
            motors[3].setDirection(DcMotorEx.Direction.REVERSE);

            for (int i = 0; i < 4; i++){
                motors[i].setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            }

        }

        //Orientation heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    @Override
    public void update(Canvas overlay){
        telemetryData.addData("m0 current pos ", motors[0].getCurrentPosition());
        telemetryData.addData("m0 power ", motors[0].getPower());

        telemetryData.addData("heading (radians)", imuSensor.getValue());
        telemetryData.addData("heading (degrees)", getAngle());
        telemetryData.addData("inTeleOp ", inTeleOp);

        telemetryData.addData("autoMode ", autoMode);

        if (inTeleOp){

            setVelocity(targetVelocity);
        }

        if (!inTeleOp){

            // adjust error for a motor power
            //error = error /  12; // might move this statement to a method
            pidController.setOutputBounds(-1, 1);

            if (autoMode != lastAutoMode){ // will reset between autoModes but user still has to
                                            // reset between strafes and turns not Y's
                resetEncoders();
                resetEncoderOmni();
                lastAutoMode = autoMode;
            }

            switch (autoMode){
                case UNKNOWN:

                    pidController = new PIDController(P, I, D);

                    // create variables that will be used in other cases
                    double target;
                    double correction;

                case Y:

                    pidController = new PIDController(P, I, D);

                    target = motors[0].getCurrentPosition() + Ytarget;
                    error = target - motors[0].getCurrentPosition();

                    correction = pidController.update(error);

                    for (int i= 0; i < 4; i++) {
                        motors[i].setPower(correction);
                    }

                    break;


                case STRAFE:

                    pidController = new PIDController(P, I, D);

                    // TODO add strafe alignment correction
                    // the target and error are based on motors 0 and 2 but the values and the same
                    // for motors 1 and 3 but opposite sign
                    target = strafeCurrentPosition() + Xtarget;
                    error = target - strafeCurrentPosition();

                    correction = pidController.update(error);

                    motors[0].setPower(correction);
                    motors[1].setPower(-correction);
                    motors[2].setPower(correction);
                    motors[3].setPower(-correction);

                    break;

                case TURN:

                    pidController = new PIDController(Pturn, Iturn, Dturn);

                    target = getAngle() + turnTarget;
                    error = target - getAngle();

                    correction = pidController.update(error);

                    motors[0].setPower(correction);
                    motors[1].setPower(correction);
                    motors[2].setPower(-correction);
                    motors[3].setPower(-correction);

                    break;

            }
        }
    }


    //////////////////////////////////  TeleOP  ////////////////////////////////////////////

    public void setPower(Pose2d target) {
        double v = target.vec().norm() * MAX_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * MAX_O;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        //setVelocity(targetVelocity);

    }


    public void setSlowPower(Pose2d target) {
        double v = target.vec().norm() * SLOW_V;
        double theta = Math.atan2(target.getX(), target.getY());
        double omega = target.getHeading() * SLOW_O;

        targetVelocity = new Pose2d(v * Math.cos(theta), v * Math.sin(theta), omega);

        //setVelocity(targetVelocity);
    }


    public void setVelocity(Pose2d v) {
        for (int i = 0; i < 4; i++) {
            Vector2d wheelVelocity = new Vector2d(v.getX() - v.getHeading() * WHEEL_POSITIONS[i].getY(),
                    v.getY() + v.getHeading() * WHEEL_POSITIONS[i].getX());
            wheelOmega = (wheelVelocity.dot(ROTOR_DIRECTIONS[i]) * Math.sqrt(2)) / WHEEL_RADIUS;
            motors[i].setVelocity(wheelOmega, AngleUnit.RADIANS);


        }

    }


    ///////////////////////////////////////// Auto ////////////////////////////////////////////

    // linear movements

    public void moveForward(double inches){
        Ytarget = inches;
        autoMode = AutoMode.Y;
    }

    public void moveBack(double inches){
        Ytarget = -inches;
        autoMode = AutoMode.Y;
    }

    public void strafeRight(double inches){
        Xtarget = inches;
        autoMode = AutoMode.STRAFE;
    }

    public void strafeLeft(double inches){
        Xtarget = -inches;
        autoMode = AutoMode.STRAFE;
    }


    public boolean atYPosition(){
        if (Math.abs(error) < YErrorTolerance){
            return true;
        }
        else{
            return false;
        }
    }


    public boolean atStrafePosition(){
        if (Math.abs(error) < XErrorTolerance){
            return true;
        }
        else{
            return false;
        }
    }

    public double strafeCurrentPosition(){
        return omniTicksPerInch(omniTracker.getCurrentPosition());
    }


    // turning

    public void turnRight(double degrees){
        turnTarget = degrees;
        autoMode = AutoMode.TURN;
    }

    public void turnLeft(double degrees){
        turnTarget = -degrees;
        autoMode = AutoMode.TURN;
    }


    public boolean atTurningPosition() {
        if (Math.abs(error) < headingErrorTolerance) {
            return true;
        } else {
            return false;
        }
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

    private int omniEncodersInchesToTicks(int inches) {
        double circumference = 2 * Math.PI * TRACKER_RADIUS;
        return (int) Math.round(inches * (500 * 4) / circumference);
    }

    private int motorEncodersInchesToTicks(double inches) {
        double circumference = 2 * Math.PI * WHEEL_RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }


    ///////////////////////////////////////Angle Corrector//////////////////////////////////////////

    public void setZero(){
        resetAngle();
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


    // servo
    public void grab() {

        stoneServo.setPosition(grabPosition);
    }

    public void release() {

        stoneServo.setPosition(releasePosition);
    }


    // misc

    private double omniTicksPerInch(int ticks){
        double D = 1.4;
        int ticksPerRev = 2000;
        double circumference = D * 3.14;

        return (circumference *  ticks / ticksPerRev);
    }

    public double ticksToInches(int ticks) {
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * WHEEL_RADIUS * revs;

    }

    public void stopMotors(){
        motors[0].setPower(0);
        motors[1].setPower(0);
        motors[2].setPower(0);
        motors[3].setPower(0);

    }

    private double convertToDegrees(double radians){
        return (180 * radians) / 3.14;
    }

    public void resetEncoderOmni(){
        omniTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void resetEncoders(){
        for(int i = 0; i < 4; i++){
            motors[i].setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void stopAndResetMotors(){
        stopMotors();
        resetEncoders();
        resetEncoderOmni();
    }

    private boolean opModeEquals(String opMode){
        if (String.valueOf(this.opMode).equals(opMode)){
            return true;
        }

        else{
            return false;
        }
    }

}
