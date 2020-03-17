package com.acmerobotics.RobomaticTesting;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.hardware.CachingSensor;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/***
 * Robomatic Subsystem test/example with explanations of what happens behind the scenes
 */

@Config

public class roboSubsystem extends Subsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    private CachingSensor imuSensor;
    private DigitalChannel hallEffect;

    public roboSubsystem(Robot robot){
        super("roboSub"); // super is basically calling and creating the parent class, in this case the constructor of subsystem
        // ^ this will add roboSub in the begining of all of the telemetryData.addData string-value outputs

        motor1 = robot.getMotor("m1"); // returns a CachingDcMotor object which is basically a modified DcMotorEx that reduces the amount
                                                        // of checks the motor gets during the runOpMode loop, it also recreates the FTC SDK methods
                                                        // (I think it would be a good idea to add the PIDBase system to the CachingDcMotorEx class)
        motor2 = robot.getMotor("m2");

        hallEffect = robot.getDigitalChannel("bottomHallEffect"); // returns a BulkReadDigitalChannel obj but not sure on the background that occurs here but it has something to do with a
                                                                        // a controller and a channel which I think refer to HUB # and port of digital device
                                                                        // the rest of a method recreation of FTC SDK digitalChannel

        BNO055IMUImpl imu = robot.getRevHubImu(0, new Robot.Orientation(Robot.Axis.POSITIVE_X, Robot.Axis.POSITIVE_Y, Robot.Axis.POSITIVE_Z));
        // ^ had 2 parts {1} (1st getRevHubImu) creates BN0055-IMU-Impl, creates a parameter obj so that the angleUnit can be specified to radians, the parameters are then inited
        // {2} the IMU axises are remapped (probably for roadrunner or just for convenience)
        // note: supposedly y and z are swapped (not sure what else) using bitwise operators but when I tried to recreate the action I was not if I did it correctly

        imuSensor = new CachingSensor<>(()->imu.getAngularOrientation().firstAngle); // first angle is heading or yaw
        //                                ^lambda expression adds imu.getAngu... statement to the update method of caching sensor which overrides the CachingHardwareDevice
        robot.registerCachingSensor(imuSensor);
        // imuSensor is added to a list of caching sensors (if not there already), when the robot update method is ran then the for each sensor in the caching sensor list
        //  the update method of each will run (update method functionality in this case is set by the lambda expression)
    }

    @Override
    public void update(Canvas canvas){ // don't know what canvas is but I know it has something to do with the dashboard
        // puts given addData inputs into dashboard with the subsystem prefix in the beginning of the string (very useful for dashboard tuning and trouble shooting)
        telemetryData.addData("m1 current pos", motor1.getCurrentPosition());
        telemetryData.addData("m2 current pos", motor2.getCurrentPosition());

        telemetryData.addData("m1 motor ticks per rev", motor1.getMotorType().getTicksPerRev());
        telemetryData.addData("get m1 velocity", motor1.getVelocity(AngleUnit.DEGREES));

        telemetryData.addData("heading", imuSensor.getValue()); // returns sensor value
    }

    public void moveBoth(double power){
        motor1.setPower(power); // runs FTC setPower but instead of running setPower multiply times even when the power being set had changed, the power is set once and
        // setPower won't run again unless a different power from the last power set is given
        motor2.setPower(power);
    }

    public void goTo(int position, double power) {

        motor1.setTargetPosition(position);
        motor2.setTargetPosition(position);

        motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void setVelocity(double velocity){
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setVelocity(180, AngleUnit.DEGREES);
        motor1.setVelocity(360, AngleUnit.DEGREES);
    }
}
