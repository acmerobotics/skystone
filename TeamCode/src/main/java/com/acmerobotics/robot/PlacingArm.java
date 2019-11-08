package com.acmerobotics.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.robomatic.util.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.robot.Lift;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;


@Config
public class PlacingArm {

    //TODO talk about moving this into the lift code. Might be worth it. Maybe idk

    //TODO add feedforward method the add feedforward to RUN_TO_POSITION

    //TODO add telemetry packet to both RUN_TO_POSITION and HOLD_POSITION

    public static double ARM_LENGTH = 15.375;

    public static double ARM_INIT;
    public static double ARM_INTAKE;
    public static double ARM_RELOCATION;
    public static double ARM_MASS = 0;

    public static double initAngle = 0; ////////////////////////////////////find angle
    public static double intakeAngle = 0; ///////////////////////////////find angle
    public static double relocationAngle = 90;


    public static double RADIUS = 0;

    private MotionProfile profile;

    private double startTime;
    private double error;
    private double correction;
    private double targetPosition;

    private DcMotorEx armMotor;

    private PIDController pidController;

    public double offset;

    public static double P = 0;
    public static double I = 0;
    public static double D = 0;
    public static double V = 0;
    public static double A = 0;
    public static double J = 0;

    private enum ArmMode{
        HOLD_POSITION,
        RUN_TO_POSITION,
        DRIVER_CONTROLLED,

    }

    private ArmMode armMode = ArmMode.DRIVER_CONTROLLED;

    public PlacingArm(HardwareMap hardwareMap){

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        pidController = new PIDController(P, I, D);

    }


    public double getPosition(){
        return internalGetPosition() + offset;

    }

    public double internalGetPosition(){
        return ((armMotor.getCurrentPosition() / (armMotor.getMotorType().getTicksPerRev())) * Math.PI * RADIUS * 2);

    }

    public void setPower(double power){
        internalSetVelocity(power);

    }

    public void setPosition(double position){
        offset = position - internalGetPosition();

    }

    public void internalSetVelocity(double v){
        armMotor.setPower(v);

    }

    public void updade(TelemetryPacket packet){
        packet.put("arm mode", armMode.toString());
        packet.put("position", getPosition());

        switch (armMode){
            case HOLD_POSITION:
                error = getPosition() - targetPosition; //error is created when the arm leaves its target position
                correction = pidController.update(error);
                internalSetVelocity(-correction);


            case RUN_TO_POSITION:
                double t = System.currentTimeMillis() - startTime/ 1000;// start time is used as move the motion state so it is out of its 0 or motion state start position
                                                                        // that way pid won't get a 0 error when not at set point. Start time skips over motion state start
                                                                        //position to not confuse pid (only pid sees a skipped motion state start position).

                MotionState target = profile.get(t); //notice how a motion state (target) is set equal to a motion profile (profile). The get(t) returns a motion state at t.
                error = getPosition() - target.getX();
                correction = pidController.update(error);
                internalSetVelocity(-correction); //add feedforward

                if(t > profile.duration()){
                    armMode = armMode.HOLD_POSITION;
                    targetPosition = profile.end().getX();
                    //return; ?????
                }

        }
    }


    public void goToPosition(double position){
        pidController = new PIDController();

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0),
                new MotionState(position, 0, 0, 0),
                V,A,J
        );
        startTime = System.currentTimeMillis();
        armMode = armMode.RUN_TO_POSITION;

    }

    public void armInitPosition(){
        ARM_INIT = getRadianLen(initAngle, ARM_LENGTH);

        goToPosition(ARM_INIT);
        pidController = new PIDController(P, I, D);

    }

    public void armIntakePosition(){
        ARM_INTAKE = getRadianLen(intakeAngle, ARM_LENGTH);

        goToPosition(ARM_INTAKE);
        pidController = new PIDController(P, I, D);
    }

    public void armRelocationPosition(){
        ARM_RELOCATION = getRadianLen(relocationAngle, ARM_LENGTH);

        goToPosition(ARM_RELOCATION);
        pidController = new PIDController(P, I, D);

    }

    public double getRadianLen(double angle, double radius){
        double i =  (angle/360) * 2 * Math.PI * radius;
        return i;
    }

}
