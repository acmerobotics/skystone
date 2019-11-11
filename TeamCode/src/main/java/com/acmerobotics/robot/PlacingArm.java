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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;


@Config
public class PlacingArm {

    //TODO add feedforward method and add feedforward to RUN_TO_POSITION

    //ToDo talk about angle issue (resting arm angle and desired arm positions)

    public static double ARM_LENGTH = 15.375;

    public static double ARM_INIT;
    public static double ARM_INTAKE;
    public static double ARM_RELOCATION;

    public static double ARM_MASS = 0;

    public static double initAngle;
    public static double intakeAngle;
    public static double relocationAngle; //find angle, but be careful because an angle of 0 degrees is the angle at which the arm
                                              //motor is doing nothing. So the motor may be doing noting while the arm is at 30 degrees.
                                              // Look at method ___ for solution
    public static double restingAngle = 0; //angle where motor is doing nothing and the arm is resting

    public static double wantInitAngle = 0; //find angle
    public static double wantIntakeAngle = 0; //find angle
    public static double wantRelocationAngle = 90;



    public static double RADIUS = 0;

    private MotionProfile profile;

    private double startTime;
    private double error;
    private double correction;
    private double targetPosition;

    private DcMotorEx armMotor;
    private Servo handServo;

    private double handOpenPos = 0; //add angle position at which hand will open
    private  double handClosePos = 0; // add angle position at which hand will close

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

    public void updade(TelemetryPacket packet){// telemetry seems to only be used in teleOP. So is
                                               // telemetryPacket like teleOP but made for code outside TeleOp class?
        packet.put("arm mode", armMode.toString());
        packet.put("position", getPosition());

        switch (armMode){
            case HOLD_POSITION:
                error = getPosition() - targetPosition; //error is created when the arm leaves its target position
                packet.put("error", error);

                correction = pidController.update(error);
                internalSetVelocity(-correction);
                packet.put("arm correction", -correction);


            case RUN_TO_POSITION:
                double t = System.currentTimeMillis() - startTime/ 1000;// start time is used as move the motion state so it is out of its 0 or motion state start position
                                                                        // that way pid won't get a 0 error when not at set point. Start time skips over motion state start
                                                                        //position to not confuse pid (only pid sees a skipped motion state start position).

                MotionState target = profile.get(t); //notice how a motion state (target) is set equal to a motion profile (profile). The get(t) returns
                                                      // the motion state of profile at t.
                error = getPosition() - target.getX();
                packet.put("error", error);
                correction = pidController.update(error);

                internalSetVelocity(-correction); //add feedforward
                packet.put("arm correction", -correction);


                if(t > profile.duration()){
                    packet.put("complete", true);
                    armMode = armMode.HOLD_POSITION;
                    targetPosition = profile.end().getX();
                    //return;  ???
                }

        }
    }


    public void goToPosition(double position){
        // initializes motion profiling and starts RUN_TO_POSITION.
        pidController = new PIDController();

        profile = MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(getPosition(), 0, 0, 0), //start
                new MotionState(position, 0, 0, 0),// goal
                V,A,J
        );
        startTime = System.currentTimeMillis();
        armMode = armMode.RUN_TO_POSITION;

    }

    public void armInitPosition(){
        //Radian of init is calculated. Motion profiling and pid are initialized. Arm moves to init position.

        initAngle = getActualAngle(wantInitAngle, restingAngle);

        ARM_INIT = getRadianLen(initAngle, ARM_LENGTH);

        goToPosition(ARM_INIT);
        pidController = new PIDController(P, I, D);

    }

    public void armIntakePosition(){
        //Radian of intake position is calculated. Motion profiling and pid are initialized. Arm moves to intake position.

        intakeAngle = getActualAngle(wantIntakeAngle, restingAngle);

        ARM_INTAKE = getRadianLen(intakeAngle, ARM_LENGTH);

        goToPosition(ARM_INTAKE);
        pidController = new PIDController(P, I, D);
    }

    public void armRelocationPosition(){
        //Arm angle is set to 90 degrees. Motion profiling and pid are initialized. Arm moves to relocation position

        relocationAngle = getActualAngle(wantRelocationAngle, restingAngle);

        ARM_RELOCATION = getRadianLen(relocationAngle, ARM_LENGTH);

        goToPosition(ARM_RELOCATION);
        pidController = new PIDController(P, I, D);

    }

    public double getRadianLen(double angle, double radius){
        // returns radian length (arm movement curve length)

        double i =  (angle/360) * 2 * Math.PI * radius;
        return i;
    }

    public void setServo(String position){
        // take in close or open then set servo position accordingly

        if (position.equals("open")){
            //open hand
            handServo.setPosition(handOpenPos);
        }

        if (position.equals("open")){
            //close hand
            handServo.setPosition(handClosePos);
        }
    }

    public double getActualAngle(double angle, double restingAngle){
        // angle is the angle you want the arm to be placed when the lift is angle 0. a is the angle that will actual work with goToPosition
        // look at "Actual Angel" paper for details.
        double a = angle - restingAngle;
        return a;
    }

}