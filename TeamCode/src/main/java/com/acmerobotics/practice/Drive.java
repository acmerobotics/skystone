package com.acmerobotics.practice;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {

    private DcMotor[] motors = new DcMotor[4];
    public static final double MAX_V = 40;
    public static final double MAX_O = 1;
    public static final double RADIUS = 2;
    public static final Vector2d[] WHEEL_POSITIONS = {
            new Vector2d(6, 7.5),
            new Vector2d(-6, 7.5),
            new Vector2d(-6, -7.5),
            new Vector2d(6, -7.5)
    };

    public static final Vector2d[] WHEEL_DIRECTIONS = {
      new Vector2d(1, -1).unit(),
      new Vector2d(1, 1).unit(),
      new Vector2d(-1, -1).unit(),
      new Vector2d(-1, 1).unit()
    };

    public Drive(HardwareMap hardwareMap){

       motors[0] = hardwareMap.get(DcMotorEx.class, "m0");
       motors[1] = hardwareMap.get(DcMotorEx.class, "m1");
       motors[2] = hardwareMap.get(DcMotorEx.class, "m2");
       motors[3] = hardwareMap.get(DcMotorEx.class, "m3");

    }

    public void setPower(Vector2d v,double omega) {

         setVelocity(v.times(MAX_V), omega * MAX_O);
    }

    public void setVelocity(Vector2d v, double omega){
        for (int i = 0; i < 4; i++){
            Vector2d wheelVelocity = new Vector2d(v.x() - omega * WHEEL_POSITIONS[i].y(), v.y() + omega * WHEEL_POSITIONS[i].x());
            double wheelOmega = wheelVelocity.dot(WHEEL_DIRECTIONS[i])* Math.sqrt(2) * RADIUS;
        }
    }
}
