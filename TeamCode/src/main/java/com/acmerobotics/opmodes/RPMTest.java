package com.acmerobotics.opmodes;

import com.acmerobotics.util.calcRPM;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="RPMTest")
public class RPMTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        while (!isStopRequested()){
            calcRPM motorRPM = new calcRPM(hardwareMap);

            if (gamepad1.a){
                motorRPM.motor.setPower(1);
            }

            else {
                motorRPM.motor.setPower(0);
            }

            motorRPM.getRPM();
        }
    }
}
