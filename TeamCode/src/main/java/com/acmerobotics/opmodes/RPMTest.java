package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.util.RPMReader;
import com.acmerobotics.util.calcRPM;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="RPMTest")
@Config
public class RPMTest extends LinearOpMode {
    public static double targetRPM = 60;

    @Override
    public void runOpMode(){

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry Telemetry = dashboard.getTelemetry();

        calcRPM getMotor = new calcRPM(hardwareMap); // just a place to get a motor

        RPMReader motorRPM = new RPMReader(hardwareMap, getMotor.motor, 560);

        waitForStart();

        motorRPM.time.reset();

        while (!isStopRequested()){

            if (gamepad1.a){
                motorRPM.motor.setPower(0.15); // max rpm should be 300 according to Rev website
                // 44 rpm
            }


            if (gamepad1.b){
                motorRPM.motor.setPower(0.2);
                // 62 rpm manual 60 rmp reader
            }


            if (gamepad1.y){
                motorRPM.motor.setPower(1);
                //
            }


            if (gamepad1.x){
                motorRPM.motor.setPower(0);
            }


            if (gamepad1.right_trigger > 0){
                motorRPM.motor.setPower(gamepad1.right_trigger);
            }

            if (gamepad1.dpad_up){
                motorRPM.setRPM(targetRPM);
            }

            telemetry.addData("rpm", motorRPM.getRPM());
            Telemetry.addData("rpm", motorRPM.getRPM());
//
            telemetry.addData("power", motorRPM.motor.getPower());
            Telemetry.addData("power", motorRPM.motor.getPower());
//
            telemetry.addData("gamepad trigger", gamepad1.right_trigger);
//
            telemetry.update();
            Telemetry.update();
        }
    }
}
