package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.util.RPMReader;
import com.acmerobotics.util.calcRPM;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="RPMTest")
public class RPMTest extends LinearOpMode {

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
            }

            else {
                motorRPM.motor.setPower(0);
            }

            if (gamepad1.right_trigger > 0){
                motorRPM.motor.setPower(gamepad1.right_trigger);
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
