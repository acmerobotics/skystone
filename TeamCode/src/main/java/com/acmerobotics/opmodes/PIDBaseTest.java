package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.robot.PIDBaseSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="pidBaseOpMode")

public class PIDBaseTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        PIDBaseSubsystem subsystem = new PIDBaseSubsystem(hardwareMap);

        waitForStart();

        while(!isStopRequested()){

            if(gamepad1.a){
                subsystem.motor1MoveInches();
            }

            if(gamepad1.b){
                subsystem.moveBoth();
            }


            telemetry.addData("m1 position", subsystem.motor1.getCurrentPosition());
            telemetry.addData("m2 position", subsystem.motor2.getCurrentPosition());

            telemetry.addData("m1 target", subsystem.motor1.getTargetPosition());
            telemetry.addData("m2 target", subsystem.motor2.getTargetPosition());

            telemetry.update();


            dashboardTelemetry.addData("m1 position", subsystem.motor1.getCurrentPosition());
            dashboardTelemetry.addData("m2 position", subsystem.motor2.getCurrentPosition());

            dashboardTelemetry.addData("m1 target", subsystem.motor1.getTargetPosition());
            dashboardTelemetry.addData("m2 target", subsystem.motor2.getTargetPosition());

            dashboardTelemetry.update();
        }

    }
}
