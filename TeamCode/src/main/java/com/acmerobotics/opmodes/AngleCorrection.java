package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.AngleCorrector;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="AngleCorrection Test")
@Config

public class AngleCorrection extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);
        AngleCorrector angleCorrector = new AngleCorrector(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.resetAngle();
        angleCorrector.setZero();

        waitForStart();

        while (!isStopRequested()){

            drive.motors[0].setPower(-0.28);
            drive.motors[1].setPower(-0.28);


            angleCorrector.setNewPower();


            dashboardTelemetry.addData("get degrees", drive.getDegrees());
            dashboardTelemetry.addData("get angle", drive.getAngle());
            dashboardTelemetry.addData("error", angleCorrector.error);
            dashboardTelemetry.addData("new power", angleCorrector.newPower);
            dashboardTelemetry.addData("power", drive.motors[0].getPower());

            dashboardTelemetry.update();
        }
    }
}
