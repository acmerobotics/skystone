package com.acmerobotics.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robot.AngleCorrector;
import com.acmerobotics.robot.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="AngleCorrection Test")
@Config

public class AngleCorrection extends LinearOpMode{
    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        Drive drive = new Drive(hardwareMap, false);
        ElapsedTime time = new ElapsedTime();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.resetAngle();
        drive.setZero();

        waitForStart();

        while (!isStopRequested()){

//            drive.motors[0].setPower(-0.28);
//            drive.motors[3].setPower(0.28);
//
//            drive.correctingPower(0.28, 1);
//            drive.correctingPower(-0.28, 2);


            // the bottom motors sets are for stationary angle correction
//            angleCorrector.setNewPower(0, 0);
//            angleCorrector.setNewPower(0, 1);
//
//            angleCorrector.setNewPower(0, 2);
//            angleCorrector.setNewPower(0, 3);

            switch(state) {
                case 0:
                drive.IgoToStrafingPos(24, "right");

                if (drive.IatStrafingPos()) {
                    drive.stopMotors();
                    time.reset();
                    state++;
                }
                break;
                case 1:
                    if (time.seconds() < 1) {
                        drive.correctingPower(0, 0);
                        drive.correctingPower(0, 1);
                        drive.correctingPower(0, 2);
                        drive.correctingPower(0, 3);
                    }
                    else{
                        state++;
                    }
                    break;

                case 2:
                    dashboardTelemetry.addLine("here");

            }


            dashboardTelemetry.addData("get degrees", drive.getDegrees());
            dashboardTelemetry.addData("get angle", drive.getAngle());
            dashboardTelemetry.addData("error", drive.error);
            dashboardTelemetry.addData("omni position", drive.IcurrentPosition());

            dashboardTelemetry.addData("power 0", drive.motors[0].getPower());
            dashboardTelemetry.addData("power 1", drive.motors[1].getPower());
            dashboardTelemetry.addData("power 2", drive.motors[2].getPower());
            dashboardTelemetry.addData("power 3", drive.motors[3].getPower());

            dashboardTelemetry.update();

            /////////////////////////////////////////////////////////////////
        }
    }
}
