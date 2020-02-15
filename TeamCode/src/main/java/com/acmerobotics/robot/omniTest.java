package com.acmerobotics.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="omniTest")
@Config
public class omniTest extends LinearOpMode {

    private int state = 0;
    public static double strafePosition = 12;

    @Override
    public void runOpMode() throws InterruptedException {
        Drive drive = new Drive(hardwareMap, false);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        drive.resetEncoderOmni();
        drive.resetEncoders();

        waitForStart();

        while (!isStopRequested()){

            switch (state) {
                case 0:

                    drive.IgoToStrafingPos(strafePosition, 0.3, "left");
                    state++;
                    break;

                case 1:

                    if(drive.IatStrafingPos()){
                        drive.stopMotors();
                        state++;
                    }

                    break;
            }

//            dashboardTelemetry.addData("pos 0", drive.motors[0].getCurrentPosition());
//            dashboardTelemetry.addData("pos 1", drive.motors[1].getCurrentPosition());
//            dashboardTelemetry.addData("pos 2", drive.motors[2].getCurrentPosition());
//            dashboardTelemetry.addData("pos 3", drive.motors[3].getCurrentPosition());
//
//            dashboardTelemetry.update();

            telemetry.addData("real ticks per inch (inches traveled)", drive.realTicksPerInch(drive.omniTracker.getCurrentPosition()));
            telemetry.addData("current position", drive.omniTracker.getCurrentPosition());

            telemetry.update();

        }
    }
}