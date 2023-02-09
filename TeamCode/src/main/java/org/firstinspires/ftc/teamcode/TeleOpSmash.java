package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpSmash extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotSmash robot = new RobotSmash(hardwareMap);

        waitForStart();
        while (opModeIsActive()){
            robot.LiftPID(gamepad2);
            robot.DriveMovement(gamepad1);
            robot.ClawManager(gamepad2);

            telemetry.addData("Lift Position Right", robot.getLiftRightPosition());
            telemetry.addData("Lift Position Left", robot.getLiftLeftPosition());
            telemetry.addData("Lift Target", robot.getLiftTarget());
            telemetry.addData("Lift Power", robot.getLiftPower());
            telemetry.update();
        }
    }

}