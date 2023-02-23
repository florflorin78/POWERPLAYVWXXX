package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="1TELEOP")
public class TeleOpSmash extends LinearOpMode {

    boolean lastA = false;
    boolean lastLB = false;
    boolean highLevel = false;
    boolean secondHalf = false;

    Gamepad.RumbleEffect StartTeleOP;
    Gamepad.RumbleEffect EndGame;
    Gamepad.RumbleEffect STOP;

    ElapsedTime runtime = new ElapsedTime();

    final double HALF_TIME = 80.0;
    final double STOP_GAME = 120.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotSmash robot = new RobotSmash(hardwareMap);

        StartTeleOP = new Gamepad.RumbleEffect.Builder()
                .addStep(0.5, 0.5, 250)  //  Rumble right motor 100% for 500 mSec
                .build();

        EndGame = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 500)  //  Rumble right motor 100% for 500 mSec
                .build();
        STOP = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 1000)  //  Rumble right motor 100% for 500 mSec
                .build();

        runtime.reset();
        waitForStart();
        while (opModeIsActive()){
            robot.LiftPID(gamepad2);
            robot.DriveMovement(gamepad1);
            robot.ClawManager(gamepad2);



            telemetry.addData("Lift Position Right", robot.getLiftRightPosition());
            telemetry.addData("Lift Position Left", robot.getLiftLeftPosition());
            telemetry.addData("Lift Target", robot.getLiftTarget());
            telemetry.addData("Lift Power", robot.getLiftPower());

            if(runtime.seconds() < 1) {
                gamepad1.runRumbleEffect(StartTeleOP);
                gamepad2.runRumbleEffect(StartTeleOP);
            }

            if(runtime.seconds() >=118.0 && runtime.seconds() <= STOP_GAME) {
                gamepad1.runRumbleEffect(STOP);
                gamepad2.runRumbleEffect(STOP);
            }
            if ((runtime.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(EndGame);
                gamepad2.runRumbleEffect(EndGame);
                secondHalf =true;}

            if (!secondHalf) {
                telemetry.addData(">", "Halftime Alert Countdown: %3.0f Sec \n", (HALF_TIME - runtime.seconds()) );
            }

            telemetry.addData("Runtime Seconds - ", runtime.seconds());
            telemetry.update();

        }
    }

}