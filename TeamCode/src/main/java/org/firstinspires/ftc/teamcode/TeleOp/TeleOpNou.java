package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.Advanced.PoseStorage;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpOfficial")
public class TeleOpNou extends LinearOpMode {
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        drive.setPoseEstimate(PoseStorage.currentPose);

        Pose2d startPose = new Pose2d(23.5, -72, 20.43);

        drive.setPoseEstimate(startPose);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {


            Pose2d poseEstimate = drive.getPoseEstimate();

//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x
//            ).rotated(-poseEstimate.getHeading());

            drive.update();

            double x = -gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;


            robot.setDrivePower(x, y, rx);

            if(!gamepad2.right_bumper && !robot.GhearaB) robot.GhearaB = true;
            if(gamepad2.right_bumper && robot.GhearaB) {
                if ( robot.GhearaValStanga  ==  robot.closedStanga  &&  robot.GhearaValDreapta  ==  robot.closedDreapta) {
                     robot.GhearaValStanga =   robot.openStanga;
                     robot.GhearaValDreapta =  robot.openDreapta;
                }
                else{
                    robot.GhearaValStanga =  robot.closedStanga;
                    robot.GhearaValDreapta =  robot.closedDreapta;}
                robot.GhearaB = false;

                robot.ServoStanga.setPosition( robot.GhearaValStanga);
                robot.ServoDreapta.setPosition( robot.GhearaValDreapta);
            }

            if(gamepad2.x==true) {
                robot.LiftDreapta.setPower(0.4);
                robot.LiftStanga.setPower(0.4);
            }
            else if(gamepad2.b==true) {
                robot.LiftDreapta.setPower(-0.2);
                robot.LiftStanga.setPower(-0.2);
            }
            else {
                robot.LiftDreapta.setPower(0.00);
                robot.LiftStanga.setPower(0.00);
            }


            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}