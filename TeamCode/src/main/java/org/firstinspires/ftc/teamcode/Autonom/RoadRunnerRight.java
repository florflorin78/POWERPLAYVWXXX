package org.firstinspires.ftc.teamcode.Autonom;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@Autonomous(name="RoadRunnerRight")
public class RoadRunnerRight extends LinearOpMode {

//    Robot robot = new Robot(hardwareMap);

    public DcMotorEx LiftDreapta, LiftStanga;

    public static double START_TO_FORWARD = 50;

    public static double START_TO_HIGH_PRELOAD_X = 10.5;
    public static double START_TO_HIGH_PRELOAD_Y = -12;
    public static double START_TO_HIGH_PRELOAD_TG = 45;

    public static double START_X1 = 26;
    public static double START_Y1 = -72;
    public static double START_HEADING = 20.43;


    Trajectory START_FORWARD;
    Trajectory START_FORWARD2;

    Trajectory START_TO_HIGH_PRELOAD;        //start cycle

    Trajectory HIGH_PRELOAD_TO_STACK_CONE1; //cone_cycle_1
    Trajectory STACK_CONE1_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_STACK_CONE2;    //cone_cycle_2
    Trajectory STACK_CONE2_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_STACK_CONE3;    //cone_cycle_3
    Trajectory STACK_CONE3_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_STACK_CONE4;    //cone_cycle_4
    Trajectory STACK_CONE4_TO_HIGH_CONE;    //

    Trajectory HIGH_CONE_TO_PARK1;          // park1
    Trajectory HIGH_CONE_TO_PARK2;          // park2
    Trajectory HIGH_CONE_TO_PARK3;          // park3


    @Override
public void runOpMode() throws InterruptedException {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    LiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
    LiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta");

    LiftStanga.setDirection(DcMotor.Direction.FORWARD);
    LiftDreapta.setDirection(DcMotor.Direction.REVERSE);

    LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//    robot.GhearaInchide();

    Pose2d startPose = new Pose2d(START_X1, START_Y1, START_HEADING);

    ElapsedTime timer = new ElapsedTime();

    drive.setPoseEstimate(startPose);


    START_FORWARD = drive.trajectoryBuilder(startPose)
                      .strafeRight(2)
                      .build();

    START_FORWARD2 = drive.trajectoryBuilder(START_FORWARD.end())
                .forward(START_TO_FORWARD)
                .build();

    START_TO_HIGH_PRELOAD =  drive.trajectoryBuilder(START_FORWARD.end())
                      .addDisplacementMarker(pathLength -> pathLength * 0.1, () -> {
                         LIFT_URCAT(0.8, 1300);
                          })

                      .build();



    HIGH_PRELOAD_TO_STACK_CONE1 = drive.trajectoryBuilder(START_TO_HIGH_PRELOAD.end())
                    .strafeRight(2)
                    .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                    .build();
    STACK_CONE1_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_PRELOAD_TO_STACK_CONE1.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_STACK_CONE2 = drive.trajectoryBuilder(STACK_CONE1_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .build();
    STACK_CONE2_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_CONE_TO_STACK_CONE2.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_STACK_CONE3 = drive.trajectoryBuilder(STACK_CONE2_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .build();
    STACK_CONE3_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_CONE_TO_STACK_CONE3.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_STACK_CONE4 = drive.trajectoryBuilder(STACK_CONE3_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                    .build();

    STACK_CONE4_TO_HIGH_CONE = drive.trajectoryBuilder(HIGH_CONE_TO_STACK_CONE4.end())
                    .lineToSplineHeading(new Pose2d(23.5, -7, Math.toRadians(90)))
                    .build();




    HIGH_CONE_TO_PARK1 = drive.trajectoryBuilder(STACK_CONE4_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(11.5, -11.5, Math.toRadians(90)))
                    .build();

    HIGH_CONE_TO_PARK2 = drive.trajectoryBuilder(STACK_CONE4_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(35.5, -11.5, Math.toRadians(90)))
                    .build();


    HIGH_CONE_TO_PARK3 = drive.trajectoryBuilder(STACK_CONE4_TO_HIGH_CONE.end())
                    .lineToSplineHeading(new Pose2d(58.5, -11.5, Math.toRadians(90)))
                    .build();


        waitForStart();

        drive.GhearaInchide();
        sleep(1000);
        drive.followTrajectory(START_FORWARD);
        drive.followTrajectory(START_TO_HIGH_PRELOAD);
        sleep(1000);
        drive.GhearaDeschide();


    PoseStorage.currentPose = drive.getPoseEstimate();
}
    public void LIFT_URCAT(double power, int distance)
    {
        if(opModeIsActive()) {
            LiftStanga.setTargetPosition(distance);
            LiftDreapta.setTargetPosition(distance);

            LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runtime.reset();
            LiftStanga.setPower(power);
            LiftDreapta.setPower(power);

            while(opModeIsActive() && LiftStanga.isBusy() && LiftDreapta.isBusy())
            {}
            LiftStanga.setPower(0.01);
            LiftDreapta.setPower(-0.01);

            LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void LIFT_COBORAT(double power, int distance)
    {
        if(opModeIsActive()) {
            LiftStanga.setTargetPosition(distance);
            LiftDreapta.setTargetPosition(distance);

            LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            runtime.reset();
            LiftStanga.setPower(-power);
            LiftDreapta.setPower(-power);

            while(opModeIsActive() && LiftStanga.isBusy() && LiftDreapta.isBusy())
            {}
            LiftStanga.setPower(0.01);
            LiftDreapta.setPower(-0.01);

            LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }


}
