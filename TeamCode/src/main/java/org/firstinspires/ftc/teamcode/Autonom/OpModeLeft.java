package org.firstinspires.ftc.teamcode.Autonom;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.camera.Detection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

@Autonomous(name="1AUTOLEFT")
///@Disabled
public class OpModeLeft extends LinearOpMode  {

    static final double FEET_PER_METER = 3.28084;

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor LeftFront = null;
    public DcMotor LeftBack = null;
    public DcMotor RightFront = null;
    public DcMotor RightBack = null;
    public DcMotor LiftStanga = null;
    public DcMotor LiftDreapta = null;
    public Servo ServoStanga = null;
    public Servo ServoDreapta = null;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    int Left = 1;
    int Middle = 2;
    int Right = 3;

    int autoCase = 2;

    boolean GhearaB= false;

    double closeClaw = 0.37;
    double openClaw = 0.63; //0.53

    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    OpenCvCamera camera;

    AprilTagDetection tagOfInterest = null;

    String webcam= "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {

        LeftFront =  hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack  =  hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack =  hardwareMap.get(DcMotor.class, "RightBack");
        LiftStanga = hardwareMap.get(DcMotor.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotor.class, "LiftDreapta");
        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        LiftStanga.setPower(0);
        LiftDreapta.setPower(0);

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LiftStanga.setDirection(DcMotor.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotor.Direction.REVERSE);
        ServoStanga.setDirection(Servo.Direction.FORWARD);
        ServoDreapta.setDirection(Servo.Direction.REVERSE);

        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ServoStanga.setPosition(closeClaw);
        ServoDreapta.setPosition(closeClaw);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    autoCase = tag.id;
                    telemetry.addData("Id", autoCase);
                    break;
                }

            }
            sleep(20);
            telemetry.update();
        }


        telemetry.addData("Status", "S-a initializat fratic");
        telemetry.update();


        waitForStart();

        /*
    YELLOW  = Parking Left
    CYAN    = Parking Middle
    MAGENTA = Parking Right
     */

        //st 550
        //dr 550
        //dr 1200

        GhearaInchide();
        LIFTURCAT(0.5, 250);
        StangaInainte45Dist(0.6, 480);
        InainteDist(0.5, 2050);
        DreaptaDist(0.5, 580);
        LIFTURCAT(0.55, 1480);
        InainteDist(0.5, 140);  //160
        sleep(2000);

        GhearaDeschide();
        InapoiDist(0.5, 200);


        if(autoCase == 1) {
            telemetry.addData("Status", "CAZ 1 - LEFT");
            StangaDist(0.5, 1700);
        }

        else if(autoCase == 2) {
            telemetry.addData("Status", "CAZ 2");
            StangaDist(0.5, 550);
        }

        else if(autoCase == 3) {
            telemetry.addData("Status", "CAZ 3");
            DreaptaDist(0.5, 550);
        }


//        LIFTURCAT(0.5, 1000);
//        sleep(1000);
//        LIFTCOBORAT(1, 500);


//        if(autoCase == 1) {
//            telemetry.addData("Status", "CAZ 1 - LEFT");
//
//            StangaDist(0.5, 550);
//        }
//
//        else if(autoCase == 2) {
//            telemetry.addData("Status", "CAZ 2");
//
//            DreaptaDist(0.5, 550);
//        }
//
//        else if(autoCase == 3) {
//            telemetry.addData("Status", "CAZ 3");
//            DreaptaDist(0.5, 1400);
//        }


    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void LIFTURCAT(double power, int distance)
    {
        int LiftTargetStanga,LiftTargetDreapta;

        if(opModeIsActive()) {

            LiftTargetStanga = LiftStanga.getCurrentPosition();
            LiftTargetDreapta = LiftDreapta.getCurrentPosition();

            LiftStanga.setTargetPosition(distance);
            LiftDreapta.setTargetPosition(distance);

            LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

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

    public void LIFTCOBORAT(double power, int distance)
    {
        int LiftTargetStanga,LiftTargetDreapta;

        if(opModeIsActive()) {

            LiftTargetStanga = LiftStanga.getCurrentPosition();
            LiftTargetDreapta = LiftDreapta.getCurrentPosition();

            LiftStanga.setTargetPosition(-distance);
            LiftDreapta.setTargetPosition(-distance);

            LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

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




    public void GhearaDeschide()
    {
        ServoStanga.setPosition(openClaw);
        ServoDreapta.setPosition(openClaw);
        Stop();
    }

    public void GhearaInchide()
    {
        ServoStanga.setPosition(closeClaw);
        ServoDreapta.setPosition(closeClaw);
        Stop();
    }

    public void InvarteDreaptaDist(double power, int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(distance);
        RightFront.setTargetPosition(-distance);
        LeftBack.setTargetPosition(distance);
        RightBack.setTargetPosition(-distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        InvarteDreapta(power);

        while(LeftFront.isBusy() && RightBack.isBusy() && RightFront.isBusy() && LeftBack.isBusy() )
        {}

        Stop();

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }
    public void InvarteStangaDist(double power, int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(-distance);
        RightFront.setTargetPosition(distance);
        LeftBack.setTargetPosition(-distance);
        RightBack.setTargetPosition(-distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        InvarteStanga(power);

        while(LeftFront.isBusy() && RightBack.isBusy() && RightFront.isBusy() && LeftBack.isBusy() )
        {}
        Stop();
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void DreaptaInapoi45Dist(double power, int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(-distance);
        RightBack.setTargetPosition(-distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DreaptaInainte45(-power);

        while(LeftFront.isBusy() && RightBack.isBusy())
        {}

//        Stop();

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void StangaInapoi45Dist(double power, int distance)
    {
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        RightFront.setTargetPosition(-distance);
        LeftBack.setTargetPosition(-distance);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        StangaInainte45(-power);

        while(RightFront.isBusy() && LeftBack.isBusy())
        {}

//        Stop();

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }


    public void StangaInainte45Dist(double power, int distance)
    {
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        RightFront.setTargetPosition(distance);
        LeftBack.setTargetPosition(distance);

        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        StangaInainte45(power);

        while(RightFront.isBusy() && LeftBack.isBusy())
        {}

//        Stop();

        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void DreaptaInainte45Dist(double power, int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(distance);
        RightBack.setTargetPosition(distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        DreaptaInainte45(power);

        while(LeftFront.isBusy() && RightBack.isBusy())
        {}

        Stop();

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void DreaptaDist(double power, int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(distance);
        RightFront.setTargetPosition(-distance);
        LeftBack.setTargetPosition(-distance);
        RightBack.setTargetPosition(distance);



        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Dreapta(power);

        while(LeftFront.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && RightBack.isBusy())
        {}

        Stop();

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

    }

    public void StangaDist(double power, int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(-distance);
        RightFront.setTargetPosition(distance);
        LeftBack.setTargetPosition(distance);
        RightBack.setTargetPosition(-distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Stanga(power);

        while(LeftFront.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && RightBack.isBusy())
        {}

        Stop();

        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void InainteDist(double power,int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(distance);
        RightFront.setTargetPosition(distance);
        LeftBack.setTargetPosition(distance);
        RightBack.setTargetPosition(distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Inainte(power);

        while(LeftFront.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && RightBack.isBusy())
        {}
        Stop();
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void InapoiDist(double power,int distance)
    {
        LeftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        LeftFront.setTargetPosition(-distance);
        RightFront.setTargetPosition(-distance);
        LeftBack.setTargetPosition(-distance);
        RightBack.setTargetPosition(-distance);

        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Inapoi(power);

        while(LeftFront.isBusy() && RightFront.isBusy() && LeftBack.isBusy() && RightBack.isBusy())
        {}
        Stop();
        LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
    }

    public void Inainte(double power)
    {
        LeftFront.setPower(power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(power);
    }

    public void Inapoi(double power)
    {
        LeftFront.setPower(-power);
        RightFront.setPower(-power);
        LeftBack.setPower(-power);
        RightBack.setPower(-power);
    }

    public void Stanga(double power)
    {
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftBack.setPower(power);
        RightBack.setPower(-power);
    }

    public void Dreapta(double power)
    {
        LeftFront.setPower(power);
        RightFront.setPower(-power);
        LeftBack.setPower(-power);
        RightBack.setPower(power);
    }

    public void DreaptaInainte45(double power)
    {
        LeftFront.setPower(power);
        RightBack.setPower(power);
    }

    public void StangaInainte45(double power)
    {
        RightFront.setPower(power);
        LeftBack.setPower(power);
    }
    public void InvarteStanga(double power)
    {
        LeftFront.setPower(-power);
        RightFront.setPower(power);
        LeftBack.setPower(-power);
        RightBack.setPower(power);
    }
    public void InvarteDreapta(double power)
    {
        LeftFront.setPower(power);
        RightFront.setPower(-power);
        LeftBack.setPower(power);
        RightBack.setPower(-power);
    }
    public void LiftUrcat(double power)
    {
        LiftStanga.setPower(power);
        LiftDreapta.setPower(power);
    }
    public void LiftCoborat(double power)
    {
        LiftStanga.setPower(-power);
        LiftDreapta.setPower(-power);
    }
    public void Stop()
    {
        LeftFront.setPower(0);
        RightFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
        sleep(1000);
    }


    public void AsteaptaVirtual() // pentru eroarea cu Motorola
    {
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.update();
        }
    }
}

