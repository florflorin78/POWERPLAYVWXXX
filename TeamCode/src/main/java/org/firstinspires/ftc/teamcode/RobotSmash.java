package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class RobotSmash {

    boolean toggleClaw = false;
    boolean buttonIsPressed = false;

    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;
    public DcMotorEx LiftDreapta;
    public DcMotorEx LiftStanga;
    Servo ServoStanga;
    Servo ServoDreapta;
    double closeClaw = 0.53;
    double openClaw = 0.37;

    int ground = 0;
    int low = 600;
    int medium  = 970;
    int high = 1300;

    public double pid;

    boolean manualControl=false;

    double kpUP = 0.008;
    double kpDOWN = 0.006;
    double ki = 0;
    double kdUP = 0.0001;
    double kdDOWN = 0;
    double ff = 0.2;
    int liftTarget = 0;

    PIDController pidController = new PIDController(kpUP, 0, kdUP);

    public RobotSmash(HardwareMap hardwareMap) {


        //region MotoareDrive
        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        //endregion
        //region MotoareLift

        LiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta");

        LiftStanga.setDirection(DcMotor.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotor.Direction.REVERSE);

        LiftStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LiftStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //endregion
        //region ServoGheara

        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

        ServoStanga.setDirection(Servo.Direction.REVERSE);
        ServoDreapta.setDirection(Servo.Direction.FORWARD);

        //endregion

        //LynxBulkCachingMode
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }

    public void ClawState(double pos){
        ServoStanga.setPosition(pos);
        ServoDreapta.setPosition(pos);
    }

    public void CloseClaw() {
        ClawState(closeClaw);
    }

    public void OpenClaw()
    {
        ClawState(openClaw);
    }

    public void DriveMovement(Gamepad gamepad) {

        double Forward = -gamepad.left_stick_y;
        double Strafe = gamepad.left_stick_x;
        double Turn = gamepad.right_stick_x;

//        Todo: Decomenteaza partea asta daca vrei sa ai toggle la viteza
        if(!gamepad.left_bumper){
            Strafe /= 2;
            Forward /= 2;
        }
        if(!gamepad.right_bumper){
            Turn /= 2;
        }

        double r = Math.hypot(Strafe, Forward);

        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        final double v1 = (r * Math.cos(robotAngle)) + Turn;
        final double v2 = (r * Math.sin(robotAngle)) - Turn;
        final double v3 = (r * Math.sin(robotAngle)) + Turn;
        final double v4 = (r * Math.cos(robotAngle)) - Turn;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);
    }

    public void LiftPID(Gamepad gamepad){

        if(gamepad.dpad_up) {setLiftTarget(high); manualControl=false;}
        else if(gamepad.dpad_left) {setLiftTarget(medium); manualControl=false;}
        else if(gamepad.dpad_down) {setLiftTarget(low); manualControl=false;}
        else if(gamepad.dpad_right) {setLiftTarget(ground); manualControl=false;}


        double manualPower = (gamepad.left_trigger-gamepad.right_trigger+ff)*0.5;

        if(gamepad.left_trigger > 0.1 || gamepad.right_trigger > 0.1)
            manualControl=true;

        if(gamepad.left_trigger > 0.9 || gamepad.right_trigger > 0.9)
            manualPower = (gamepad.left_trigger-gamepad.right_trigger+ff)*0.7;


//        pidController.setPID(kp, ki, kd);
        int armPos = LiftStanga.getCurrentPosition();
        double pid = pidController.calculate(armPos, liftTarget);

        double pidPower = pid + ff;

        if(manualControl){
        LiftDreapta.setPower(manualPower);
        LiftStanga.setPower(manualPower);}
        else
        {LiftDreapta.setPower(pidPower);
            LiftStanga.setPower(pidPower);}
    }

    public void LiftPID(){

//        pidController.setPID(kp, ki, kd);
        int armPos = LiftDreapta.getCurrentPosition();
        pid = pidController.calculate(armPos, liftTarget);

        double power = pid + ff;
        LiftDreapta.setPower(power);
        LiftStanga.setPower(power);

    }

    public void setLiftTarget(int pos){
        liftTarget = pos;
        if(liftTarget > getLiftRightPosition()) pidController.setPID(kpUP, 0, kdUP);
        else pidController.setPID(kpDOWN, 0, kdDOWN);
    }

    public void ClawManager(Gamepad gamepad){

        if(gamepad.x && !buttonIsPressed){
            buttonIsPressed = true;
            toggleClaw =! toggleClaw;
            if(toggleClaw) ClawState(closeClaw);
            else ClawState(openClaw);
        }
        else if(!gamepad.x) buttonIsPressed = false;

    }

//    public void LIFT_URCAT(double power, int distance)
//    {
//            LiftStanga.setTargetPosition(distance);
//            LiftDreapta.setTargetPosition(distance);
//
//            LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            runtime.reset();
//            LiftStanga.setPower(power);
//            LiftDreapta.setPower(power);
//
//            while(LiftStanga.isBusy() && LiftDreapta.isBusy())
//            {}
//            LiftStanga.setPower(0.01);
//            LiftDreapta.setPower(-0.01);
//
//            LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }
//
//    public void LIFT_COBORAT(double power, int distance)
//    {
//            LiftStanga.setTargetPosition(distance);
//            LiftDreapta.setTargetPosition(distance);
//
//            LiftStanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            LiftDreapta.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            runtime.reset();
//            LiftStanga.setPower(-power);
//            LiftDreapta.setPower(-power);
//
//            while(LiftStanga.isBusy() && LiftDreapta.isBusy())
//            {}
//            LiftStanga.setPower(0.01);
//            LiftDreapta.setPower(-0.01);
//
//            LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//    }

    public int getLiftLeftPosition() {return LiftStanga.getCurrentPosition();}

    public int getLiftRightPosition() {return LiftDreapta.getCurrentPosition();}

    public int getLiftTarget() {return liftTarget;}

    public double getLiftPower() {return LiftDreapta.getPower();}
}