package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront, LiftDreapta, LiftStanga;
    public Servo ServoStanga, ServoDreapta;

    public double denominator;

    public boolean GhearaB = false;
    public double  closedStanga = 0.48,openStanga = 0.63, closedDreapta = 0.48, openDreapta = 0.63, GhearaValStanga = 0.53, GhearaValDreapta = 0.53;

    public Robot(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "LeftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "LeftBack");
        rightRear = hardwareMap.get(DcMotorEx.class, "RightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "RightFront");

        LiftStanga = hardwareMap.get(DcMotorEx.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotorEx.class, "LiftDreapta");

        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        LiftStanga.setDirection(DcMotor.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotor.Direction.REVERSE);

        ServoStanga.setDirection(Servo.Direction.REVERSE);
        ServoDreapta.setDirection(Servo.Direction.FORWARD);

        LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void GhearaDeschide() {
        ServoStanga.setPosition(openStanga);
        ServoDreapta.setPosition(openDreapta);
    }

    public void GhearaInchide()
    {
        ServoStanga.setPosition(closedStanga);
        ServoDreapta.setPosition(closedDreapta);
    }

    public void setDrivePower(double x, double y, double rx) {

//        if( gamepad1.left_bumper == true )  { denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 4);}
//
//        if( gamepad1.right_bumper == true ) { denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.5);}

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);

        double LeftFrontPower = (y - x + rx)/denominator;
        double LeftBackPower = (y + x + rx)/denominator;
        double RightFrontPower = (y + x - rx)/denominator;
        double RightBackPower = (y - x - rx)/denominator;

        leftFront.setPower(LeftFrontPower);   // +
        leftRear.setPower(LeftBackPower);     // -
        rightFront.setPower(RightFrontPower); // -
        rightRear.setPower(RightBackPower);   // +
    }

}