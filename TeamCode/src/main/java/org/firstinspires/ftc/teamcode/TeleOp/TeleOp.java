package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LiftStanga = null;
    private DcMotor LiftDreapta = null;

     Servo ServoStanga = null;
     Servo ServoDreapta = null;
    boolean GhearaB = false;
    double closedStanga = 0.53;
    double openStanga = 0.37;



    double closedDreapta = 0.53;
    double openDreapta = 0.37;
    double GhearaValStanga = 0.53;
    double GhearaValDreapta = 0.53;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        LeftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        RightFront = hardwareMap.get(DcMotor.class, "RightFront");
        RightBack = hardwareMap.get(DcMotor.class, "RightBack");
        LiftStanga = hardwareMap.get(DcMotor.class, "LiftStanga");
        LiftDreapta = hardwareMap.get(DcMotor.class, "LiftDreapta");

        ServoStanga = hardwareMap.get(Servo.class, "ServoStanga");
        ServoDreapta = hardwareMap.get(Servo.class, "ServoDreapta");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        LiftStanga.setDirection(DcMotor.Direction.FORWARD);
        LiftDreapta.setDirection(DcMotor.Direction.REVERSE);
        ServoStanga.setDirection(Servo.Direction.REVERSE);
        ServoDreapta.setDirection(Servo.Direction.FORWARD);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        LiftStanga.setPower(0);
        LiftDreapta.setPower(0);

        ServoStanga.setPosition(closedStanga);
        ServoDreapta.setPosition(closedDreapta);

        LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LiftStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LiftDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {}
    @Override
    public void start() {
        runtime.reset();
    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
        if(gamepad1.left_bumper == true){ denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 4);}
        if(gamepad1.right_bumper == true) { denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 0.5);}
        double LeftFrontPower = (y - x + rx)/denominator;
        double LeftBackPower = (y + x + rx)/denominator;
        double RightFrontPower = (y + x - rx)/denominator;
        double RightBackPower = (y - x - rx)/denominator;

        LeftFront.setPower(LeftFrontPower);   // +
        LeftBack.setPower(LeftBackPower);     // -
        RightFront.setPower(RightFrontPower); // -
        RightBack.setPower(RightBackPower);   // +

        if(gamepad2.right_bumper == false && GhearaB == false) GhearaB = true;
        if(gamepad2.right_bumper == true && GhearaB == true) {
            if (GhearaValStanga == closedStanga && GhearaValDreapta == closedDreapta) {
                GhearaValStanga = openStanga;
                GhearaValDreapta = openDreapta;
            }
            else{
                GhearaValStanga = closedStanga;
                GhearaValDreapta = closedDreapta;}
                GhearaB = false;

            ServoStanga.setPosition(GhearaValStanga);
            ServoDreapta.setPosition(GhearaValDreapta);
            //GhearaVal =
        }
        telemetry.addData("ServoStanga", ServoStanga.getPosition());
        telemetry.addData("ServoDreapta", ServoDreapta.getPosition());
        telemetry.update();
/*        if(gamepad2.y ==true){
            ServoStanga.setPosition(openStanga);
            ServoDreapta.setPosition(openDreapta);
        }
        if(gamepad2.a == true)
        {ServoStanga.setPosition(closedStanga);
            ServoDreapta.setPosition(closedDreapta);}*/


        if(gamepad2.x==true) {
            LiftDreapta.setPower(0.4);
            LiftStanga.setPower(0.4);
        }
        else if(gamepad2.b==true) {
            LiftDreapta.setPower(-0.2);
            LiftStanga.setPower(-0.2);
        }
        else if(gamepad2.a==true) {
            LiftDreapta.setPower(0.02);
            LiftStanga.setPower(0.02);
        }
        else {
            LiftDreapta.setPower(0.00);
            LiftStanga.setPower(0.00);
        }


}

    @Override
    public void stop() {

    }
}


