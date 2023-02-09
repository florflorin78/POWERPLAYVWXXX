package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.camera.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "Test camera fratic")
public class VisionTest extends LinearOpMode {

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;



    // Name of the Webcam to be set in the config
    String webcam= "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcam), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("CAZ: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();

        if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER)
        {
            telemetry.addData("Status", "CAZU 2");
        }
        else if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.LEFT)
        {
            telemetry.addData("Status", "CAZU 1");
        }
        else if(sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT)
        {
            telemetry.addData("Status", "CAZU 3");
        }
    }
}
