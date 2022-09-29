package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="Skystone camera example", group="Auto")
@Disabled
public class Skystone_OpenCvExample extends LinearOpMode {
    Pp_Robot Powerslay =new Pp_Robot();

    @Override
    public void runOpMode()throws InterruptedException {
        Powerslay.init(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Camera failed:", "You did something wrong!!!");
            }
        });






        waitForStart();

        OpenCVTesting detector = new OpenCVTesting(telemetry);
        webcam.setPipeline(detector);

        switch(detector.getLocation()){
            case LEFT:
                //stuff
                break;
            case RIGHT:
                //stuff
                break;
            case NOT_FOUND:
                //stuff
        }

        webcam.stopStreaming();
    }



}
