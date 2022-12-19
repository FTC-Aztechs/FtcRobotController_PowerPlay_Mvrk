package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(name="Skystone camera example", group="Auto")
public class Skystone_OpenCvExample extends LinearOpMode {
    Ryk_Robot Powerslay =new Ryk_Robot();
    FtcDashboard mvrkDashboard = FtcDashboard.getInstance();


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
                telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                mvrkDashboard.startCameraStream(webcam,0);



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
