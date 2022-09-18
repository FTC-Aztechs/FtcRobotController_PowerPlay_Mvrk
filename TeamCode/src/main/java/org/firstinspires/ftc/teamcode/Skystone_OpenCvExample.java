package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Skystone camera example", group="Auto")
public class Skystone_OpenCvExample extends LinearOpMode {
    OpenCvCamera Sauron;
    @Override
    public void runOpMode()throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameaMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        Sauron = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        OpenCVTesting detector = new OpenCVTesting(telemetry);
        Sauron.setPipeline(detector);
        Sauron.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
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
        Sauron.stopStreaming();
    }



}
