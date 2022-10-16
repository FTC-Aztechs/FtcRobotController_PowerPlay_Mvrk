package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous
public class Beacon_Testing extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");
        FtcDashboard rykDashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened(){
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                rykDashboard.startCameraStream(webcam1, 0);
            }

            public void onError(int errorCode){

            }
        });
    }

    @Override
    public void loop() {

    }

    public class examplePipeline extends OpenCvPipeline {
        Mat HSV = new Mat();
        Mat picCrop;
        double Hue;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(0.0,0.0,0.0);
       int[] red = {160, 180, 0, 10};
       int[] green = {50, 65};
       int[] blue = {70, 90};


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");


            Rect mainRect = new Rect(120, 90, 79, 59);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, mainRect,rectColor,2);

            picCrop = HSV.submat(mainRect);

            Scalar Average = Core.mean(picCrop);

            Core.extractChannel(picCrop, picCrop, 1);

            Hue = Average.val[0];

            if ( (red[0] < Hue && Hue < red[1]) || (red[2] < Hue && Hue < red[3])) {
                telemetry.addLine("Red");
            } else if ( green[0] < Hue && Hue < green[1]) {
                telemetry.addLine("Green");
            } else if ( blue[0] < Hue && Hue < blue[1]) {
                telemetry.addLine("Blue");
            } else {
                telemetry.addLine("Color Not detected");
            }

            telemetry.addData("Hue", "%f", Hue);


            return(outPut);
        };
    };

}
