package org.firstinspires.ftc.teamcode;

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

@Autonomous
public class Beacon_Testing extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened(){
                webcam1.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode){

            }
        });
    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat picCrop;
        Mat redCrop;
        Mat blueCrop;
        double redAvgfin;
        double blueAvgfin;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(255.0,0.0,0.0);
        double maxValue = 1.0;


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");


            Rect mainRect = new Rect(80, 60, 159, 119);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, mainRect,rectColor,2);

            picCrop = YCbCr.submat(mainRect);

            Scalar Average = Core.mean(picCrop);


            redAvgfin = Average.val[0];
            blueAvgfin = Average.val[2];

//            if (redAvgfin > GreenAvgfin) {
//                telemetry.addLine("Left");
//            } else {
//                telemetry.addLine("Right");
//            }

            telemetry.addData("red", "%f", redAvgfin);
            telemetry.addData("blue", "%f", blueAvgfin);



            return(outPut);
        };
    };

}
