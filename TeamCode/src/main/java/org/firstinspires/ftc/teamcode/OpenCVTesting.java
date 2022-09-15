package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class OpenCVTesting extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    static final Rect ROI = new Rect(
            new Point(60, 35),
            new Point(120, 75));

    static double PERCENT_COLOR_THRESHOLD = 0.4;
    public OpenCVTesting(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);
        Mat wholeimage = mat.submat(ROI);

        double wholeimageValue = Core.sumElems(wholeimage).val[0] / ROI.area() / 255;

        wholeimage.release();

        telemetry.addData("Image raw value", (int) Core.sumElems(wholeimage).val[0]);
        telemetry.addData("Image percentage", Math.round(wholeimageValue * 100) + "%");

        boolean stone = wholeimageValue > PERCENT_COLOR_THRESHOLD;

        if (stone) {
            //not found
        }
    }
}
