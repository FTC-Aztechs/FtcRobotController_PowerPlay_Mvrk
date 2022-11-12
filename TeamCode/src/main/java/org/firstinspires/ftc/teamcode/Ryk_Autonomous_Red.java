package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.GroundJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.MidJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_drop_wait;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(name="Red Corner",group = "Autonomous")
public class Ryk_Autonomous_Red extends LinearOpMode {

    enum RykAllianceField {
        RED,
        BLUE
    }

    Ryk_Robot Mavryk = new Ryk_Robot();

    private static FtcDashboard rykRobot;
    OpenCvWebcam Sauron = null;
    examplePipeline pipeline;

    private static int iTeleCt = 1;


    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";


    // Field Dimensions
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    private OpenGLMatrix mrvLastLocation = new OpenGLMatrix();

    public TrajectorySequence Position1;
    public TrajectorySequence Position2;
    public TrajectorySequence Position3;
    public TrajectorySequence Terminal;

    ElapsedTime timer = new ElapsedTime(MILLISECONDS);



    public TrajectorySequence Park;

    public static int col = 1;

    double SlideHigh = (Mavryk.Slide_High_Revs * Mavryk.Slide_Ticks_Per_Rev);
    double SlidePickup = (Mavryk.Slide_Min_Pickup_Revs * Mavryk.Slide_Ticks_Per_Rev);
    double SlideRest = (Mavryk.Slide_rest * Mavryk.Slide_Ticks_Per_Rev);


    public static class examplePipeline extends OpenCvPipeline {

        public enum Color
        {
            RED,
            GREEN,
            BLUE,
            NONE
        }

        Mat HSV = new Mat();
        Mat picCrop;
        double Hue;
        Mat outPut = new Mat();
        Scalar rectColor = new Scalar(0.0,0.0,0.0);
        int[] red = {135, 180, 0, 10};
        int[] green = {70, 90};
        int[] blue = {100, 120};

        private volatile examplePipeline.Color color = examplePipeline.Color.NONE;

        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
            Rect mainRect = new Rect(850, 620, 100, 200);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, mainRect,rectColor,2);

            picCrop = HSV.submat(mainRect);
            Scalar Average = Core.mean(picCrop);

            Core.extractChannel(picCrop, picCrop, 1);
            Hue = Average.val[0];

            if ( (red[0] < Hue && Hue < red[1]) || (red[2] < Hue && Hue < red[3])) {
                color = Color.RED;
            } else if ( green[0] < Hue && Hue < green[1]) {
                color = Color.GREEN;
            } else if ( blue[0] < Hue && Hue < blue[1]) {
                color = Color.BLUE;
            } else {
                color = Color.NONE;
            }

            return(outPut);
        }
        public double getAnalysis()
        {
            return Hue;
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        Mavryk.init(hardwareMap);

        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);

        // init Dashboard
        rykRobot = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine(String.format("%d. Mavryk Initialized!", iTeleCt++));

        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));

        Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Mavryk.Claw_Close_Pos);
        sleep(3000);

        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        //init
        //acquireCamera();

        // During Init:
            // 1. Use Vuforia to Calculate initial pose estimate from viewables
            // 2. Use OpenCV to prepare to recognize the signal
        // Upon Play:
            // 3. Determine coordinates for parking position based on signal
            // 4. Move to drop off the pre-loaded cone on the middle tall pole
            // 5. Turn to face the cone pick-up stack
            // 6. Drive to the programmed cone pick-up position
            // 7. Use Vuforia to re-calculate pose estimate
            // 8. Spline to this position - from where we are
            // 9. Cycle cone <- loop?
            // 10. After last cone cycled, drive to park position.

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Sauron = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        examplePipeline pipeline = new examplePipeline();
        Sauron.setPipeline(pipeline);
        telemetry.addData("Status: ", "Pipeline set ...");

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        Sauron.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                Sauron.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Status: ", "Sauron Streaming ...");
                rykRobot.startCameraStream(Sauron, 0);
            }
            public void onError(int errorCode) {
                return;
            }
         });

        sleep(7000);
        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // 1. Calculate Parking Position
        telemetry.addLine(String.format("Detected Hue value: %f", pipeline.Hue));
//        telemetry.update();
        switch(pipeline.color)
        {
            case RED:
                telemetry.addData("Color: ", "Red");
                col = 1;
                break;
            case GREEN:
                telemetry.addData("Color: ", "Green");
                col = 2;
                break;
            case BLUE:
                telemetry.addData("Color: ", "Blue");
                col = 3;
                break;
            case NONE:
                telemetry.addData("Color: ", "Not detected");
                col = 1;
                break;
        }

        telemetry.update();

        initMotorsAndServos(true);
        buildParkTrajectories_position(col);

        // 2. Move to dropoff pole

        // 3. Extend Slides + dropoff
        /*int iLinacDropoffPos = (int) (Mavryk.Linac_Dropoff_Revs * Mavryk.Linac_Ticks_Per_Rev);
        double dLinacPower = 1;
        Mavryk.setPower(Ryk_Robot.RykMotors.LIN_AC, dLinacPower);
        while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.LIN_AC) < iLinacDropoffPos ) {
            idle();
        }
        Mavryk.setPower(Ryk_Robot.RykMotors.LIN_AC, 0);*/

        // 4. Drive to cone stack

        // 5. Pickup cone
        Pose2d startPose = Ryk_Robot.Red_Start;
        Mavryk.mecanumDrive.setPoseEstimate(startPose);

        // 6. Drive back to pole

        // 7. repeat steps 3-6 for cycling

        // 8. park in spot

        telemetry.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
        trajectoryTimer.reset();

        Mavryk.mecanumDrive.followTrajectorySequence(Park);

        telemetry.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        telemetry.update();
    }


    void buildParkTrajectories_position(int col) {

        telemetry.addLine(String.format("%d. buildWarehouseTrajectories_position", iTeleCt++));

        switch (col) {
            case 1:
                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Ryk_Robot.Red_Start)
                        .lineToLinearHeading(Ryk_Robot.Red_Dropoff_Dodge)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
                                idle();
                            }
                        })
                        .waitSeconds(2)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
                                idle();
                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                                idle();
                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Red_Park_Pos1)
                        .build();
                break;
            case 2:
                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Ryk_Robot.Red_Start)
                        .lineToLinearHeading(Ryk_Robot.Red_Dropoff_Dodge)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
                                idle();
                            }
                        })
                        .waitSeconds(2)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
                                idle();
                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                                idle();
                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Red_Park_Pos2)
                        .build();
                break;
            case 3:
                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Ryk_Robot.Red_Start)
                        .lineToLinearHeading(Ryk_Robot.Red_Dropoff_Dodge)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
                                idle();
                            }
                        })
                        .waitSeconds(2)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
                                idle();
                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Red_DropOff_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                                idle();
                            }
                        })
//                        .lineToLinearHeading(Ryk_Robot.Red_Park_Pos3)
                        .build();
                break;
        }

        telemetry.addLine(String.format("%d. Trajectory Duration after build: %.3f", iTeleCt++, Park.duration()));
        return;

    }

    void initMotorsAndServos(boolean run_to_position)
    {
        // Reset Slides - current position becomes 0
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, STOP_AND_RESET_ENCODER);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_WITHOUT_ENCODER);

    }


    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void acquireCamera()
    {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Sauron = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        examplePipeline pipeline = new examplePipeline();
        Sauron.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.

        Sauron.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                Sauron.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
                rykRobot.startCameraStream(Sauron, 0);
            }

            public void onError(int errorCode) {

                return;

            }



        });


        return;
    }


}


