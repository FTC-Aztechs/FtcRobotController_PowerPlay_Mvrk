package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.BottomCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.BottomMidCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Blue_Preload_DropOff;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.TopMidCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.TopCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.GroundJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_drop_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_move_wait;

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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(name="Blue Corner",group = "Autonomous")
public class Ryk_Autonomous_Blue extends LinearOpMode {

    enum RykAllianceField {
        RED,
        BLUE
    }

    Ryk_Robot Mavryk = new Ryk_Robot();

    private static FtcDashboard rykRobot;
    OpenCvWebcam Sauron = null;
    AprilTagDetectionPipeline pipeline;

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

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.04;

    // Tag ID 2,9,16 from 36h11 family
    int LEFT = 2;
    int MIDDLE = 9;
    int RIGHT = 20;

    public AprilTagDetection tagOfInterest = null;

    ElapsedTime timer = new ElapsedTime(MILLISECONDS);



    public TrajectorySequence Park;

    public static int pos = 1;

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
        sleep(1000);

        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Sauron");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Sauron = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        telemetry.addData("Status: ", "camera created  ...");
        telemetry.update();

        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        telemetry.addData("Status: ", "pipeline created  ...");
        telemetry.update();

        Sauron.setPipeline(pipeline);
        telemetry.addData("Status: ", "Pipeline set ...");
        telemetry.update();

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        Sauron.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                Sauron.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Status: ", "Sauron Streaming ...");
                rykRobot.startCameraStream(Sauron, 0);
            }
            public void onError(int errorCode) {
                return;
            }
        });

        telemetry.addData("Status: ", "Starting April Tag detection");
        telemetry.update();

        // while waiting for the game to start, search for april tags
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT ||  tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        //once program starts
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


        // 1. Calculate Parking Position

        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            telemetry.addLine("Going to Position 1");
            pos = 1;
        } else if(tagOfInterest.id == MIDDLE){
            telemetry.addLine("Going to Position 2");
            pos = 2;
        }else{
            telemetry.addLine("Going to Position 3");
            pos = 3;
        }

        telemetry.update();

        initMotorsAndServos(true);
        buildParkTrajectories_position(pos);

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
        Pose2d startPose = Ryk_Robot.Blue_Start;
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
                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Ryk_Robot.Blue_Start)
                        //preload
                        .lineToLinearHeading(Ryk_Robot.Blue_Offset)
                        .lineToLinearHeading(Ryk_Robot.Blue_Dropoff_Dodge)
                        .waitSeconds(auto_move_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                        })
                        .waitSeconds(auto_move_wait)
                        .turn(Math.toRadians(37))
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .turn(Math.toRadians(-90))
                        .lineToLinearHeading(Ryk_Robot.Blue_Park_Pos3)
                        .waitSeconds(auto_move_wait)
//                        //pickup cone 5
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopCone);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Picking up Top Cone");
//                                telemetry.update();
//                                idle();
//                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
                        .addTemporalMarker(() ->{
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
                        })
                        .waitSeconds(auto_move_wait)
                        .addTemporalMarker(()->{
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Lift up cone");
//                                telemetry.update();
//                                idle();
//                            }
                            timer.reset();
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
                        .waitSeconds(auto_move_wait)
                        .turn(Math.toRadians(130))
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                        })
                        .waitSeconds(auto_move_wait)
                        .lineToLinearHeading(Blue_Preload_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
//                        //cone 4
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopMidCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Top Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 3
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking Up Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 2
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomMidCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Bottom Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 1
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking Up Bottom Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //park
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Lowering to rest position.");
//                                telemetry.update();
//                                idle();
//                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Park_Pos1)
                        .build();
                break;
            case 2:
                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Ryk_Robot.Blue_Start)
                        //preload
                        .lineToLinearHeading(Ryk_Robot.Blue_Offset)
                        .lineToLinearHeading(Ryk_Robot.Blue_Dropoff_Dodge)
                        .waitSeconds(auto_move_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                        })
                        .waitSeconds(auto_move_wait)
                        .turn(Math.toRadians(37))
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .turn(Math.toRadians(-90))
                        .lineToLinearHeading(Ryk_Robot.Blue_Park_Pos3)
                        .waitSeconds(auto_move_wait)
//                        //pickup cone 5
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopCone);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Picking up Top Cone");
//                                telemetry.update();
//                                idle();
//                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
                        .addTemporalMarker(() ->{
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
                        })
                        .waitSeconds(auto_move_wait)
                        .addTemporalMarker(()->{
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Lift up cone");
//                                telemetry.update();
//                                idle();
//                            }
                            timer.reset();
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
                        .waitSeconds(auto_move_wait)
                        .turn(Math.toRadians(130))
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                        })
                        .waitSeconds(auto_move_wait)
                        .lineToLinearHeading(Blue_Preload_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
//                        //cone 4
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopMidCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Top Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 3
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking Up Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 2
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomMidCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Bottom Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 1
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking Up Bottom Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
                        //park
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Lowering to rest position.");
//                                telemetry.update();
//                                idle();
//                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Park_Pos2)
                        .build();

                break;
            case 3:
                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Ryk_Robot.Blue_Start)
                        //preload
                        .lineToLinearHeading(Ryk_Robot.Blue_Offset)
                        .lineToLinearHeading(Ryk_Robot.Blue_Dropoff_Dodge)
                        .waitSeconds(auto_move_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                        })
                        .waitSeconds(auto_move_wait)
                        .turn(Math.toRadians(37))
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .turn(Math.toRadians(-90))
                        .lineToLinearHeading(Ryk_Robot.Blue_Park_Pos3)
                        .waitSeconds(auto_move_wait)
//                        //pickup cone 5
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopCone);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Picking up Top Cone");
//                                telemetry.update();
//                                idle();
//                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
                        .addTemporalMarker(() ->{
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
                        })
                        .waitSeconds(auto_move_wait)
                        .addTemporalMarker(()->{
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Lift up cone");
//                                telemetry.update();
//                                idle();
//                            }
                            timer.reset();
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
                        .waitSeconds(auto_move_wait)
                        .turn(Math.toRadians(130))
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                        })
                        .waitSeconds(auto_move_wait)
                        .lineToLinearHeading(Blue_Preload_DropOff)
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//                                idle();
//                            }
                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                        })
                        .waitSeconds(auto_drop_wait)
                        .lineToLinearHeading(Ryk_Robot.Blue_Preload_Tile)
                        .turn(Math.toRadians(-90))
//                        //cone 4
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopMidCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Top Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 3
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking Up Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 2
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomMidCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Bottom Middle Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        //cone 1
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomCone);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking Up Bottom Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Blue_Cycle_Tile)
                        //park
                        .addTemporalMarker(() -> {
                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);

//                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//                                telemetry.addLine("Lowering to rest position.");
//                                telemetry.update();
//                                idle();
//                            }
                        })
                        .lineToLinearHeading(Ryk_Robot.Blue_Park_Pos3)
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
        pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
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

    public void getTag(AprilTagDetectionPipeline pipeline){

        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

        if(currentDetections.size() != 0)
        {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections)
            {
                if(tag.id == LEFT ||  tag.id == MIDDLE || tag.id == RIGHT)
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        }
        else
        {
            telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null)
            {
                telemetry.addLine("(The tag has never been seen)");
            }
            else
            {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

        telemetry.update();
        sleep(20);
    }







    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}


