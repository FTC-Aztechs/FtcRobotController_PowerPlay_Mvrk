package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.BottomCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.BottomMidCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Claw_Open_Pos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.DropoffPos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.IntakeInsidePos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Dropoff;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Park_Pos1;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Park_Pos2;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Park_Pos3;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Pickup;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Preload_DropOff;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Push_Signal;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.Red_Start;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.RykMotors.CAT_MOUSE;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.RykServos.FLAMETHROWER;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.RykServos.FUNKY_MONKEY;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.RykServos.TWIN_TOWERS;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.TopMidCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.TopCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.GroundJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_drop_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_extend_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_half_raise_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_move_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_pickup_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_raise_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.auto_retract_wait;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.xSlideDropPos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.xSlideInPos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.xSlideOutPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
@Autonomous(name="Autonomous Testing",group = "Autonomous")
public class Ryk_Autonomous extends LinearOpMode {

    private Pose2d currentPose;

    enum RykAllianceField {
        RED,
        BLUE
    }

    Ryk_Robot Mavryk = new Ryk_Robot();

    private static FtcDashboard rykRobot;
    OpenCvWebcam Sauron = null;
    AprilTagDetectionPipeline pipeline;

    private static int iTeleCt = 1;
    public static int cyclesToRun = 1;

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

//    public TrajectorySequence Park;
    public TrajectorySequence trajPreLoadDropOff;
    public TrajectorySequence trajCycleDropOff;
    public TrajectorySequence trajParking;
    public int currentCyclePickupCone = TopCone;

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

        Mavryk.setPosition(TWIN_TOWERS, Mavryk.Claw_Close_Pos);
        Mavryk.setPosition(FUNKY_MONKEY, IntakeInsidePos);
        Mavryk.setPosition(FLAMETHROWER, xSlideInPos);

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
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        //once program starts
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        // 1. Calculate Parking Position

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            telemetry.addLine("Going to Position 1");
            pos = 1;
        } else if (tagOfInterest.id == MIDDLE) {
            telemetry.addLine("Going to Position 2");
            pos = 2;
        } else {
            telemetry.addLine("Going to Position 3");
            pos = 3;
        }

        telemetry.update();

        initMotorsAndServos(true);

        Pose2d startPose = Red_Start;

//        buildParkTrajectories_position(pos);
//        Mavryk.mecanumDrive.setPoseEstimate(startPose);
//
//        telemetry.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
//        trajectoryTimer.reset();
//
//        Mavryk.mecanumDrive.followTrajectorySequence(Park);

        // Coach Stuff: 
        buildParkTrajectories_position_coach(pos);

        // Drop off preload
        trajectoryTimer.reset();
        Mavryk.mecanumDrive.setPoseEstimate(Red_Start);
        Mavryk.mecanumDrive.followTrajectorySequence(trajPreLoadDropOff);
        telemetry.addLine(String.format("%d. Preload Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));

        // Cycle 1
        if (cyclesToRun > 0) {
            currentCyclePickupCone = TopCone;
            EstimateCurrentPose();
            if(!currentPose.epsilonEquals(Red_Pickup))
            {
                Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(Red_Pickup)
                        .build();
                Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
            }
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOff);
            telemetry.addLine(String.format("%d. Cycle 1 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 2
        if (cyclesToRun > 1) {
            currentCyclePickupCone = TopMidCone;
            EstimateCurrentPose();
            if(!currentPose.epsilonEquals(Red_Pickup))
            {
                Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(Red_Pickup)
                        .build();
                Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
            }
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOff);
            telemetry.addLine(String.format("%d. Cycle 2 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 3
        if (cyclesToRun > 2) {
            currentCyclePickupCone = MiddleCone;
            EstimateCurrentPose();
            if(!currentPose.epsilonEquals(Red_Pickup))
            {
                Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(Red_Pickup)
                        .build();
                Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
            }
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOff);
            telemetry.addLine(String.format("%d. Cycle 3 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 4
        if (cyclesToRun > 3) {
            currentCyclePickupCone = BottomMidCone;
            EstimateCurrentPose();
            if(!currentPose.epsilonEquals(Red_Pickup))
            {
                Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(Red_Pickup)
                        .build();
                Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
            }
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOff);
            telemetry.addLine(String.format("%d. Cycle 4 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 5
        if (cyclesToRun > 4) {
            currentCyclePickupCone = BottomCone;
            EstimateCurrentPose();
            if(!currentPose.epsilonEquals(Red_Pickup))
            {
                Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
                        .lineToLinearHeading(Red_Pickup)
                        .build();
                Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
            }
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOff);
            telemetry.addLine(String.format("%d. Cycle 5 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Park
        EstimateCurrentPose();
        if(!currentPose.epsilonEquals(Red_Pickup))
        {
            Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
                    .lineToLinearHeading(Red_Pickup)
                    .build();
            Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
        }
        trajectoryTimer.reset();
        Mavryk.mecanumDrive.followTrajectorySequence(trajParking);
        telemetry.addLine(String.format("%d. Park Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));

        telemetry.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        telemetry.update();
    }

    private void EstimateCurrentPose() {
        // TODO: use vumarks to update current pose
        currentPose = Red_Pickup;
    }

    void buildParkTrajectories_position_coach(int col) {

        telemetry.addLine(String.format("%d. buildParkTrajectories_position_coach", iTeleCt++));

        trajPreLoadDropOff = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Start)
                //preload
                .lineToLinearHeading(Red_Push_Signal)
                .lineToLinearHeading(Red_Dropoff)
                .addTemporalMarker(() -> {
                    // Raise Tom&Jerry
                    Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
                })
                .waitSeconds(auto_raise_wait)
                .addTemporalMarker(()-> {
                    // Extend FlameThrower
                    Mavryk.setPosition(FLAMETHROWER,xSlideOutPos );
                })
                .waitSeconds(auto_extend_wait)
                .addTemporalMarker(()-> {
                    // Lower Tom & Jerry
                    Mavryk.setTargetPosition(CAT_MOUSE, DropoffPos);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
                    Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
                })
                .waitSeconds(auto_drop_wait)
                .addTemporalMarker(()->{
                    // Retract FlameThrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
                })
                .waitSeconds(auto_retract_wait)
                .addTemporalMarker(()->{
                    // Lower Tom&Jerry to Top Cone
                    Mavryk.setTargetPosition(CAT_MOUSE, TopCone);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
                })
                .waitSeconds(auto_drop_wait)
                .build();

        trajCycleDropOff = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff)
                .lineToLinearHeading(Red_Pickup)
                .waitSeconds(auto_move_wait)
                .addTemporalMarker(() -> {
                    // Extend Flamethrower & Grab Cone
                    Mavryk.setPosition(FLAMETHROWER, xSlideOutPos);
                })
                .waitSeconds(auto_extend_wait)
                .addTemporalMarker(()-> {
                    Mavryk.setPosition(TWIN_TOWERS, Claw_Close_Pos);
                })
                .waitSeconds(auto_pickup_wait)
                .addTemporalMarker(()->{
                    // Raise to Low Junction
                    Mavryk.setTargetPosition(CAT_MOUSE, LowJunction);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
                })
                .waitSeconds(auto_half_raise_wait)
                .addTemporalMarker(()->{
                    // Retract Flamethrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
                })
                .waitSeconds(auto_retract_wait)
                .lineToLinearHeading(Red_Dropoff)
                .addTemporalMarker(() -> {
                    // Raise Tom&Jerry
                    Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
                })
                .waitSeconds(auto_raise_wait)
                .addTemporalMarker(()-> {
                    // Extend FlameThrower
                    Mavryk.setPosition(FLAMETHROWER,xSlideOutPos );
                })
                .waitSeconds(auto_extend_wait)
                .addTemporalMarker(()-> {
                    // Lower Tom & Jerry
                    Mavryk.setTargetPosition(CAT_MOUSE, DropoffPos);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
                    Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
                })
                .waitSeconds(auto_drop_wait)
                .addTemporalMarker(() -> {
                    // Retract FlameThrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
                })
                .waitSeconds(auto_retract_wait)
                .addTemporalMarker(() -> {
                    // Lower Tom&Jerry to Top Cone
                    Mavryk.setTargetPosition(CAT_MOUSE, currentCyclePickupCone);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
                })
                .build();

        switch(col) {
            case 1:
            default:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff)
                        .lineToLinearHeading(Red_Park_Pos1)
                        .build();
                break;
            case 2:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff)
                        .lineToLinearHeading(Red_Park_Pos2)
                        .build();
                break;
            case 3:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff)
                        .lineToLinearHeading(Red_Park_Pos3)
                        .build();
                break;
        }

        telemetry.addLine(String.format("%d. Trajectory Duration after build: %.3f", iTeleCt++, trajParking.duration()));
        return;
    }

//    void buildParkTrajectories_position(int col) {
//
//        telemetry.addLine(String.format("%d. buildWarehouseTrajectories_position", iTeleCt++));
//
//        switch (col) {
//            case 1:
//                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Start)
//                        //preload
//                        .lineToLinearHeading(Ryk_Robot.Red_Offset)
//                        .lineToLinearHeading(Ryk_Robot.Red_Dropoff_Dodge)
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-45))
//                        .addTemporalMarker(() -> {
//                            Mavryk.setPosition(FLAMETHROWER, xSlideDropPos);
//                            Mavryk.setTargetPosition(CAT_MOUSE, DropoffPos);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .addTemporalMarker(()-> {
//                            Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
//
//                            Mavryk.setTargetPosition(CAT_MOUSE, GroundJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//                        })
//                        .turn(Math.toRadians(135))
//                        .lineToLinearHeading(Red_Park_Pos1)
//                        .waitSeconds(auto_move_wait)
////                        //pickup cone 5
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, TopCone);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//
//                            Mavryk.setPosition(FLAMETHROWER, xSlideOutPos);
//
//                        })
//                        .lineToLinearHeading(Red_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//                            timer.reset();
//                            Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
//
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-90))
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-45))
//                        .addTemporalMarker(() -> {
//                            Mavryk.setPosition(FLAMETHROWER, xSlideDropPos);
//                            Mavryk.setTargetPosition(CAT_MOUSE, DropoffPos);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .addTemporalMarker(()-> {
//                            Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
//
//                            Mavryk.setTargetPosition(CAT_MOUSE, GroundJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//                        })
//                        .turn(Math.toRadians(135))
////                        //cone 4
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopMidCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking up Top Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 3
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking Up Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 2
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomMidCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking up Bottom Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 1
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking Up Bottom Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //park
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, GroundJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lowering to rest position.");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Red_Park_Pos1)
//                        .build();
//                break;
//            case 2:
//                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Start)
//                        //preload
//                        .lineToLinearHeading(Ryk_Robot.Red_Offset)
//                        .lineToLinearHeading(Ryk_Robot.Red_Dropoff_Dodge)
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-37))
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .turn(Math.toRadians(90))
//                        .lineToLinearHeading(Red_Park_Pos1)
//                        .waitSeconds(auto_move_wait)
////                        //pickup cone 5
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, TopCone);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Top Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Red_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-135))
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Red_Preload_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .turn(Math.toRadians(90))
////                        //cone 4
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopMidCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking up Top Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 3
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking Up Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 2
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomMidCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking up Bottom Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 1
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking Up Bottom Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
//                        //park
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, GroundJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lowering to rest position.");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Red_Park_Pos2)
//                        .build();
//
//                break;
//            case 3:
//                Park = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Start)
//                        //preload
//                        .lineToLinearHeading(Ryk_Robot.Red_Offset)
//                        .lineToLinearHeading(Ryk_Robot.Red_Dropoff_Dodge)
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-37))
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .turn(Math.toRadians(90))
//                        .lineToLinearHeading(Red_Park_Pos1)
//                        .waitSeconds(auto_move_wait)
////                        //pickup cone 5
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, TopCone);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Picking up Top Cone");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Red_Pickup)
//                        .addTemporalMarker(() ->{
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Close_Pos);
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .addTemporalMarker(()->{
//                            Mavryk.setTargetPosition(CAT_MOUSE, Ryk_Robot.LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lift up cone");
////                                telemetry.update();
////                                idle();
////                            }
//                            timer.reset();
//                        })
//                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
//                        .waitSeconds(auto_move_wait)
//                        .turn(Math.toRadians(-135))
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                        })
//                        .waitSeconds(auto_move_wait)
//                        .lineToLinearHeading(Red_Preload_DropOff)
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, LowJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
////                                idle();
////                            }
//                            Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
//                        })
//                        .waitSeconds(auto_drop_wait)
//                        .lineToLinearHeading(Ryk_Robot.Red_Preload_Tile)
//                        .turn(Math.toRadians(90))
////                        //cone 4
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, TopMidCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking up Top Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 3
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking Up Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 2
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomMidCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking up Bottom Middle Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        //cone 1
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, BottomCone);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Picking Up Bottom Cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Pickup)
////                        .addTemporalMarker(() ->{
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Close_Pos);
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .addTemporalMarker(()->{
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, Ryk_Robot.LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
////
//////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
//////                                telemetry.addLine("Lift up cone");
//////                                telemetry.update();
//////                                idle();
//////                            }
////                            timer.reset();
////                        })
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                        })
////                        .waitSeconds(auto_move_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_DropOff)
////                        .addTemporalMarker(() -> {
////                            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
////                            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
////                            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
//////                            while(opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) ) {
//////                                idle();
//////                            }
////                            Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
////                        })
////                        .waitSeconds(auto_drop_wait)
////                        .lineToLinearHeading(Ryk_Robot.Red_Cycle_Tile)
//                        //park
//                        .addTemporalMarker(() -> {
//                            Mavryk.setTargetPosition(CAT_MOUSE, GroundJunction);
//                            Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
//                            Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//
////                            while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
////                                telemetry.addLine("Lowering to rest position.");
////                                telemetry.update();
////                                idle();
////                            }
//                        })
//                        .lineToLinearHeading(Red_Park_Pos3)
//                        .build();
//                break;
//        }
//
//        telemetry.addLine(String.format("%d. Trajectory Duration after build: %.3f", iTeleCt++, Park.duration()));
//        return;
//
//    }

    void initMotorsAndServos(boolean run_to_position)
    {
        // Reset Slides - current position becomes 0
        Mavryk.setRunMode(CAT_MOUSE, STOP_AND_RESET_ENCODER);
        Mavryk.setRunMode(CAT_MOUSE, RUN_WITHOUT_ENCODER);

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

    public void getTag(AprilTagDetectionPipeline pipeline)
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


