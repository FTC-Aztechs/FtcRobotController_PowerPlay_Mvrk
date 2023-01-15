package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.CycleExtendFlamethrowerOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.CycleRetractFlamethrowerOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.DropoffPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.FloorPosition;
//import static org.firstinspires.ftc.teamcode.Mvrk_Robot.IntakeInsidePos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.DropoffExtendFlamethrowerOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.PlSlideDownOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.PlSlideUpOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Dropoff;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Inter_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Park_Pos1;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Park_Pos2;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Park_Pos3;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Pickup;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Push_Signal;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Start;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkMotors.CAT_MOUSE;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.FLAMETHROWER;
//import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.FUNKY_MONKEY;
//import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.TWIN_TOWERS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.TopMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.TopCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_drop_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_extend_half_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_half_raise_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_move_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_pickup_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_raise_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_retract_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_cyclesToRun;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideDropPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideOutPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
@Autonomous(name="Red Corner",group = "Autonomous")
public class Mvrk_Autonomous_Red extends LinearOpMode {

    private Pose2d currentPose;

    enum MvrkAllianceField {
        RED,
        BLUE
    }

    Mvrk_Robot Mavryk = new Mvrk_Robot();

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
    public TrajectorySequence trajCycleDropOffTopCone;
    public TrajectorySequence trajCycleDropOffTopMidCone;
    public TrajectorySequence trajCycleDropOffMiddleCone;
    public TrajectorySequence trajCycleDropOffBottomMidCone;
    public TrajectorySequence trajCycleDropOffBottomCone;
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

//        Mavryk.setPosition(TWIN_TOWERS, Mavryk.Claw_Close_Pos);
//        Mavryk.setPosition(FUNKY_MONKEY, IntakeInsidePos);
        Mavryk.setPosition(FLAMETHROWER, xSlideInPos);

        telemetry.addData("Status: ", "Building Pre-load and drop off Trajectories......");
        telemetry.update();

        buildPreloadTrajectory();
        if(Red_cyclesToRun > 0)
            trajCycleDropOffTopCone = buildCycleTrajectory(TopMidCone); // Note: Drop slides to pick up the next cone, in this case Top Mid
        if(Red_cyclesToRun > 1)
            trajCycleDropOffTopMidCone = buildCycleTrajectory(MiddleCone); // Note: Drop slides to pick up the next cone, in this case Middle
        if(Red_cyclesToRun > 2)
            trajCycleDropOffMiddleCone = buildCycleTrajectory(BottomMidCone); // Note: Drop slides to pick up the next cone, in this case BottomMid
        if(Red_cyclesToRun > 3)
            trajCycleDropOffBottomMidCone = buildCycleTrajectory(BottomCone); // Note: Drop slides to pick up the next cone, in this case Bottom
        if(Red_cyclesToRun > 4)
            trajCycleDropOffBottomCone = buildCycleTrajectory(FloorPosition); // Note: Drop slides to pick up the next cone, in this case Floor

        telemetry.update();

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
//                telemetry.addData("Status: ", "Sauron Streaming ...");
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

//                if (tagFound) {
//                    //telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    //tagToTelemetry(tagOfInterest);
//                } else {
//                    telemetry.addLine("Don't see tag of interest :(");
//
//                    if (tagOfInterest == null) {
//                        //telemetry.addLine("(The tag has never been seen)");
//                    } else {
//                        //telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        //tagToTelemetry(tagOfInterest);
//                    }
//                }

            } else {
//                telemetry.addLine("Don't see tag of interest :(");
//
//                if (tagOfInterest == null) {
//                    telemetry.addLine("(The tag has never been seen)");
//                } else {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }

            }

            telemetry.update();
            sleep(20);
        }

//        //once program starts
//        if (tagOfInterest != null) {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        } else {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }

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

        buildParkTrajectory(pos);
        //telemetry.update();

        // Drop off preload
        trajectoryTimer.reset();
        Mavryk.mecanumDrive.setPoseEstimate(Red_Start.pose2d());
        Mavryk.mecanumDrive.followTrajectorySequence(trajPreLoadDropOff);
        telemetry.addLine(String.format("%d. Preload Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));

        // Cycle 1
        if (Red_cyclesToRun > 0) {
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOffTopCone);
            telemetry.addLine(String.format("%d. Cycle 1 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 2
        if (Red_cyclesToRun > 1) {
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOffTopMidCone);
            telemetry.addLine(String.format("%d. Cycle 2 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 3
        if (Red_cyclesToRun > 2) {
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOffMiddleCone);
            telemetry.addLine(String.format("%d. Cycle 3 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 4
        if (Red_cyclesToRun > 3) {
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOffBottomMidCone);
            telemetry.addLine(String.format("%d. Cycle 4 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Cycle 5
        if (Red_cyclesToRun > 4) {
            trajectoryTimer.reset();
            Mavryk.mecanumDrive.followTrajectorySequence(trajCycleDropOffBottomCone);
            telemetry.addLine(String.format("%d. Cycle 5 Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        }

        // Park
//        EstimateCurrentPose();
//        if (!currentPose.epsilonEquals(Red_Pickup)) {
//            Trajectory trajAdjustPos = Mavryk.mecanumDrive.trajectoryBuilder(currentPose)
//                    .lineToLinearHeading(Red_Pickup)
//                    .build();
//            Mavryk.mecanumDrive.followTrajectory(trajAdjustPos);
//        }
        trajectoryTimer.reset();
        Mavryk.mecanumDrive.followTrajectorySequence(trajParking);

//        Mvrk_Robot.currentPose= Mavryk.mecanumDrive.getPoseEstimate();

        telemetry.addLine(String.format("%d. Park Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));

        telemetry.update();
    }

    private void EstimateCurrentPose() {
        // TODO: use vumarks to update current pose
        currentPose = Red_Pickup.pose2d();
    }

    void buildPreloadTrajectory() {
        telemetry.addLine(String.format("%d. buildPreloadTrajectory", iTeleCt++));

        trajPreLoadDropOff = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Start.pose2d())
                //preload
                .lineToLinearHeading(Red_Push_Signal.pose2d())
                .lineToLinearHeading(Red_Dropoff.pose2d())
                .UNSTABLE_addTemporalMarkerOffset(PlSlideUpOffset, () -> {
                    // Raise Tom&Jerry
                    Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
                })
                .waitSeconds(auto_raise_wait)
                .UNSTABLE_addTemporalMarkerOffset(DropoffExtendFlamethrowerOffset, () -> { // Start after 1.5s of raise
                    // Extend FlameThrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideOutPos);
                })

                .waitSeconds(auto_extend_half_wait)
                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
                    // Lower Tom&Jerry to Top Cone
                    Mavryk.setTargetPosition(CAT_MOUSE, DropoffPos);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//                    Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
                })
                .waitSeconds(auto_drop_wait)
                .addTemporalMarker(() -> {
                    // Retract FlameThrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideDropPos);
                })
                .waitSeconds(auto_retract_wait) // Eliminate
                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
                    // Lower Tom&Jerry to Top Cone
                    Mavryk.setTargetPosition(CAT_MOUSE, TopCone);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
                })
                .waitSeconds(auto_drop_wait)
                .build();


        int iNumSegments = trajPreLoadDropOff.size();
        telemetry.addLine(String.format("%d. Preload Trajectory numTrajectory Segments: %d", iTeleCt++, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Preload Trajectory Segment %d Duration: %.3f", iTeleCt++, iSeg,trajPreLoadDropOff.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Park Preload calculated Duration: %.3f", iTeleCt++, trajPreLoadDropOff.duration()));

        return;
    }

    void buildParkTrajectory(int iPos)
    {
        telemetry.addLine(String.format("%d. buildParkTrajectory", iTeleCt++));

        switch (iPos) {
            case 1:
            default:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff.pose2d())
                        .addTemporalMarker(()->{
                            Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
                        })
                        .lineToLinearHeading(Red_Park_Pos1.pose2d())
                        .build();
                break;
            case 2:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff.pose2d())
                        .addTemporalMarker(()->{
                            Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
                        })
                        .lineToLinearHeading(Red_Park_Pos2.pose2d())
                        .build();
                break;
            case 3:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff.pose2d())
                        .addTemporalMarker(()->{
                            Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
                        })
                        .lineToLinearHeading(Red_Park_Pos3.pose2d())
                        .build();
                break;
        }

        int iNumSegments = trajParking.size();
        telemetry.addLine(String.format("%d. Park Trajectory numTrajectory Segments: %d", iTeleCt++, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Park Trajectory Segment %d Duration: %.3f", iTeleCt++, iSeg,trajParking.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Park Trajectory calculated Duration: %.3f", iTeleCt++, trajParking.duration()));

        return;
    }

    TrajectorySequence buildCycleTrajectory(int iCycleConePickup)
    {
        telemetry.addLine(String.format("%d. buildCycleTrajectory %d", iTeleCt++, iCycleConePickup));
        TrajectorySequence trajSeq = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff.pose2d())
//                .lineToLinearHeading(Red_Inter_Pos.pose2d())
                .lineToLinearHeading(Red_Pickup.pose2d())
                .waitSeconds(auto_move_wait)  // Eliminate
                .UNSTABLE_addTemporalMarkerOffset(CycleExtendFlamethrowerOffset, () -> {
                    // Extend Flamethrower & Grab Cone
                    Mavryk.setPosition(FLAMETHROWER, xSlideOutPos);
                })
                .waitSeconds(auto_extend_half_wait)
                .addTemporalMarker(() -> {
//                    Mavryk.setPosition(TWIN_TOWERS, Claw_Close_Pos);
                })
                .waitSeconds(auto_pickup_wait)
                .addTemporalMarker(() -> {
                    // Raise to Low Junction
                    Mavryk.setTargetPosition(CAT_MOUSE, LowJunction);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
                })
                .waitSeconds(auto_half_raise_wait)
                .UNSTABLE_addTemporalMarkerOffset(CycleRetractFlamethrowerOffset, () -> {
                    // Retract Flamethrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideDropPos);
                })
                .waitSeconds(auto_retract_wait) // Eliminate
                .lineToLinearHeading(Red_Dropoff.pose2d())
                .UNSTABLE_addTemporalMarkerOffset(PlSlideUpOffset, () -> {
                    // Raise Tom&Jerry
                    Mavryk.setTargetPosition(CAT_MOUSE, HighJunction);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Up);
                })
                .waitSeconds(auto_raise_wait)
                .UNSTABLE_addTemporalMarkerOffset(DropoffExtendFlamethrowerOffset, () -> {
                    // Extend FlameThrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideOutPos);
                })
                .waitSeconds(auto_extend_half_wait)
                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
                    // Lower Tom&Jerry to Top Cone
                    Mavryk.setTargetPosition(CAT_MOUSE, DropoffPos);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
//                    Mavryk.setPosition(TWIN_TOWERS, Claw_Open_Pos);
                })
                .waitSeconds(auto_drop_wait)
                .addTemporalMarker(() -> {
                    // Retract FlameThrower
                    Mavryk.setPosition(FLAMETHROWER, xSlideDropPos);
                })
                .waitSeconds(auto_retract_wait) // Eliminate
                .addTemporalMarker(() -> {
                    // Lower Tom&Jerry to Top Cone
                    Mavryk.setTargetPosition(CAT_MOUSE, iCycleConePickup);
                    Mavryk.setRunMode(CAT_MOUSE, RUN_TO_POSITION);
                    Mavryk.setPower(CAT_MOUSE, SlidePower_Down);
                })
                .build();

        int iNumSegments = trajSeq.size();
        telemetry.addLine(String.format("%d. Cycle %d numTrajectory Segments: %d", iTeleCt++, iCycleConePickup, iNumSegments));
        for(int iSeg=0; iSeg<iNumSegments; iSeg++ ) {
            telemetry.addLine(String.format("%d. Cycle %d Trajectory Segment %d Duration: %.3f", iTeleCt++, iCycleConePickup, iSeg,trajSeq.get(iSeg).getDuration()));
        }
        telemetry.addLine(String.format("%d. Cycle %d Trajectory calculated Duration: %.3f", iTeleCt++, iCycleConePickup, trajSeq.duration()));
        return trajSeq;
    }

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


