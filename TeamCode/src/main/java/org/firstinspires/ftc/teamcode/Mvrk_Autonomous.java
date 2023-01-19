package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Position;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.CycleExtendFlamethrowerOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.CycleRetractFlamethrowerOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.DropoffPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomLimit;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.DropoffExtendFlamethrowerOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.CARTOON;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MvrkServos.TEACUP;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.PlSlideDownOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.PlSlideUpOffset;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Red_Dropoff;
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
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_drop_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_extend_half_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_half_raise_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_move_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_pickup_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_raise_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.auto_retract_wait;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.slide_newPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretRight;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretUp;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turret_newPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideDropPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideOutPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlide_Position;

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
public class Mvrk_Autonomous extends LinearOpMode {

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

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
    final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
    final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;

    private boolean targetVisible       = false;


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

    Mvrk_Robot.AutoState currentAutoState = Mvrk_Robot.AutoState.IDLE;

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

        Mavryk.setPosition(CARTOON, Claw_Close_Pos);
        Mavryk.setPosition(FLAMETHROWER, xSlideInPos);
        Mavryk.setPosition(TEACUP, turret_newPos);

        telemetry.addData("Status: ", "Building Pre-load and drop off Trajectories......");
        telemetry.update();

        buildPreloadTrajectory();
        trajCycleDropOffTopCone = buildCycleTrajectory(TopCone); // Note: Drop slides to pick up the next cone, in this case Top Mid
        trajCycleDropOffTopMidCone = buildCycleTrajectory(TopMidCone); // Note: Drop slides to pick up the next cone, in this case Middle
        trajCycleDropOffMiddleCone = buildCycleTrajectory(MiddleCone); // Note: Drop slides to pick up the next cone, in this case BottomMid
        trajCycleDropOffBottomMidCone = buildCycleTrajectory(BottomMidCone); // Note: Drop slides to pick up the next cone, in this case Bottom
        trajCycleDropOffBottomCone = buildCycleTrajectory(BottomCone); // Note: Drop slides to floor to park

        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Sauron = OpenCvCameraFactory.getInstance().createWebcam(Mavryk.eyeOfSauron, cameraMonitorViewId);
        telemetry.addData("Status: ", "camera created  ...");
        telemetry.update();

        //initVuforia();
        telemetry.addData("Status: ", "initialized vuforia");
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

//            telemetry.update();
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

        buildParkTrajectory(pos);

        initMotorsAndServos(true);

        // Drop off preload
        trajectoryTimer.reset();
        Mavryk.mecanumDrive.setPoseEstimate(Red_Start.pose2d());
        currentAutoState = Mvrk_Robot.AutoState.PRELOAD;
        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajPreLoadDropOff);
//        telemetry.addLine(String.format("%d. Preload Trajectory completed in: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        //while(Mavryk.mecanumDrive.isBusy())
        {
            // Init and start reading Vuforia stuff?
            // Get position from camera
        }
        //EstimateCurrentPose();
//        Mavryk.mecanumDrive.setPoseEstimate(currentPose);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

            // We essentially define the flow of the state machine through this switch statement
            switch (currentAutoState) {
                case PRELOAD:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes" );
                        currentAutoState = Mvrk_Robot.AutoState.TOPCONE;
                        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffTopCone);
                    }
                    telemetry.update();
                    break;
                case TOPCONE:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes" );
                        currentAutoState = Mvrk_Robot.AutoState.TOPMIDCONE;
                        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffTopMidCone);
                    }
                    telemetry.update();
                    break;
                case TOPMIDCONE:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes" );
                        currentAutoState = Mvrk_Robot.AutoState.MIDCONE;
                        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffMiddleCone);
                    }
                    telemetry.update();
                    break;
                case MIDCONE:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes");
                        currentAutoState = Mvrk_Robot.AutoState.BOTTOMMIDCONE;
                        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffBottomMidCone);
                    }
                    telemetry.update();
                    break;
                case BOTTOMMIDCONE:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes" );
                        currentAutoState = Mvrk_Robot.AutoState.BOTTOMCONE;
                        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajCycleDropOffBottomCone);
                    }
                    telemetry.update();
                    break;
                case BOTTOMCONE:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes" );
                        currentAutoState = Mvrk_Robot.AutoState.PARK;
                        Mavryk.mecanumDrive.followTrajectorySequenceAsync(trajParking);
                    }
                    telemetry.update();
                    break;
                case PARK:
                    telemetry.addData("switching", "no" );
                    if (!Mavryk.mecanumDrive.isBusy()) {
                        telemetry.addData("switching", "yes" );
                        currentAutoState = Mvrk_Robot.AutoState.IDLE;
                    }
                    telemetry.update();
                    break;
                case IDLE:
                    telemetry.addData("switching", "no" );
                    telemetry.update();
                    // Do nothing in IDLE
                    // currentAutoState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            // Print state to telemetry
            telemetry.addData("Currently Running", currentAutoState);
            telemetry.update();

            Mavryk.mecanumDrive.update();

//            //Slides
//            if( newSlidePos != currentSlidePos && newSlidePos >= FloorPosition && newSlidePos <= HighJunction ) {
//                double command = Mvrk_Robot.control.output(newSlidePos, Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
//                if(newSlidePos < currentSlidePos)
//                    Mvrk_Robot.SlidePower = Math.max(command/HighJunction, SlidePower_Down);
//                else
//                    Mvrk_Robot.SlidePower = Math.min(command/HighJunction, SlidePower_Up);
//                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE,Mvrk_Robot.SlidePower);
//                currentSlidePos = Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE);
//            }
//
//            telemetry.addData("Current Slide Position: ", currentSlidePos);
//            telemetry.update();
//
//            //Turret
//            if( Mavryk.turret_newPos != Mavryk.turret_currentPos  && Mavryk.turret_newPos >= turret_Range[0] && Mavryk.turret_newPos <= turret_Range[1] ) {
//                Mavryk.turret_Move = (Mavryk.turret_newPos - Mavryk.turret_currentPos) * Mvrk_Robot.turretSpeed;
//                Mavryk.Teacup.setPosition(Mavryk.Teacup.getPosition() + Mavryk.turret_Move);
//                Mavryk.turret_currentPos = Mavryk.Teacup.getPosition();
//            }
//
//            //xSlide
//            Mavryk.FlameThrower.setPosition(Mavryk.xSlide_Position);
//
//            //Claw
//            Mavryk.Looney.setPosition(Mavryk.Claw_Position);
        }

    }




//    private void EstimateCurrentPose() {
//        // TODO: use vumarks to update current pose
//
//
//        currentPose = Red_Pickup.pose2d();
//
////        //get vuforia position
////        // check all the trackable targets to see which one (if any) is visible.
////        targetVisible = false;
////        for (VuforiaTrackable trackable : allTrackables) {
////            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
////                telemetry.addData("Visible Target", trackable.getName());
////                targetVisible = true;
////
////                // getUpdatedRobotLocation() will return null if no new information is available since
////                // the last time that call was made, or if the trackable is not currently visible.
////                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
////                if (robotLocationTransform != null) {
////                    lastLocation = robotLocationTransform;
////                }
////                break;
////            }
////        }
//
//        // Provide feedback as to where the robot is located (if we know).
//        if (targetVisible) {
//            // express position (translation) of robot in inches.
//            VectorF translation = lastLocation.getTranslation();
//            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
//                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
//
//            // express the rotation of the robot in degrees.
//            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
//            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
//        }
//        else {
//            telemetry.addData("Visible Target", "none");
//        }
//        telemetry.update();
//        //get separate coordinates
//        //set currentPose = -|x|, -|y|
//    }

    void buildPreloadTrajectory() {
        telemetry.addLine(String.format("%d. buildPreloadTrajectory", iTeleCt++));

        trajPreLoadDropOff = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Start.pose2d())
                //preload
                .lineToLinearHeading(Red_Push_Signal.pose2d())
                .splineToLinearHeading(Red_Dropoff.pose2d(), Math.toRadians(200))
                .UNSTABLE_addTemporalMarkerOffset(PlSlideUpOffset,() -> {
                    // Raise Tom&Jerry
                    turret_newPos = turretRight;
                    slide_newPos = HighJunction;
                })
                .waitSeconds(auto_raise_wait)
                .UNSTABLE_addTemporalMarkerOffset(DropoffExtendFlamethrowerOffset, () -> { // Start after 1.5s of raise
                    // Extend FlameThrower
                    xSlide_Position = xSlideDropPos;
                })
                .waitSeconds(auto_extend_half_wait)
                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
                    // Lower Tom&Jerry to Dropoff Height
                    slide_newPos = DropoffPos;
                    Claw_Position = Claw_Open_Pos;
                })
                .waitSeconds(auto_drop_wait)
                .addTemporalMarker(() -> {
                    // Retract FlameThrower
                    xSlide_Position = xSlideInPos;
                })
                .waitSeconds(auto_retract_wait) // Eliminate
                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
                    // Lower Tom&Jerry to Top Cone
                    slide_newPos = TopCone;
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
                            turret_newPos = turretUp;
                            xSlide_Position = xSlideInPos;
                        })
                        .lineToLinearHeading(Red_Park_Pos1.pose2d())
                        .build();
                break;
            case 2:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff.pose2d())
                        .addTemporalMarker(()->{
                            turret_newPos = turretUp;
                            xSlide_Position = xSlideInPos;
                        })
                        .lineToLinearHeading(Red_Park_Pos2.pose2d())
                        .build();
                break;
            case 3:
                trajParking = Mavryk.mecanumDrive.trajectorySequenceBuilder(Red_Dropoff.pose2d())
                        .addTemporalMarker(()->{
                            turret_newPos = turretUp;
                            xSlide_Position = xSlideInPos;
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
                .lineToLinearHeading(Red_Pickup.pose2d())
                .waitSeconds(auto_move_wait)  // Eliminate
                .UNSTABLE_addTemporalMarkerOffset(CycleExtendFlamethrowerOffset, () -> {
                    // Extend Flamethrower & Grab Cone
                    turret_newPos = turretUp;
                    xSlide_Position = xSlideOutPos;
                })
                .waitSeconds(auto_extend_half_wait)
                .addTemporalMarker(() -> {
                    Claw_Position = Claw_Close_Pos;
                })
                .waitSeconds(auto_pickup_wait)
                .addTemporalMarker(() -> {
                    // Raise to Low Junction
                    slide_newPos = LowJunction;
                })
                .waitSeconds(auto_half_raise_wait)
                .UNSTABLE_addTemporalMarkerOffset(CycleRetractFlamethrowerOffset, () -> {
                    // Retract Flamethrower
                    xSlide_Position = xSlideInPos;
                })
                .waitSeconds(auto_retract_wait) // Eliminate
                .lineToLinearHeading(Red_Dropoff.pose2d())
                .UNSTABLE_addTemporalMarkerOffset(PlSlideUpOffset, () -> {
                    // Raise Tom&Jerry
                    turret_newPos = turretUp;
                    slide_newPos = HighJunction;
                })
                .waitSeconds(auto_raise_wait)
                .UNSTABLE_addTemporalMarkerOffset(DropoffExtendFlamethrowerOffset, () -> {
                    // Extend FlameThrower
                    xSlide_Position = xSlideDropPos;
                })
                .waitSeconds(auto_extend_half_wait)
                .UNSTABLE_addTemporalMarkerOffset(PlSlideDownOffset, () -> {
                    // Lower Tom&Jerry to Top Cone
                    slide_newPos = DropoffPos;
                    Claw_Position = Claw_Open_Pos;
                })
                .waitSeconds(auto_drop_wait)
                .addTemporalMarker(() -> {
                    // Retract FlameThrower
                    xSlide_Position = xSlideInPos;
                })
                .waitSeconds(auto_retract_wait) // Eliminate
                .addTemporalMarker(() -> {
                   slide_newPos = iCycleConePickup;
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

//    void initVuforia(){
//        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
////        webcamName = Mavryk.eyeOfSauron;
//
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
//         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
//         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
//         */
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
//
//        // We also indicate which camera we wish to use.
//        parameters.cameraName = webcamName;
//
//        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
//        parameters.useExtendedTracking = false;
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Load the data sets for the trackable objects. These particular data
//        // sets are stored in the 'assets' part of our application.
//        targets = this.vuforia.loadTrackablesFromAsset("PowerPlay");
//
//        // For convenience, gather together all the trackable objects in one easily-iterable collection */
//        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//        allTrackables.addAll(targets);
//
//
//        // Name and locate each trackable object
//        identifyTarget(0, "Red Audience Wall",   -halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0,  90);
//        identifyTarget(1, "Red Rear Wall",        halfField,  -oneAndHalfTile, mmTargetHeight, 90, 0, -90);
//        identifyTarget(2, "Blue Audience Wall",  -halfField,   oneAndHalfTile, mmTargetHeight, 90, 0,  90);
//        identifyTarget(3, "Blue Rear Wall",       halfField,   oneAndHalfTile, mmTargetHeight, 90, 0, -90);
//
//        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
//                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));
//
//        /**  Let all the trackable listeners know where the camera is.  */
//        for (VuforiaTrackable trackable : allTrackables) {
//            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
//        }
//    }
//    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
//        VuforiaTrackable aTarget = targets.get(targetIndex);
//        aTarget.setName(targetName);
//        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
//                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
//    }

}


