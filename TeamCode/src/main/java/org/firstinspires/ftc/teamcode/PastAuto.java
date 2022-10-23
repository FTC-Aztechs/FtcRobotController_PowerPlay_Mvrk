package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

/*
 * This is an example of a more complex path to really test the tuning.
 */


@Config
@Autonomous(name="Blue 2" ,group = "Autonomous")
public class Mrv_AutoBlue2 extends LinearOpMode {

    enum MrvAllianceField {
        RED,
        BLUE
    }

    Mrv_Robot marvyn = new Mrv_Robot();

    //    private static final String[] TFOD_MODEL_LABELS =
//            {
//                    "Aztechs_TSE"
//            };
    private static final String[] TFOD_MODEL_LABELS =
            {
                    "TSE"
            };

    private static FtcDashboard mrvDashboard;
    private static VuforiaLocalizer mrvVuforia;
    private static TFObjectDetector mrvTfod;
    private static Telemetry mrvDashboardTelemetry;
    private static TelemetryPacket mrvDashboardTelemetryPacket = new TelemetryPacket();
    private static int iTeleCt = 1;

    // Tensorflow camera settings
    private static int TFodResolution = 320;
    private static double TFodZoomFactor = 1;

    // VUFORIA Key
    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";
    //private static final String TFOD_MODEL_ASSET = "FreightFrenzy_AztechsTSE1.tflite";
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_AztechsTSE_Magnet.tflite";

    // Field Dimensions
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField = 72 * mmPerInch;
    private static final float halfTile = 12 * mmPerInch;
    private static final float oneAndHalfTile = 36 * mmPerInch;
    private OpenGLMatrix mrvLastLocation = new OpenGLMatrix();

    public TrajectorySequence mrvRed1;
    public TrajectorySequence mrvRed2;
    public TrajectorySequence mrvBlue1;
    public TrajectorySequence mrvBlue2;

    // Warehouse drop off level
    private static int mrvWarehouseLevel;
    private static double dWarehouseLevelToPickup;





    @Override
    public void runOpMode() throws InterruptedException {

        marvyn.init(hardwareMap);

        ElapsedTime trajectoryTimer = new ElapsedTime(MILLISECONDS);

        iTeleCt = 1;

        telemetry.clearAll();
        telemetry.update();
        telemetry.setAutoClear(false);

        // init Dashboard
        mrvDashboard = FtcDashboard.getInstance();
        mrvDashboardTelemetryPacket = new TelemetryPacket();

        telemetry.addLine(String.format("%d. Marvin Initialized!", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Marvin Initialized!", iTeleCt++));

        double volts = getBatteryVoltage();
        telemetry.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Battery voltage: %.1f volts", iTeleCt++, volts));

        // init VUFORIA
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuParams.cameraName = marvyn.eyeOfSauron;
        vuParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuParams.useExtendedTracking = false;
        mrvVuforia = ClassFactory.getInstance().createVuforia(vuParams);

        // init Tensorflow
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.4f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = TFodResolution;
        mrvTfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, mrvVuforia);
        mrvTfod.loadModelFromAsset(TFOD_MODEL_ASSET, TFOD_MODEL_LABELS);

        telemetry.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));

        // Startup camera
        mrvDashboard.startCameraStream(mrvTfod, 0); // start streaming camera
        telemetry.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));

        if (mrvTfod != null) {
            mrvTfod.activate();
            mrvTfod.setZoom(TFodZoomFactor, 16.0 / 9.0);
        }
        sleep(1000);
        telemetry.addLine(String.format("%d. Tfod activated! Ready to Start!", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Tfod activated! Ready to Start!", iTeleCt++));

        telemetry.update();
        mrvDashboard.sendTelemetryPacket(mrvDashboardTelemetryPacket);

        waitForStart();
        mrvDashboardTelemetryPacket.clearLines();

        // 1. Calculate Dropoff Level
        mrvWarehouseLevel = MrvGetWarehouseLevel(MrvAllianceField.BLUE);
        telemetry.addLine(String.format("%d. Detected Warehouse Level: %d", iTeleCt++, mrvWarehouseLevel));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Detected Warehouse Level: %d", iTeleCt++, mrvWarehouseLevel));
        telemetry.update();

        initMotorsAndServos(true);
        buildWarehouseTrajectories_position();

        // 2. Set Wrist to Drop off position
        if(mrvWarehouseLevel !=0 )
            marvyn.Wristy.setPosition(marvyn.Wrist_Dropoff_Pos);
        else
            marvyn.Wristy.setPosition(marvyn.Wrist_Pickup_Pos-0.01);


        // 3. Extend Linac
        int iLinacDropoffPos = (int) (mavryk.Linac_Dropoff_Revs * marvyn.Linac_Ticks_Per_Rev);
        double dLinacPower = 1;
        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, dLinacPower);
        while(opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.LIN_AC) < iLinacDropoffPos ) {
            idle();
        }
        marvyn.setPower(Mrv_Robot.MrvMotors.LIN_AC, 0);

        // 4. Dawinchi Dropoff Position
        DawinchiToDropoffPosition(mrvWarehouseLevel);

        // 5. Follow Trajectory
        Pose2d startPose;
        startPose = marvyn.blue_2_pose_estimate;
        marvyn.mecanumDrive.setPoseEstimate(startPose);



        telemetry.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
        trajectoryTimer.reset();

        marvyn.mecanumDrive.followTrajectorySequence(mrvBlue2);

        telemetry.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.seconds()));

        telemetry.update();
        mrvDashboard.sendTelemetryPacket(mrvDashboardTelemetryPacket);
    }

    int MrvGetWarehouseLevel(MrvAllianceField field) {
        int PyraPos = 0;
        dWarehouseLevelToPickup = marvyn.dFromLevel0ToPickup;
        marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level0_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev * -1);
        marvyn.Dawinchi_pickup_ticks = (int) (marvyn.DaWinchi_pickup_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        marvyn.Dawinchi_Level2_dropoff_ticks = (int) (marvyn.DaWinchi_Level2_Dropoff_Pos* Mrv_Robot.Dawinchi_Ticks_Per_Rev);

        if (mrvTfod != null && opModeIsActive()) {
            boolean bDuckFound = false;
            float left = 0.0f;
            float top = 0.0f;
            float bottom = 0.0f;
            float right = 0.0f;
            float mdpt = 0.0f;
            List<Recognition> updatedRecognition = mrvTfod.getUpdatedRecognitions();
            if (updatedRecognition != null) {
                telemetry.addLine(String.format("%d. # Objects detected: %d", iTeleCt++, updatedRecognition.size()));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. # Objects detected: %d", iTeleCt++, updatedRecognition.size()));
                int i = 0;
                for (Recognition recognition : updatedRecognition) {
                    left = recognition.getLeft();
                    right = recognition.getRight();
                    top = recognition.getTop();
                    bottom = recognition.getBottom();
                    mdpt = (left + right) / 2;

                    bDuckFound = true;
                    telemetry.addLine(String.format("%d. Object (%d:) ", iTeleCt++, i) + recognition.getLabel());
                    telemetry.addLine(String.format("%d. Confidence: %.3f", iTeleCt++, recognition.getConfidence()));
                    telemetry.addLine(String.format("%d. BBox (%d), (%.03f, %.03f)  (%.03f, %.03f)", iTeleCt++, i, left, top, right, bottom));
                    telemetry.addLine(String.format("%d. BBox midpt (%d),  (%.03f)", iTeleCt++, i, mdpt));
                    telemetry.addLine(String.format("%d. Image Size (%d), Width: %d; Height: %d ", iTeleCt++, i, recognition.getImageWidth(), recognition.getImageHeight()));
                    telemetry.update();

                    mrvDashboardTelemetryPacket.addLine(String.format("%d. Object (%d:) ", iTeleCt++, i) + recognition.getLabel());
                    mrvDashboardTelemetryPacket.addLine(String.format("%d. Confidence: %.3f", iTeleCt++, recognition.getConfidence()));
                    mrvDashboardTelemetryPacket.addLine(String.format("%d. BBox (%d), (%.03f, %.03f)  (%.03f, %.03f)", iTeleCt++, i, left, top, right, bottom));
                    mrvDashboardTelemetryPacket.addLine(String.format("%d. BBox midpt (%d),  (%.03f)", iTeleCt++, i, mdpt));
                    mrvDashboardTelemetryPacket.addLine(String.format("%d. Image Size (%d), Width: %d; Height: %d ", iTeleCt++, i, recognition.getImageWidth(), recognition.getImageHeight()));

                    //if (recognition.getLabel()== "Duck") {
                    if (recognition.getLabel() == "TSE" && recognition.getConfidence() > 0.8) {
                        if (marvyn.FirstPosMin <= mdpt && mdpt <= marvyn.FirstPosMax) {
                            PyraPos = 1;
                            dWarehouseLevelToPickup = marvyn.dFromLevel1ToPickup;
                            marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level1_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev * -1);
                        } else if (marvyn.SecPosMin <= mdpt && mdpt <= marvyn.SecPosMax) {
                            PyraPos = 2;
                            dWarehouseLevelToPickup = marvyn.dFromLevel2ToPickup;
                            marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level2_Dropoff_Pos*marvyn.Dawinchi_Ticks_Per_Rev * -1);
                        }
                        i++;
                        break;
                    }
                }
            } else {
                telemetry.addLine(String.format("%d. Objects Detected: None!", iTeleCt++));
                mrvDashboardTelemetryPacket.addLine(String.format("%d. Objects Detected: None!", iTeleCt++));
                telemetry.update();
                PyraPos = 0;
                dWarehouseLevelToPickup = marvyn.dFromLevel0ToPickup;
                marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level0_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev * -1);

            }
        }
        telemetry.addLine(String.format("%d. PyraPos: %d", iTeleCt++, PyraPos));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. PyraPos: %d", iTeleCt++, PyraPos));

        telemetry.update();

        return PyraPos;
    }


    void buildWarehouseTrajectories_position() {

        telemetry.addLine(String.format("%d. buildWarehouseTrajectories_position", iTeleCt++));
        mrvDashboardTelemetryPacket.addLine(String.format("%d. buildWarehouseTrajectories_position", iTeleCt++));

        mrvBlue2 = marvyn.mecanumDrive.trajectorySequenceBuilder(marvyn.blue_2_pose_estimate)

                // Step 1: Drop off pre-loaded Freight and return to Warehouse Enter pos
                .lineToLinearHeading(marvyn.blue_2_shipping_hub_pos)
                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetEjectFreight,() -> {  // Runs 0.1 seconds into the WaitSeconds call and keeps the servos on for 0.4s
                    marvyn.Claw_Left.setPower(-0.5);
                    marvyn.Claw_Right.setPower(-0.5);
                })
                .waitSeconds(marvyn.dEjectFreight)
                .addTemporalMarker( ()-> {
                    marvyn.Claw_Left.setPower(0);
                    marvyn.Claw_Right.setPower(0);
                })
                .lineToLinearHeading(marvyn.blue_warehouse_enter_pos)
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetLowerToPickup, () -> {  // Begin Lowering DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                    marvyn.Wristy.setPosition(marvyn.Wrist_Pickup_Pos);
//                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, marvyn.Dawinchi_pickup_ticks);
//                    marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_TO_POSITION);
//                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0.5);
//                    while(opModeIsActive() && marvyn.areMotorsBusy(Mrv_Robot.MrvMotors.DA_WINCHI)) {
//                        idle();
//                    }
//                })
//                .waitSeconds(dWarehouseLevelToPickup)
//
//                // Step2: Enter Warehouse, Pickup freight, drop it off and return to Warehouse enter pos
//                .lineToLinearHeading(marvyn.blue_warehouse_pos1, marvyn.mecanumDrive.getVelocityConstraint(marvyn.slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(marvyn.slower_accel))
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetIntakeFreight, () -> { // Start intake 0.3 seconds before reaching warehouse_pos and keep running them for 0.5 seconds after.
//                    marvyn.Claw_Left.setPower(0.5);
//                    marvyn.Claw_Right.setPower(0.5);
//                })
//                .waitSeconds(marvyn.dIntakeFreight)
//                .addTemporalMarker(()-> {
//                    marvyn.Claw_Left.setPower(0);
//                    marvyn.Claw_Right.setPower(0);
//                })
//                .lineToLinearHeading(marvyn.blue_warehouse_enter_pos, marvyn.mecanumDrive.getVelocityConstraint(marvyn.slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(marvyn.slower_accel))
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetRaiseToDropOff, () -> { // Begin Raising DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                    marvyn.Wristy.setPosition(marvyn.Wrist_Dropoff_Pos);
//                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, -1* marvyn.Dawinchi_Level2_dropoff_ticks);
//                    marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_TO_POSITION);
//                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI,-0.5);
//                    while(opModeIsActive() && marvyn.areMotorsBusy(Mrv_Robot.MrvMotors.DA_WINCHI)) {
//                        idle();
//                    }
//                })
//                .waitSeconds(marvyn.dRaiseToLevel2)
//
//                .lineToLinearHeading(marvyn.blue_2_shipping_hub_pos)
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetEjectFreight,() -> {  // Runs 0.1 seconds into the WaitSeconds call and keeps the servos on for 0.4s
//                    marvyn.Claw_Left.setPower(-0.5);
//                    marvyn.Claw_Right.setPower(-0.5);
//                })
//                .waitSeconds(marvyn.dEjectFreight)
//                .addTemporalMarker(() -> {
//                    marvyn.Claw_Left.setPower(0);
//                    marvyn.Claw_Right.setPower(0);
//                })
//                .lineToLinearHeading(marvyn.blue_warehouse_enter_pos)
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetLowerToPickup, () -> {  // Begin Lowering DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                    marvyn.Wristy.setPosition(marvyn.Wrist_Pickup_Pos);
//                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, marvyn.Dawinchi_pickup_ticks);
//                    marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_TO_POSITION);
//                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI,0.5);
//                    while(opModeIsActive() && marvyn.areMotorsBusy(Mrv_Robot.MrvMotors.DA_WINCHI)) {
//                        idle();
//                    }
//                })
//                .waitSeconds(dWarehouseLevelToPickup)
//
//                // Step 3: Enter Warehouse, Pickup freight, drop it off and return to Warehouse enter pos
//                .lineToLinearHeading(marvyn.blue_warehouse_pos2, marvyn.mecanumDrive.getVelocityConstraint(marvyn.slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(marvyn.slower_accel))
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetIntakeFreight, () -> { // Start intake 0.3 seconds before reahing warehouse_pos and keep running them for 0.5 seconds after.
//                    marvyn.Claw_Left.setPower(0.5);
//                    marvyn.Claw_Right.setPower(0.5);
//                })
//                .waitSeconds(marvyn.dIntakeFreight)
//                .addTemporalMarker(()-> {
//                    marvyn.Claw_Left.setPower(0);
//                    marvyn.Claw_Right.setPower(0);
//                })
//                .lineToLinearHeading(marvyn.blue_warehouse_enter_pos, marvyn.mecanumDrive.getVelocityConstraint(marvyn.slower_speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), marvyn.mecanumDrive.getAccelerationConstraint(marvyn.slower_accel))
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetRaiseToDropOff, () -> { // Begin Raising DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                    marvyn.Wristy.setPosition(marvyn.Wrist_Dropoff_Pos);
//                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, -1* marvyn.Dawinchi_Level2_dropoff_ticks);
//                    marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_TO_POSITION);
//                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI,-0.5);
//                    while(opModeIsActive() && marvyn.areMotorsBusy(Mrv_Robot.MrvMotors.DA_WINCHI)) {
//                        idle();
//                    }
//                })
//                .waitSeconds(marvyn.dRaiseToLevel2)
//
//                .lineToLinearHeading(marvyn.blue_2_shipping_hub_pos)
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetEjectFreight,() -> {  // Runs 0.1 seconds into the WaitSeconds call and keeps the servos on for 0.4s
//                    marvyn.Claw_Left.setPower(-0.5);
//                    marvyn.Claw_Right.setPower(-0.5);
//                })
//                .waitSeconds(marvyn.dEjectFreight)
//                .addTemporalMarker(() -> {
//                    marvyn.Claw_Left.setPower(0);
//                    marvyn.Claw_Right.setPower(0);
//                })
//                .lineToLinearHeading(marvyn.blue_warehouse_enter_pos)
//                .UNSTABLE_addTemporalMarkerOffset(marvyn.offsetLowerToPickup, () -> {  // Begin Lowering DaWinchi 0.5 seconds before reaching warehouse enter pos and keep it running for 0.5 s after (total time = 1s)
//                    marvyn.Wristy.setPosition(marvyn.Wrist_Pickup_Pos);
//                    marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, marvyn.Dawinchi_pickup_ticks);
//                    marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_TO_POSITION);
//                    marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI,0.5);
//                    while(opModeIsActive() && marvyn.areMotorsBusy(Mrv_Robot.MrvMotors.DA_WINCHI)) {
//                        idle();
//                    }
//                })
//                .waitSeconds(dWarehouseLevelToPickup)

                // Step 4: Go to park position
                .lineToLinearHeading(marvyn.blue_warehouse_pos)
                .lineToLinearHeading(marvyn.blue_warehouse_park_pos)

                .build()
        ;
        mrvDashboardTelemetryPacket.addLine(String.format("%d. Trjectory Duration after build: %.3f", iTeleCt++, mrvBlue2.duration()));
        return;

    }

    void initMotorsAndServos(boolean run_to_position)
    {
        // Reset Dawinchi - current position becomes 0
        marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, STOP_AND_RESET_ENCODER);
        marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_WITHOUT_ENCODER);

        // Reset Linac - current position becomes 0
        marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, STOP_AND_RESET_ENCODER);
        marvyn.setRunMode(Mrv_Robot.MrvMotors.LIN_AC, RUN_WITHOUT_ENCODER);

        marvyn.Wristy.setPosition(marvyn.Wrist_Dropoff_Pos);
    }

    void DawinchiToDropoffPosition(int iLevel )
    {
        if(iLevel == 0)
        {
            marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level0_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        }
        else if(iLevel == 1)
        {
            marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level1_Dropoff_pos*marvyn.Dawinchi_Ticks_Per_Rev);
        }
        else if(iLevel == 2)
        {
            marvyn.Dawinchi_initial_dropoff_ticks = (int) (marvyn.DaWinchi_Level2_Dropoff_Pos*marvyn.Dawinchi_Ticks_Per_Rev);
        }

        // Lower Dawinchi
        int iDawinchiDropoffPos = -1* marvyn.Dawinchi_initial_dropoff_ticks;
        double dDawinchiPower = -1*marvyn.DaWinchi_Power;
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        double timeTaken = 0;
        int iCurrentPos = 0;

        if(!marvyn.profileUsingRunToPosition) {
            timer.reset();
            marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, dDawinchiPower);
            while (opModeIsActive() && marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI) > iDawinchiDropoffPos) {
                idle();
            }
            marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, 0);
        }
        else {
            marvyn.setTargetPosition(Mrv_Robot.MrvMotors.DA_WINCHI, iDawinchiDropoffPos);
            marvyn.setRunMode(Mrv_Robot.MrvMotors.DA_WINCHI, RUN_TO_POSITION);
            timer.reset();
            marvyn.setPower(Mrv_Robot.MrvMotors.DA_WINCHI, dDawinchiPower);
            while (opModeIsActive() && marvyn.areMotorsBusy(Mrv_Robot.MrvMotors.DA_WINCHI)) {
                idle();
            }
        }
        timeTaken = timer.seconds();
        iCurrentPos = marvyn.getCurrentPosition(Mrv_Robot.MrvMotors.DA_WINCHI);

        mrvDashboardTelemetryPacket.addLine(String.format("%d. Raise Winch to level: %d, position %d: iCurrentPos %d, Time taken: %.3f s", iTeleCt++, iLevel, iDawinchiDropoffPos, iCurrentPos, timeTaken));
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

}

