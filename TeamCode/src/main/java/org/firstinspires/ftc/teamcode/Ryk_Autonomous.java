package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
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
public class Ryk_Autonomous extends LinearOpMode {

    enum MrvAllianceField {
        RED,
        BLUE
    }

    Ryk_Robot Mavryk = new Ryk_Robot();

    //    private static final String[] TFOD_MODEL_LABELS =
//            {
//                    "Aztechs_TSE"
//            };
    private static final String[] TFOD_MODEL_LABELS =
            {
                    "TSE"
            };

    private static FtcDashboard rykRobot;
    private static VuforiaLocalizer rykVuforia;
    private static TFObjectDetector rykTfod;

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

    double SlideHigh = (Mavryk.Slide_High_Revs * Mavryk.Slide_Ticks_Per_Rev);
    double SlidePickup = (Mavryk.Slide_Min_Pickup_Revs * Mavryk.Slide_Ticks_Per_Rev);
    double SlideRest = (Mavryk.Slide_rest * Mavryk.Slide_Ticks_Per_Rev);




    // Warehouse drop off level
    private static int rykParkPosition;
    private static double dWarehouseLevelToPickup;





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

        // init VUFORIA
        VuforiaLocalizer.Parameters vuParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuParams.cameraName = Mavryk.eyeOfSauron;
        vuParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuParams.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        vuParams.useExtendedTracking = false;
        rykVuforia = ClassFactory.getInstance().createVuforia(vuParams);

//        // init Tensorflow
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.4f;
//        tfodParameters.isModelTensorFlow2 = true;
//        tfodParameters.inputSize = TFodResolution;
//        rykTfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, rykVuforia);
//        rykTfod.loadModelFromAsset(TFOD_MODEL_ASSET, TFOD_MODEL_LABELS);
//
//        telemetry.addLine(String.format("%d. Tensorflow assets loaded", iTeleCt++));
//
//        // Startup camera
//        rykRobot.startCameraStream(rykTfod, 0); // start streaming camera
//        telemetry.addLine(String.format("%d. TFod Camera Stream started", iTeleCt++));
//
//        if (rykTfod != null) {
//            rykTfod.activate();
//            rykTfod.setZoom(TFodZoomFactor, 16.0 / 9.0);
//        }
//        sleep(1000);
//        telemetry.addLine(String.format("%d. Tfod activated! Ready to Start!", iTeleCt++));
//
//        telemetry.update();

        waitForStart();

        // 1. Calculate Parking Position
        rykParkPosition = RykGetParkingPosition(MrvAllianceField.BLUE);
        telemetry.addLine(String.format("%d. Detected Warehouse Level: %d", iTeleCt++, rykParkPosition));
        telemetry.update();

        initMotorsAndServos(true);
        buildWarehouseTrajectories_position();

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
        DawinchiToDropoffPosition(rykParkPosition);

        // 5. Pickup cone
        Pose2d startPose;
        startPose = Mavryk.blue_2_pose_estimate;
        Mavryk.mecanumDrive.setPoseEstimate(startPose);

        // 6. Drive back to pole

        // 7. repeat steps 3-6 for cycling

        // 8. park in spot

        telemetry.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
        telemetry.addLine(String.format("%d. Initial Pose Estimate: (x: %.3f, y: %.3f, Heading: %.3f)", iTeleCt++, startPose.getX(), startPose.getY(), startPose.getHeading()));
        trajectoryTimer.reset();

        Mavryk.mecanumDrive.followTrajectorySequence(mrvBlue2);

        telemetry.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.seconds()));
        telemetry.addLine(String.format("%d. Total time to complete Trajectory Sequence: %.3f ", iTeleCt++, trajectoryTimer.seconds()));

        telemetry.update();
    }

    int RykGetParkingPosition(MrvAllianceField field) {
        // 1 = red, 2 = green, 3 = blue
        int BeaconPos = 1;
        dWarehouseLevelToPickup = Mavryk.dFromLevel0ToPickup;
        Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level0_Dropoff_pos* Mavryk.Dawinchi_Ticks_Per_Rev * -1);
        Mavryk.Dawinchi_pickup_ticks = (int) (Mavryk.DaWinchi_pickup_pos* Mavryk.Dawinchi_Ticks_Per_Rev);
        Mavryk.Dawinchi_Level2_dropoff_ticks = (int) (Mavryk.DaWinchi_Level2_Dropoff_Pos* Mrv_Robot.Dawinchi_Ticks_Per_Rev);

        //TODO: replace with EasyOpenCV
        if (rykTfod != null && opModeIsActive()) {
            boolean bDuckFound = false;
            float left = 0.0f;
            float top = 0.0f;
            float bottom = 0.0f;
            float right = 0.0f;
            float mdpt = 0.0f;
            List<Recognition> updatedRecognition = rykTfod.getUpdatedRecognitions();
            if (updatedRecognition != null) {
                telemetry.addLine(String.format("%d. # Objects detected: %d", iTeleCt++, updatedRecognition.size()));
                telemetry.addLine(String.format("%d. # Objects detected: %d", iTeleCt++, updatedRecognition.size()));
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

                    telemetry.addLine(String.format("%d. Object (%d:) ", iTeleCt++, i) + recognition.getLabel());
                    telemetry.addLine(String.format("%d. Confidence: %.3f", iTeleCt++, recognition.getConfidence()));
                    telemetry.addLine(String.format("%d. BBox (%d), (%.03f, %.03f)  (%.03f, %.03f)", iTeleCt++, i, left, top, right, bottom));
                    telemetry.addLine(String.format("%d. BBox midpt (%d),  (%.03f)", iTeleCt++, i, mdpt));
                    telemetry.addLine(String.format("%d. Image Size (%d), Width: %d; Height: %d ", iTeleCt++, i, recognition.getImageWidth(), recognition.getImageHeight()));

                    //if (recognition.getLabel()== "Duck") {
                    if (recognition.getLabel() == "TSE" && recognition.getConfidence() > 0.8) {
                        if (Mavryk.FirstPosMin <= mdpt && mdpt <= Mavryk.FirstPosMax) {
                            BeaconPos = 1;
                            dWarehouseLevelToPickup = Mavryk.dFromLevel1ToPickup;
                            Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level1_Dropoff_pos* Mavryk.Dawinchi_Ticks_Per_Rev * -1);
                        } else if (Mavryk.SecPosMin <= mdpt && mdpt <= Mavryk.SecPosMax) {
                            BeaconPos = 2;
                            dWarehouseLevelToPickup = Mavryk.dFromLevel2ToPickup;
                            Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level2_Dropoff_Pos* Mavryk.Dawinchi_Ticks_Per_Rev * -1);
                        }
                        i++;
                        break;
                    }
                }
            } else {
                telemetry.addLine(String.format("%d. Objects Detected: None!", iTeleCt++));
                telemetry.addLine(String.format("%d. Objects Detected: None!", iTeleCt++));
                telemetry.update();
                BeaconPos = 0;
                dWarehouseLevelToPickup = Mavryk.dFromLevel0ToPickup;
                Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level0_Dropoff_pos* Mavryk.Dawinchi_Ticks_Per_Rev * -1);

            }
        }
        telemetry.addLine(String.format("%d. BeaconPos: %d", iTeleCt++, BeaconPos));

        telemetry.update();

        return BeaconPos;
    }


    void buildWarehouseTrajectories_position() {

        telemetry.addLine(String.format("%d. buildWarehouseTrajectories_position", iTeleCt++));

        mrvBlue2 = Mavryk.mecanumDrive.trajectorySequenceBuilder(Mavryk.blue_2_pose_estimate)

        // 2. Move to dropoff pole

                .lineToLinearHeading(new Pose2d(12,60, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(-90)))


        // 3. Extend Slides + dropoff
                .addDisplacementMarker(()  -> {

                    //raise slides
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, Mavryk.dSlidePower);
                    while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) < SlideHigh) {
                        idle();
                    }
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);

                    //open claw
                    Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                })

                .lineToLinearHeading(new Pose2d(16,8, Math.toRadians(-45)))
                .waitSeconds(0.2)
                //close claw
                .addDisplacementMarker(() -> {
                    Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Close_Pos);
                })
                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(0)))
                .addDisplacementMarker(()  -> {

                    //lower slides
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, -Mavryk.dSlidePower);
                    while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) > SlideRest) {
                        idle();
                    }
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);
                })





        // 4. Drive to cone stack
                .lineToLinearHeading(new Pose2d(60,12, Math.toRadians(0)))

        // 5. Pickup cone

                //raise to pickup and open claw
                .addDisplacementMarker(()  -> {
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, Mavryk.dSlidePower);
                    while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) < SlidePickup) {
                        idle();
                    }
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);

                    //open claw
                    Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                })

                //drive to cone
                .lineToLinearHeading(new Pose2d(60,12, Math.toRadians(0)))

                //close claw
                .addDisplacementMarker(()  -> {
                    //close claw
                    Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Close_Pos);
                })

        // 6. Drive back to pole
                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(0)))
        // 7. repeat steps 3-6 for cycling
                .addDisplacementMarker(()  -> {

                    //raise slides
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, Mavryk.dSlidePower);
                    while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) < SlideHigh) {
                        idle();
                    }
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);

                    //open claw
                    Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Open_Pos);
                })

                .lineToLinearHeading(new Pose2d(16,8, Math.toRadians(-45)))
                .waitSeconds(0.2)
                //close claw
                .addDisplacementMarker(() -> {
                    Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Ryk_Robot.Claw_Close_Pos);
                })
                .lineToLinearHeading(new Pose2d(12,12, Math.toRadians(0)))
                .addDisplacementMarker(()  -> {

                    //lower slides
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, -Mavryk.dSlidePower);
                    while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) > SlideRest) {
                        idle();
                    }
                    Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);
                })

                // 8. park in spot

                .build()
        ;
        telemetry.addLine(String.format("%d. Trjectory Duration after build: %.3f", iTeleCt++, mrvBlue2.duration()));
        return;

    }

    void initMotorsAndServos(boolean run_to_position)
    {
        // Reset Slides - current position becomes 0
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, STOP_AND_RESET_ENCODER);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_WITHOUT_ENCODER);

    }

//    void DawinchiToDropoffPosition(int iLevel )
//    {
//        if(iLevel == 0)
//        {
//            Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level0_Dropoff_pos* Mavryk.Dawinchi_Ticks_Per_Rev);
//        }
//        else if(iLevel == 1)
//        {
//            Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level1_Dropoff_pos* Mavryk.Dawinchi_Ticks_Per_Rev);
//        }
//        else if(iLevel == 2)
//        {
//            Mavryk.Dawinchi_initial_dropoff_ticks = (int) (Mavryk.DaWinchi_Level2_Dropoff_Pos* Mavryk.Dawinchi_Ticks_Per_Rev);
//        }
//
//        // Lower Dawinchi
//        int iDawinchiDropoffPos = -1* Mavryk.Dawinchi_initial_dropoff_ticks;
//        double dDawinchiPower = -1* Mavryk.DaWinchi_Power;
//        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
//        double timeTaken = 0;
//        int iCurrentPos = 0;
//
//        if(!Mavryk.profileUsingRunToPosition) {
//            timer.reset();
//            Mavryk.setPower(Ryk_Robot.RykMotors.DA_WINCHI, dDawinchiPower);
//            while (opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.DA_WINCHI) > iDawinchiDropoffPos) {
//                idle();
//            }
//            Mavryk.setPower(Ryk_Robot.RykMotors.DA_WINCHI, 0);
//        }
//        else {
//            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.DA_WINCHI, iDawinchiDropoffPos);
//            Mavryk.setRunMode(Ryk_Robot.RykMotors.DA_WINCHI, RUN_TO_POSITION);
//            timer.reset();
//            Mavryk.setPower(Ryk_Robot.RykMotors.DA_WINCHI, dDawinchiPower);
//            while (opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.DA_WINCHI)) {
//                idle();
//            }
//        }
//        timeTaken = timer.seconds();
//        iCurrentPos = Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.DA_WINCHI);
//
//        telemetry.addLine(String.format("%d. Raise Winch to level: %d, position %d: iCurrentPos %d, Time taken: %.3f s", iTeleCt++, iLevel, iDawinchiDropoffPos, iCurrentPos, timeTaken));
//    }

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


