/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;


public class Ryk_Robot
{
    enum RykMotors
    {
        UPPER_LEFT,
        LOWER_LEFT,
        UPPER_RIGHT,
        LOWER_RIGHT,
        CAT_MOUSE,
        ALL_DRIVES,
        ALL_ATTACHMENTS,
        ALL
    }

    enum RykServos
    {
        HANDSEL,
        GRABBEL,
        TWIN_TOWERS,
        IntakeLeft,
        IntakeRight
    }


    /* Public OpMode members. */
    public DcMotor upper_right = null;
    public DcMotor upper_left = null;
    public DcMotor lower_left = null;
    public DcMotor lower_right = null;

    public DcMotor Jerry = null;
    public DcMotor Tom = null;

    //    public DcMotor duck_wheel = null;
//    public DcMotor another_duck_wheel = null;
//    public DcMotor Linac = null;
//    public DcMotor Da_Winch = null;
//    //public Servo The_Claw = null;
    public Servo Handsel = null;
    public Servo Grabbel = null;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public WebcamName eyeOfSauron = null;
    OpenCvWebcam Sauron = null;

//    public DigitalChannel Touche_Linac = null;
//    public DigitalChannel Touche_Winch = null;

//    Orientation angles;
//
    public static int stallDetectionThreshold = 500;
    public static ElapsedTime HandselClawTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static ElapsedTime GrabbelClawTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public static double HandselClawLastPos = 0.0f;
    public static double GrabbelClawLastPos = 0.0f;

    public static double UpAdjust = 10;
    public static int HighJunction = 1100;
    public static int MidJunction = 800;
    public static int LowJunction = 450;
    public static int GroundJunction = 100;
    public static int FloorPosition = 10;
    public static double SlidePower_Up= 1;
    public static double SlidePower_Down = 0.3;
    public static int ticks_stepSize = 100;
    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    public static final String VUFORIA_KEY =
            "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    public static final float halfField        = 72 * mmPerInch;
    public static final float halfTile         = 12 * mmPerInch;
    public static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    public static OpenGLMatrix lastLocation   = null;
    public static VuforiaLocalizer vuforia    = null;
    public static VuforiaTrackables targets   = null ;
    public static WebcamName webcamName       = null;

    public static boolean targetVisible       = false;


//
//    // TFOD detection
//    public static double FirstPosMax = 250;
//    public static double FirstPosMin = 50;
//    public static double SecPosMax = 600;
//    public static double SecPosMin = 400;

    // Trajectory sequencing




    public static Pose2d Red_Start = new Pose2d(40.5, 64.5, Math.toRadians(-90));
    public static Pose2d Red_Dropoff_Dodge = new Pose2d(10,64.5, Math.toRadians(-90));
    public static Pose2d Red_DropOff_Tile = new Pose2d(10,16, Math.toRadians(-90));
    public static Pose2d Red_DropOff = new Pose2d(19,7, Math.toRadians(-50));
    public static Pose2d Red_TurnToCones = new Pose2d(10,12, Math.toRadians(0));
    public static Pose2d Red_Pickup = new Pose2d(60,12, Math.toRadians(0));


    public static Pose2d Blue_Start = new Pose2d(-31.5, 64.5, Math.toRadians(-90));
    public static Pose2d Blue_Dropoff_Dodge = new Pose2d(-10,64.5, Math.toRadians(-90));
    public static Pose2d Blue_DropOff_Tile = new Pose2d(-10,16, Math.toRadians(-90));
    public static Pose2d Blue_DropOff = new Pose2d(-18,7, Math.toRadians(-130));
    public static Pose2d Blue_TurnToCones = new Pose2d(-10,12, Math.toRadians(180));
    public static Pose2d Blue_Pickup = new Pose2d(-60,12, Math.toRadians(180));

    public static Pose2d Red_Park_Pos1 = new Pose2d(60,12, Math.toRadians(0));
    public static Pose2d Red_Park_Pos2 = new Pose2d(36,12, Math.toRadians(0));
    public static Pose2d Red_Park_Pos3 = new Pose2d(13,12, Math.toRadians(0));

    public static Pose2d Blue_Park_Pos1 = new Pose2d(-13,12, Math.toRadians(180));
    public static Pose2d Blue_Park_Pos2 = new Pose2d(-36,12, Math.toRadians(180));
    public static Pose2d Blue_Park_Pos3 = new Pose2d(-60,12, Math.toRadians(180));





    //    public static int Dawinchi_Ticks_Per_Rev = 1060; // 295; // From REV Robotics HD HEX 40:1
    public static double Slide_Ticks_Per_Rev = 537.7; // From REV Robotics Core HEX
//
    double dSlidePower = 1;

//    public static double Wrist_Pickup_Pos = 0.39;
//    public static double Wrist_Dropoff_Pos = 0.355;

    public static double Claw_Open_Pos = 0.5;
    public static double Claw_Close_Pos = 0.35;
//
//    // Speed control variables
//    public static double slower_speed = 40;
//    public static double slower_accel = 40;
//
//    public static int overrideWarehouseDropoffLevel = 2;
//    public static double DaWinchi_Level0_Dropoff_pos = 0;
//    public static double DaWinchi_Level1_Dropoff_pos = 0.37;
//    public static double DaWinchi_Level2_Dropoff_Pos = 0.8;
//    public static double DaWinchi_pickup_pos = 0.295;
//    public static int Dawinchi_initial_dropoff_ticks = 0;
//    public static int Dawinchi_Level2_dropoff_ticks = 0;
//    public static int    Dawinchi_pickup_ticks = 0;
//
//    public static double DaWinchi_Power = 0.5;
//
    public static double Slide_Ground_Revs = 0.7;
    public static double Slide_Low_Revs = 0.7;
    public static double Slide_Mid_Revs = 0.7;
    public static double Slide_High_Revs = 0.7;
    public static double Slide_rest = 0;

    public static double Slide_Min_Pickup_Revs = 0.9;
    public static double Slide_increment_Pickup_Revs = 0.9;

    public static double auto_drop_wait = 1;

    static int[] red = {135, 180, 0, 10};
    static int[] green = {70, 90};
    static int[] blue = {100, 120};



//    public static double dFromLevel0ToPickup = 0.2;    // 1. Time to lower from level 0 -> Pickup
//    public static double dFromLevel1ToPickup = 0.3;    // 2. Time to lower from level 1 -> Pickup
//    public static double dFromLevel2ToPickup = 0.36;    // 3. Time to lower from level 2 -> Pickup
//    public static double dRaiseToLevel2      = 0.81;    // 4. Time to raise from Pickup -> level2
//    public static double dEjectFreight       = 1;    // 5. Time to drop off freight element
//    public static double dIntakeFreight      = 0.4;    // 6. Time to pick up freight element
//    public static double offsetEjectFreight  = -0.1;    // 7. Offset to eject freight after reaching shipping hub pos
//    public static double offsetIntakeFreight = -0.2;    // 8. Offset to pickup freight before reaching warehouse pos
//    public static double offsetLowerToPickup = -0.3;    // 9. Offset to lower to pickup position (Start to do this after retracting from drop-off position)
//    public static double offsetRaiseToDropOff = -0.2;   // 10. Offset to raise to drop-off position (Start to do this after retracting from pickup position)
//
//    //Ryk_autonomous debugging variables
//    public static boolean runProfiling = false;
//    public static boolean profileUsingRunToPosition = true;
//    public static boolean runPositionTrajectories = true;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    SampleMecanumDrive mecanumDrive;

    /* Constructor */
    public Ryk_Robot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        upper_right  = hwMap.get(DcMotor.class, "Upper_Right");
        upper_left = hwMap.get(DcMotor.class, "Upper_Left");
        lower_left = hwMap.get(DcMotor.class, "Lower_Left");
        lower_right = hwMap.get(DcMotor.class, "Lower_Right");

        Jerry = hwMap.get(DcMotor.class, "Jerry");
        Tom = hwMap.get(DcMotor.class, "Tom");

        //Servo
        Handsel = hwMap.get(Servo.class, "Handsel");
        Grabbel = hwMap.get(Servo.class, "Grabbel");
        intakeLeft = hwMap.get(CRServo.class, "SweeperLeft");
        intakeRight = hwMap.get(CRServo.class, "SweeperRight");

        // Acquire gyro

        //get touch sensor
//        Touche_Linac = hwMap.get(DigitalChannel.class, "Touche");
//        Touche_Winch = hwMap.get(DigitalChannel.class, "Touche2");



        // Set all motors to zero power
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Jerry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Tom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        upper_left.setDirection(DcMotor.Direction.REVERSE);  //-
        upper_right.setDirection(DcMotor.Direction.FORWARD); //+
        lower_left.setDirection(DcMotor.Direction.REVERSE); //- used to be
        lower_right.setDirection(DcMotor.Direction.FORWARD); //+ used to be
        Jerry.setDirection(DcMotor.Direction.FORWARD); //- used to be
        Tom.setDirection(DcMotor.Direction.REVERSE); //+ used to be
        Handsel.setDirection(Servo.Direction.REVERSE);
        Grabbel.setDirection(Servo.Direction.FORWARD);

        mecanumDrive = new SampleMecanumDrive(hwMap);
        eyeOfSauron = hwMap.get(WebcamName.class, "Sauron");

//        Touche_Linac.setMode(DigitalChannel.Mode.INPUT);
//        Touche_Winch.setMode(DigitalChannel.Mode.INPUT);

    }
    String formatAngle( AngleUnit angleUnit, double angle) {
        return formatDegrees(angleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hwMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    public void setRunMode(RykMotors eWhichMotor, DcMotor.RunMode eMode )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setMode(eMode);
                break;
            case UPPER_RIGHT:
                upper_right.setMode(eMode);
                break;
            case LOWER_LEFT:
                lower_left.setMode(eMode);
                break;
            case LOWER_RIGHT:
                lower_right.setMode(eMode);
                break;
            case CAT_MOUSE:
                Jerry.setMode(eMode);
                Tom.setMode(eMode);
                break;
            case ALL_DRIVES:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                break;
            case ALL_ATTACHMENTS:
                break;
            case ALL:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                Jerry.setMode(eMode);
                Tom.setMode(eMode);
                break;
        }
    }

    public void setPower(RykMotors eWhichMotor, double dPower )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setPower(dPower);
                break;
            case UPPER_RIGHT:
                upper_right.setPower(dPower);
                break;
            case LOWER_LEFT:
                lower_left.setPower(dPower);
                break;
            case LOWER_RIGHT:
                lower_right.setPower(dPower);
                break;
            case CAT_MOUSE:
                Jerry.setPower(dPower);
                Tom.setPower(dPower);
                break;
            case ALL_DRIVES:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                break;
            case ALL:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                Jerry.setPower(dPower);
                Tom.setPower(dPower);
                break;
        }
    }

    public int getCurrentPosition( RykMotors eWhichMotor )
    {
        switch(eWhichMotor)
        {
            case UPPER_LEFT:
                return upper_left.getCurrentPosition();
            case LOWER_LEFT:
                return lower_left.getCurrentPosition();
            case UPPER_RIGHT:
                return upper_right.getCurrentPosition();
            case LOWER_RIGHT:
                return lower_right.getCurrentPosition();
            case CAT_MOUSE:
                return Jerry.getCurrentPosition();
            default:
                return 0;
        }
    }

    public void setPosition( RykServos eWhichServo, double iPos )
    {
        switch( eWhichServo)
        {
            case HANDSEL:
                Handsel.setPosition(iPos);
                break;
            case GRABBEL:
                Grabbel.setPosition(iPos);
                break;
            case TWIN_TOWERS:
                Handsel.setPosition(iPos);
                Grabbel.setPosition(iPos);
                break;
            default :
                break;
        }
    }

    public boolean areMotorsBusy(RykMotors eWhichMotor) {

        switch(eWhichMotor)
        {
            case UPPER_LEFT: // upper left
                return upper_left.isBusy();
            case LOWER_LEFT: // lower left
                return lower_left.isBusy();
            case UPPER_RIGHT: // upper right
                return upper_right.isBusy();
            case LOWER_RIGHT: // lower right
                return lower_right.isBusy();
            case CAT_MOUSE:
                return Jerry.isBusy() && Tom.isBusy();
            case ALL_DRIVES: // All Drives
                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy();
            case ALL_ATTACHMENTS:
                //return Linac.isBusy() && duck_wheel.isBusy() && Da_Winch.isBusy();
            case ALL:
                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy();
                        //&& duck_wheel.isBusy() && Linac.isBusy()&& Da_Winch.isBusy();
            default:
                return false;
        }
    }

    public void setTargetPosition( RykMotors eWhichMotor, int iPos ) {
        switch (eWhichMotor) {
            case UPPER_LEFT:
                upper_left.setTargetPosition(iPos);
                break;
            case LOWER_LEFT:
                lower_left.setTargetPosition(iPos);
                break;
            case UPPER_RIGHT:
                upper_right.setTargetPosition(iPos);
                break;
            case LOWER_RIGHT:
                lower_right.setTargetPosition(iPos);
                break;
            case CAT_MOUSE:
                Jerry.setTargetPosition(iPos);
                Tom.setTargetPosition(iPos);
                break;
            case ALL_DRIVES:
                lower_right.setTargetPosition(iPos);
                lower_left.setTargetPosition(iPos);
                upper_right.setTargetPosition(iPos);
                upper_left.setTargetPosition(iPos);
                break;
            case ALL:
                lower_right.setTargetPosition(iPos);
                lower_left.setTargetPosition(iPos);
                upper_right.setTargetPosition(iPos);
                upper_left.setTargetPosition(iPos);
                Jerry.setTargetPosition(iPos);
                Tom.setTargetPosition(iPos);
            default:
                break;
        }
    }
}

