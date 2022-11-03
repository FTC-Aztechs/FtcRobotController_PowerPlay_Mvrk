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

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Slide Test" , group="Manual mode")
//@Disabled
public class Slide_Debugger extends LinearOpMode {
    // Declare OpMode members.
    Ryk_Robot Mavryk = new Ryk_Robot();
//
//    static final double INCREMENT = 0.1;     // amount to slew servo each CYCLE_MS cycle
//    static final double[] RANGE_FULL = {0.0, 1.0};

    public static double speedAdjust = 5;
    //    public static double linacAdjust = 10;
    public static double UpAdjust = 10;
    //    public static double dawinchAdjust = 6;
//    private boolean assumingPickPosition = false;
//    private boolean assumingDropPosition = false;
    private final boolean changingWheelSpeed = false;
    private boolean changingSlideSpeed = false;
//    private boolean changingDaWinchiSpeed = false;
//    private boolean changingWrist = false;

    //    public static double duck_power = 0.29;
//
    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position
    public static int MotorPos = 0;
    public static double SlidePower= 0.5;

    public static int Mode = 0;

//    public static double Wrist_Parallel_to_Linac = 0.425; // Parallel to arm
//    public static double Wrist_chute_dropoff = 0.26; // Perpendicular to Arm at top
//    public static double Wrist_TSE_Pickup_Pos = 0.325;
//    public static double Wrist_Pos = 0.3;
//    public static double Wrist_Start_Pos = 0.39; // Perpendicular to Arm at bottom
//    public static double Wrist_Pickup_Pos = 0.39;
//    public static double Wrist_Dropoff_Pos = 0.355;
//    public static double Wrist_Low_Pos = 0.39;
//    public static double wrist_increment = 0.015;
//    public static double door_open = 0.8;
//    public static double door_close = 0.50;

    //
    //0.52 for picking up preset
//    public static int Winch_Parallel_to_ground = 100;
//    public static int Linac_Parallel_to_ground = 100;
//    public static int Winch_Chute_Dropoff = 100;
//    public static int Linac_Chute_Dropoff = 100;
//
    public static int BUTTON_TRIGGER_TIMER_MS = 500;
    //    public static int Wristy_Button_Trigger_Timer_Ms = 85;
//    public static int Linac_Grab_Position_Ticks = 288; // From REV Robotics Core HEX
//    public static int Dawinchi_Grab_Position_Ticks = 1120; // From REV Robotics HD HEX 40:1
//
//    private static ElapsedTime timer_gp1_buttonA;
//    private static ElapsedTime timer_gp1_buttonX;
//    private static ElapsedTime timer_gp1_buttonY;
//    private static ElapsedTime timer_gp1_buttonB;
//    private static ElapsedTime timer_gp1_dpad_up;
//    private static ElapsedTime timer_gp1_dpad_down;
    private static final ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);
    //    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
//    ;
//    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);


//    int DuckPowerDir = 1;
//    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";

    //boolean DuckOn = false;
    FtcDashboard rykDashboard;
    @Override
    public void runOpMode() {

        // Initialize the drive system vriables
        Mavryk.init(hardwareMap);

        initMavryk();
        waitForStart();


        while (opModeIsActive()) {

            if (Mode == 0) {
                rykUpSlide();
            }else {
                Tester();
            }
            rykClaw();
        }
    }

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = rykDashboard;

        Claw_Position = Ryk_Robot.Claw_Close_Pos;


        Mavryk.Tom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Mavryk.Jerry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, STOP_AND_RESET_ENCODER);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_USING_ENCODER);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status:", "Robot is ready to roll!");
        telemetry.update();

        return;
    }

    public void rykClaw() {
        ServoTurn = gamepad2.right_trigger == 1f;


        if (ServoTurn) {
            Claw_Position = Ryk_Robot.Claw_Open_Pos;
        } else {
            Claw_Position = Ryk_Robot.Claw_Close_Pos;
        }

        Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Position);

    }


    public void rykUpSlide() {

        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (UpAdjust <= 1) {
                    UpAdjust = 1;
                } else {
                    UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        //Gamepad 2right -> Increase Speed
        if (gamepad2.dpad_right) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_right.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (UpAdjust >= 10) {
                    UpAdjust = 10;
                } else {
                    UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, -gamepad2.left_stick_y * (UpAdjust / 10));

        telemetry.addData("Encoder Ticks", Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
        telemetry.update();

    }

    public void Tester (){
        double SlideDir = 1;

        if (Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) <= MotorPos){
            SlideDir = 1;
        } else{
            SlideDir = -1;
        }

        if ((MotorPos - 10) < Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE)< (MotorPos+10)){
            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);

        } else {
            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlideDir* SlidePower);
        }

        telemetry.addData("Encoder Ticks", Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
        telemetry.update();

    }
}









