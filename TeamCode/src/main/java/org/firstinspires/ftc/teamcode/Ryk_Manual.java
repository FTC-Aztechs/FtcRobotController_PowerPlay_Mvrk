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
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Ryk_Manual", group="Manual mode")
//@Disabled
public class Ryk_Manual extends LinearOpMode {
    // Declare OpMode members.
    Ryk_Robot powerslay = new Ryk_Robot();
//
//    static final double INCREMENT = 0.1;     // amount to slew servo each CYCLE_MS cycle
//    static final double[] RANGE_FULL = {0.0, 1.0};

    public static double speedAdjust = 5;
    //    public static double linacAdjust = 10;
    public static double UpAdjust = 5;
    //    public static double dawinchAdjust = 6;
//    private boolean assumingPickPosition = false;
//    private boolean assumingDropPosition = false;
    private boolean changingWheelSpeed = false;
    private boolean changingLinacSpeed = false;
//    private boolean changingDaWinchiSpeed = false;
//    private boolean changingWrist = false;

//    public static double duck_power = 0.29;
//
    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position

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
    public static double Claw_Open_Pos = 0.6;
    public static double Claw_Close_Pos = 0.5;
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
    private static ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
//    ;
//    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);


//    int DuckPowerDir = 1;
//    public static final String VUFORIA_LICENSE_KEY = "AZRnab7/////AAABmTUhzFGJLEyEnSXEYWthkjhGRzu8klNOmOa9WEHaryl9AZCo2bZwq/rtvx83YRIgV60/Jy/2aivoXaHNzwi7dEMGoaglSVmdmzPF/zOPyiz27dDJgLVvIROD8ww7lYzL8eweJ+5PqLAavvX3wgrahkOxxOCNeKG9Tl0LkbS5R11ATXL7LLWeUv5FP1aDNgMZvb8P/u96OdOvD6D40Nf01Xf+KnkF5EXwNQKk1r7qd/hiv9h80gvBXMFqMkVgUyogwEnlK2BfmeUhGVm/99BiwwW65LpKSaLVPpW/6xqz9SyPgZ/L/vshbWgSkTB/KoERiV8MsW79RPUuQS6NTOLY32I/kukmsis3MFst5LP/d3gx";

    //boolean DuckOn = false;
    FtcDashboard rykDashboard;
    @Override
    public void runOpMode() {

        // Initialize the drive system vriables
        powerslay.init(hardwareMap);

        initMavryk();
        waitForStart();


        while (opModeIsActive()) {
            PpManualDrive();
//            mrvDuckWheel();
            rykUpSlide();
            rykClaw();

//            mrvLinAc();
//            mrvDaWinchi();
//            mrvWrist();
//            mrvIntakeClaw();
//            mrvFlappy();
        }
    }

//    public void mrvIntakeClaw() {
//        boolean bIntake = gamepad2.right_trigger == 1f;
//        boolean bOuttake = gamepad2.left_trigger == 1f;
//
//        if (bIntake) {
//            marvyn.Claw_Left.setPower(0.5);
//            marvyn.Claw_Right.setPower(0.5);
//        } else if (bOuttake) {
//            marvyn.Claw_Left.setPower(-0.5);
//            marvyn.Claw_Right.setPower(-0.5);
//        } else {
//            marvyn.Claw_Left.setPower(0);
//            marvyn.Claw_Right.setPower(0);
//        }
//    }

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = rykDashboard;

        Claw_Position = Claw_Close_Pos;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status:", "Robot is ready to roll!");
        telemetry.update();

        return;
    }

    public void PpManualDrive() {
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (speedAdjust <= 1) {
                    speedAdjust = 1;
                } else {
                    speedAdjust -= 1;
                }
                telemetry.addData("Current speed: ", "%f", speedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        //gamepad right -> increase wheel speed
        if (gamepad1.dpad_right) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_right.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (speedAdjust >= 10) {
                    speedAdjust = 10;
                } else {
                    speedAdjust += 1;
                }
                telemetry.addData("Current speed: ", "%f", speedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        float turnDir = gamepad1.right_stick_x;
        float moveDir = gamepad1.left_stick_y;
        float strafeDir = gamepad1.left_stick_x;

        if (turnDir > 1) {
            turnDir = 1;
        } else if (turnDir < -1) {
            turnDir = -1;
        }
        powerslay.lower_left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        powerslay.lower_right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        powerslay.upper_left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        powerslay.upper_right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0

        return;
    }

//    public void mrvDuckWheel() {
//
//        if (gamepad2.left_bumper) {
//            DuckOn = gamepad2.left_bumper;
//            telemetry.addData("Duck Wheel toggle to:", DuckOn);
//            if (DuckOn) {
//                telemetry.addData("Duck Wheelsss:", "Spinning Clockwise");
//                telemetry.update();
//                DuckPowerDir = 1;
//            }
//            sleep(500);
//        } else if (gamepad2.right_bumper) {
//            DuckOn = gamepad2.right_bumper;
//            telemetry.addData("Duck Wheel toggle to:", DuckOn);
//            if (DuckOn) {
//                telemetry.addData("Duck Wheelsss:", "Spinning Counterclockwise");
//                telemetry.update();
//                DuckPowerDir = -1;
//            }
//            sleep(500);
//        } else {
//            DuckOn = false;
//        }
//
//        if (DuckOn) {
//            powerslay.setPower(Pp_Robot.MrvMotors.DUCK_WHEELS, duck_power * DuckPowerDir);
//        } else {
//            powerslay.setPower(Pp_Robot.MrvMotors.DUCK_WHEELS, 0);
//        }
//        return;
//    }
//
    public void rykClaw() {
        ServoTurn = gamepad2.right_trigger == 1f;
        // slew the servo, according to the rampUp (direction) variable.
//        if (gamepad2.dpad_left) {
//            if (!changingLinacSpeed) {
//                timer_gp1_dpad_left.reset();
//                changingLinacSpeed = true;
//            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                if (UpAdjust <= 1) {
//                    UpAdjust = 1;
//                } else {
//                    UpAdjust -= 1;
//                }
//                telemetry.addData("Current slides speed: ", "%f", UpAdjust);
//                telemetry.update();
//                changingLinacSpeed = false;
//            }
//        }
//
//        //gamepad right -> increase wheel speed
//        if (gamepad2.dpad_right) {
//            if (!changingLinacSpeed) {
//                timer_gp2_dpad_right.reset();
//                changingLinacSpeed = true;
//            } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                if (UpAdjust >= 10) {
//                    UpAdjust = 10;
//                } else {
//                    UpAdjust += 1;
//                }
//                telemetry.addData("Current slides speed: ", "%f", UpAdjust);
//                telemetry.update();
//                changingLinacSpeed = false;
//            }
//        }

        if (ServoTurn) {
            Claw_Position = Claw_Close_Pos;
        } else {
            Claw_Position = Claw_Open_Pos;
        }

        powerslay.Handsel.setPosition(Claw_Position);
        powerslay.Grabbel.setPosition(Claw_Position);

    }
//
//    public void mrvWrist() {
//        // if (gamepad2.left_trigger == 1f) {
//        //  Wrist_Pos = Wrist_chute_dropoff;
//        //} else
//        if (gamepad2.dpad_down) {
//            if (!changingWrist) {
//                timer_gp2_dpad_down.reset();
//                changingWrist = true;
//            } else if (timer_gp2_dpad_down.time(TimeUnit.MILLISECONDS) > Wristy_Button_Trigger_Timer_Ms) {
//                if (Wrist_Pos >= Wrist_Low_Pos) {
//                    Wrist_Pos = Wrist_Low_Pos;
//                } else {
//                    Wrist_Pos += wrist_increment;
//                }
//                telemetry.addData("Wrist Pos: ", "%f", Wrist_Pos);
//                telemetry.update();
//                changingWrist = false;
//            }
//        } else if (gamepad2.dpad_up) {
//            if (!changingWrist) {
//                timer_gp2_dpad_up.reset();
//                changingWrist = true;
//            } else if (timer_gp2_dpad_up.time(TimeUnit.MILLISECONDS) > Wristy_Button_Trigger_Timer_Ms) {
//                if (Wrist_Pos <= Wrist_chute_dropoff) {
//                    Wrist_Pos = Wrist_chute_dropoff;
//                } else {
//                    Wrist_Pos -= wrist_increment;
//                }
//                telemetry.addData("Wrist Pos: ", "%f", Wrist_Pos);
//                telemetry.update();
//                changingWrist = false;
//            }
//        }
//
//        if (gamepad2.a) {
//            if (!assumingPickPosition) {
//                timer_gp2_buttonA.reset();
//                assumingPickPosition = true;
//            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                telemetry.addLine("GP2_A triggered. Will set Wrist to pickup position");
//                telemetry.update();
//                Wrist_Pos = Wrist_Pickup_Pos;
//                //   marvyn.The_Claw.setPosition(0);
//                assumingPickPosition = false;
//            }
//        }
//
//        if (gamepad2.y) {
//            if (!assumingPickPosition) {
//                timer_gp2_buttonY.reset();
//                assumingPickPosition = true;
//            } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                telemetry.addLine("GP2_Y triggered. Will set Wrist to chute dropoff position");
//                telemetry.update();
//                Wrist_Pos = Wrist_chute_dropoff;
//                //   marvyn.The_Claw.setPosition(0);
//                assumingPickPosition = false;
//            }
//        }
//
//        if (gamepad2.x) {
//            if (!assumingPickPosition) {
//                timer_gp2_buttonX.reset();
//                assumingPickPosition = true;
//            } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                telemetry.addLine("GP2_X triggered. Will set Wrist to TSE pickup position");
//                telemetry.update();
//                Wrist_Pos = Wrist_TSE_Pickup_Pos;
//                //   marvyn.The_Claw.setPosition(0);
//                assumingPickPosition = false;
//            }
//        }
//
//        if (gamepad2.b) {
//            if (!assumingDropPosition) {
//                timer_gp2_buttonB.reset();
//                assumingDropPosition = true;
//            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                telemetry.addLine("GP2_B triggered. Will set Wrist to dropoff position");
//                telemetry.update();
//                Wrist_Pos = Wrist_Dropoff_Pos;
//                assumingDropPosition = false;
//            }
//        } else {
//            assumingDropPosition = false;
//        }
//
//
//        powerslay.Wristy.setPosition(Wrist_Pos);
//        return;
//    }
//
//
//    public void mrvFlappy() {
//        powerslay.Flappy_Bird.setPosition(((door_open - door_close) * gamepad1.right_trigger)+ door_close);
//    }
//
//    public void mrvLinAc() {
//        if (!powerslay.Touche_Linac.getState()) {
//            if (-gamepad2.left_stick_y < 0) {
//                powerslay.setPower(Pp_Robot.MrvMotors.LIN_AC, 0);
//            } else {
//                powerslay.setPower(Pp_Robot.MrvMotors.LIN_AC, -gamepad2.left_stick_y * (linacAdjust / 10));
//            }
//        } else {
//            powerslay.setPower(Pp_Robot.MrvMotors.LIN_AC, -gamepad2.left_stick_y * (linacAdjust / 10));
//        }
//    }


    public void rykUpSlide() {


        powerslay.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, -gamepad2.left_stick_y * (UpAdjust / 10));

//
//
//
//    public void mrvDaWinchi() {
//        if (!powerslay.Touche_Winch.getState()) {
//            if (-gamepad2.right_stick_y < 0) {
//                powerslay.setPower(Pp_Robot.MrvMotors.DA_WINCHI, 0);
//            } else {
//                powerslay.setPower(Pp_Robot.MrvMotors.DA_WINCHI, gamepad2.right_stick_y * (dawinchAdjust / 10));
//            }
//        }
//        else {
//            powerslay.setPower(Pp_Robot.MrvMotors.DA_WINCHI, gamepad2.right_stick_y * (dawinchAdjust / 10));
//        }
//    }
    }
}









