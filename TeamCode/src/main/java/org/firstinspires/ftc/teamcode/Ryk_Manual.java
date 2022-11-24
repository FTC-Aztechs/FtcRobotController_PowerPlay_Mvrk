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
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import static org.firstinspires.ftc.teamcode.Ryk_Robot.IntakeDrivePos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.IntakePos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Ryk_Manual", group="Manual mode")
//@Disabled
public class Ryk_Manual extends LinearOpMode {

    // Declare OpMode members.
    Ryk_Robot Mavryk = new Ryk_Robot();
//
//    static final double INCREMENT = 0.1;     // amount to slew servo each CYCLE_MS cycle
//    static final double[] RANGE_FULL = {0.0, 1.0};

    public static double speedAdjust = 5;
    public static double UpAdjust = 10;
    private boolean changingWheelSpeed = false;
    private boolean changingSlideSpeed = false;

    public static int HighJunction = 1080;
    public static int MidJunction = 800;
    public static int LowJunction = 500;
    public static int GroundJunction = 100;
    public static int FloorPosition = 10;
    public static double SlidePower_Up= 1;
    public static double SlidePower_Down = 0.5;
    public static int ticks_stepSize = 100;
    public static int Mode = 1;
    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position
    double Intake_Position;
    double Sweeper_Power;


    //    private static ElapsedTime timer_gp1_buttonA;
//    private static ElapsedTime timer_gp1_buttonX;
//    private static ElapsedTime timer_gp1_buttonY;
//    private static ElapsedTime timer_gp1_buttonB;
//    private static ElapsedTime timer_gp1_dpad_up;
//    private static ElapsedTime timer_gp1_dpad_down;
    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
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

    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;
    private int currPos = FloorPosition;


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
            PpManualDrive();
            rykUpSlide();
            rykClaw();
            rykIntake();
        }
    }

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = rykDashboard;

        Claw_Position = Mavryk.Claw_Close_Pos;

        Intake_Position = IntakeDrivePos;

        Mavryk.setPosition(Ryk_Robot.RykServos.FUNKY_MONKEY, Intake_Position);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status:", "Robot is ready to roll!");
        telemetry.update();

        return;
    }


    public void rykIntake() {
        boolean bIntake = gamepad1.right_trigger == 1f;

        if (bIntake) {
            Sweeper_Power = 0.5;
            Intake_Position = IntakePos;
        }
        else {
            Sweeper_Power = 0;
            Intake_Position = IntakeDrivePos;
        }

        Mavryk.setCRPower(Ryk_Robot.RykServos.CAR_WASH, Sweeper_Power);
        Mavryk.setPosition(Ryk_Robot.RykServos.FUNKY_MONKEY, Intake_Position);

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
        Mavryk.lower_left.setPower((moveDir + strafeDir - turnDir) * (-speedAdjust / 10)); // 1.0
        Mavryk.lower_right.setPower((moveDir - strafeDir + turnDir) * (-speedAdjust / 10)); // 1.0
        Mavryk.upper_left.setPower((moveDir - strafeDir - turnDir) * (-speedAdjust / 10)); // 0
        Mavryk.upper_right.setPower((moveDir + strafeDir + turnDir) * (-speedAdjust / 10)); // 0

        return;
    }

    public void rykClaw() {
        ServoTurn = gamepad2.right_trigger == 1f;


        if (ServoTurn) {
            Claw_Position = Mavryk.Claw_Open_Pos;
        } else {
            Claw_Position = Mavryk.Claw_Close_Pos;
        }

        Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Claw_Position);

    }

    public void rykUpSlide_rue() {
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

        int newPos = currPos + (int) ( -gamepad2.left_stick_y * ticks_stepSize);
        if( newPos >= HighJunction)
            newPos = HighJunction;
        else if (newPos <= FloorPosition)
            newPos = FloorPosition;
        telemetry.addData("newPos calc from gamePad2.left_stick_y: ", newPos);
        telemetry.update();

        if (gamepad2.y) {
            if (!assumingHighPosition) {
                timer_gp2_buttonY.reset();
                assumingHighPosition = true;
            } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_Y triggered. Set Tom&Jerry to High position");
                telemetry.update();

                newPos = HighJunction;
                assumingHighPosition = false;
            }
        }

        if (gamepad2.x) {
            if (!assumingMidPosition) {
                timer_gp2_buttonX.reset();
                assumingMidPosition = true;
            } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_X triggered. Set Tom&Jerry to Mid position");
                telemetry.update();
                newPos = MidJunction;
                assumingMidPosition = false;
            }
        }

        if(gamepad2.a) {
            if (!assumingLowPosition) {
                timer_gp2_buttonY.reset();
                assumingLowPosition = true;
            } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_A triggered. Set Tom&Jerry to Low position");
                telemetry.update();

                newPos = LowJunction;
                assumingLowPosition = false;
            }
        }

        if (gamepad2.b) {
            if (!assumingFloorPosition) {
                timer_gp2_buttonB.reset();
                assumingFloorPosition = true;
            } else if (timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_B triggered. Set Tom&Jerry to Floor position");
                telemetry.update();

                newPos = FloorPosition;
                assumingFloorPosition = false;
            }
        }


        telemetry.addData("newPos from Any button triggers: ", newPos);
        telemetry.update();

        if( newPos != currPos && newPos >=FloorPosition && newPos <= HighJunction ) {
            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, newPos);
            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
            if (newPos > currPos) {
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                while (opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                    idle();
                }
            }
            else if (newPos < currPos) {
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                while (opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                    idle();
                }
            }
            currPos = newPos;
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_rue: Current Slide Position: ", Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
        telemetry.update();
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
    }
}









