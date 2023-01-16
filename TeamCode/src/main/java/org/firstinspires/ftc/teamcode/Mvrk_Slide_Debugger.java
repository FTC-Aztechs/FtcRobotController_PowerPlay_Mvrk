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

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BottomMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.FloorPosition;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MidJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.TopCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.TopMidCone;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.ticks_stepSize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="MvrkSlideTester" , group="Manual mode")
//@Disabled


public class Mvrk_Slide_Debugger extends LinearOpMode {
    // Declare OpMode members.
    Mvrk_Robot Mavryk = new Mvrk_Robot();

    private boolean changingSlideSpeed = false;
    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position

    // Dashboard exposed control variables
//    public static double UpAdjust = 10;
//    public static int HighJunction = 1000;
//    public static int MidJunction = 800;
//    public static int LowJunction = 450;
//    public static int GroundJunction = 100;
//    public static int FloorPosition = 10;
//    public static int Cone1 = 20;
//    public static int Cone2 = 70;
//    public static int Cone3 = 110;
//    public static int Cone4 = 150;
//    public static int Cone5 = 190;
//    public static double SlidePower_Up= 1;
//    public static double SlidePower_Down = 0.5;
//    public static int ticks_stepSize = 100;
    public static int Mode = 3;
//    public static int BUTTON_TRIGGER_TIMER_MS = 500;


//    private static ElapsedTime timer_gp1_buttonA;
//    private static ElapsedTime timer_gp1_buttonX;
//    private static ElapsedTime timer_gp1_buttonY;
//    private static ElapsedTime timer_gp1_buttonB;
//    private static ElapsedTime timer_gp1_dpad_up;
//    private static ElapsedTime timer_gp1_dpad_down;
//    private static final ElapsedTime timer_gp1_dpad_left = new ElapsedTime(MILLISECONDS);
//    private static final ElapsedTime timer_gp1_dpad_right = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonA = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonX = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonY = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_buttonB = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
//    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp2_dpad_left = new ElapsedTime(MILLISECONDS);
    private static final ElapsedTime timer_gp2_dpad_right = new ElapsedTime(MILLISECONDS);

    FtcDashboard mvrkDashboard;
    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;

    private int currPos = 0;
    private int newPos = currPos;

    //    public enum TomJerryState {
    //        LIFT_FLOOR,
    //        LIFT_HIGH_POS,
    //        LIFT_MEDIUM_POS,
    //        LIFT_LOW_POS,
    //    }

    // AutoState Transitions:
    // FLOOR -> High = +pwr, position
    // FLOOR -> Med =  +pwr, position
    // Floor -> low =  +pwr, position
    // High -> Med =   -pwr, position
    // High -> Low =   -pwr, position
    // High -> Floor = -pwr, position
    // Med -> High = +pwr, position
    // Med -> Low = -pwr, position
    // Med -> Floor = -pwr, position
    // Low -> High = +pwr, position
    // Low -> Med = +pwr, position
    // Low -> Floor = -pwr, position
    // Floor -> Floor = No Op
    // Low -> Low = No Op
    // Med -> Med= No Op
    // High -> High = No Op

    @Override
    public void runOpMode() {

        // Initialize the drive system variables
        Mavryk.init(hardwareMap);

        initMavryk();
//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Mavryk.Claw_Open_Pos);
        waitForStart();

        if( Mode == 1 || Mode == 2) {
            Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_USING_ENCODER);
        }
        else
            Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, STOP_AND_RESET_ENCODER);
            Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_WITHOUT_ENCODER);

        telemetry.addData("Slide Debugger Running in mode: ", Mode);
        telemetry.update();

        while (opModeIsActive()) {
            if (Mode == 0) {
                mvrkUpSlide();
            } else if(Mode == 1){
                mvrkUpSlide_rue();
            } else if(Mode == 2) {
                mvrkSlideTester_rue();
            } else if(Mode == 3) {
                mvrkSlideTester_pid();
            }
            else if(Mode == 4) {
                mvrkUpSlide_rtp();
            }
            telemetry.addData("Ryk reporting voltage: ", Mavryk.getBatteryVoltage());

            //rykClaw();
        }
    }

    // Initialization

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = mvrkDashboard;
        Claw_Position = Mvrk_Robot.Claw_Open_Pos;
        currPos = 0;
        Mavryk.Tom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Mavryk.Jerry.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Ryk reporting voltage: ", Mavryk.getBatteryVoltage());
        telemetry.addData("Status:", "Ryk is ready to roll!");
        telemetry.update();

        return;
    }

    public void rykClaw() {
        ServoTurn = gamepad2.right_trigger == 1f;


        if (ServoTurn) {
            Claw_Position = Mvrk_Robot.Claw_Open_Pos;
        } else {
            Claw_Position = Mvrk_Robot.Claw_Close_Pos;
        }

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Position);

    }

    //Slider speed control

    public void mvrkUpSlide() {

        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (Mvrk_Robot.UpAdjust <= 1) {
                    Mvrk_Robot.UpAdjust = 1;
                } else {
                    Mvrk_Robot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
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
                if (Mvrk_Robot.UpAdjust >= 10) {
                    Mvrk_Robot.UpAdjust = 10;
                } else {
                    Mvrk_Robot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, -gamepad2.left_stick_y * (Mvrk_Robot.UpAdjust / 10));


        telemetry.addData("gamepad power", Mavryk.Tom.getPower() + ", " + Mavryk.Jerry.getPower());
        telemetry.addData("rykUpSlide: Current Slide Position: ", Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
        telemetry.update();

    }

    public void mvrkUpSlide_rue() {
        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (Mvrk_Robot.UpAdjust <= 1) {
                    Mvrk_Robot.UpAdjust = 1;
                } else {
                    Mvrk_Robot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
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
                if (Mvrk_Robot.UpAdjust >= 10) {
                    Mvrk_Robot.UpAdjust = 10;
                } else {
                    Mvrk_Robot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
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
            Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, newPos);
            Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
            if (newPos > currPos) {
                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);
                while (opModeIsActive() && Mavryk.areMotorsBusy(Mvrk_Robot.MvrkMotors.CAT_MOUSE)) {
                    idle();
                }
            }
            else if (newPos < currPos) {
                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);
                while (opModeIsActive() && Mavryk.areMotorsBusy(Mvrk_Robot.MvrkMotors.CAT_MOUSE)) {
                    idle();
                }
            }
            currPos = newPos;
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_rue: Current Slide Position: ", Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
        telemetry.update();
    }


    public void mvrkUpSlide_rtp() {
        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (Mvrk_Robot.UpAdjust <= 1) {
                    Mvrk_Robot.UpAdjust = 1;
                } else {
                    Mvrk_Robot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
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
                if (Mvrk_Robot.UpAdjust >= 10) {
                    Mvrk_Robot.UpAdjust = 10;
                } else {
                    Mvrk_Robot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
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
            Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, newPos);
            Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
            if (newPos > currPos) {
                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);
            }
            else if (newPos < currPos) {
                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);
            }
            currPos = newPos;
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_rue: Current Slide Position: ", Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
        telemetry.update();
    }


    public void mvrkSlideTester_rue(){
        
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Open_Pos);

        // Setting the target position to Cone5
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, TopCone);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone5 position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Close_Pos);

        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, LowJunction);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone5 position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Open_Pos);


        // Setting the target position to Cone4
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, TopMidCone);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone4 position....");
            telemetry.update();
            idle();
        }
        timer.reset();
//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, LowJunction);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone5 position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Open_Pos);


        // Setting the target position to Cone3
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, MiddleCone);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone3 position....");
            telemetry.update();
            idle();
        }
        timer.reset();
//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, LowJunction);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone5 position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Open_Pos);


        // Setting the target position to Cone2
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, BottomMidCone);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone2 position....");
            telemetry.update();
            idle();
        }
        timer.reset();
//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, LowJunction);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone5 position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Open_Pos);


        // Setting the target position to Cone1
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, BottomCone);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone1 position....");
            telemetry.update();
            idle();
        }
        timer.reset();
//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Close_Pos);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, LowJunction);
        Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.MvrkMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Cone5 position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        Mavryk.setPosition(Mvrk_Robot.MvrkServos.TWIN_TOWERS, Claw_Open_Pos);

        telemetry.addData("Tester_rue: Current Slide Position: ", Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
        telemetry.update();
    }

    public void mvrkSlideTester_pid() {

        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (Mvrk_Robot.UpAdjust <= 1) {
                    Mvrk_Robot.UpAdjust = 1;
                } else {
                    Mvrk_Robot.UpAdjust -= 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
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
                if (Mvrk_Robot.UpAdjust >= 10) {
                    Mvrk_Robot.UpAdjust = 10;
                } else {
                    Mvrk_Robot.UpAdjust += 1;
                }
                telemetry.addData("Current Slide Speed: ", "%f", Mvrk_Robot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        newPos = newPos + (int) (-gamepad2.left_stick_y * ticks_stepSize);
        if( newPos >= HighJunction)
            newPos = HighJunction;
        else if (newPos <= FloorPosition)
            newPos = FloorPosition;
        telemetry.addData("newPos", newPos);
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

        telemetry.addData("newPos", newPos);
        telemetry.update();

        if( newPos != currPos && newPos >= FloorPosition && newPos <= HighJunction ) {
            double command = Mvrk_Robot.control.output(newPos, Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
            if(newPos < currPos)
                Mvrk_Robot.SlidePower = Math.max(command/HighJunction, SlidePower_Down);
            else
                Mvrk_Robot.SlidePower = Math.min(command/HighJunction, SlidePower_Up);

            Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE,Mvrk_Robot.SlidePower);
            currPos = Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE);
            telemetry.addData("currPos updated to: ", currPos);
            telemetry.addData("command power:", Mvrk_Robot.SlidePower);
            telemetry.update();
        }

        telemetry.addData("rykUpSlide_pid: Current Slide Position: ", Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
        telemetry.update();
    }
}
