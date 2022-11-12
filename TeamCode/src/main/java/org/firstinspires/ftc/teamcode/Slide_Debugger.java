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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Floor;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="Slide Test" , group="Manual mode")
//@Disabled


public class Slide_Debugger extends LinearOpMode {
    // Declare OpMode members.
    Ryk_Robot Mavryk = new Ryk_Robot();

    private boolean changingSlideSpeed = false;
    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position

    // Dashboard exposed control variables
    public static double UpAdjust = 10;
    public static int HighJunction = 1000;
    public static int MidJunction = 800;
    public static int LowJunction = 450;
    public static int GroundJunction = 100;
    public static int FloorPosition = 10;
    public static double SlidePower_Up= 1;
    public static double SlidePower_Down = 0.5;
    public static int ticks_stepSize = 100;
    public static int Mode = 1;
    public static int BUTTON_TRIGGER_TIMER_MS = 500;


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

    FtcDashboard rykDashboard;
    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;

    RykPIDController control = new RykPIDController(0.05, 0, 0 );

    private int currPos = FloorPosition;

    //    public enum TomJerryState {
    //        LIFT_FLOOR,
    //        LIFT_HIGH_POS,
    //        LIFT_MEDIUM_POS,
    //        LIFT_LOW_POS,
    //    }

    // State Transitions:
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

        // Initialize the drive system vriables
        Mavryk.init(hardwareMap);

        initMavryk();
        Mavryk.setPosition(Ryk_Robot.RykServos.TWIN_TOWERS, Mavryk.Claw_Close_Pos);
        waitForStart();

        if( Mode == 1 || Mode == 2) {
            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_USING_ENCODER);
            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, STOP_AND_RESET_ENCODER);
        }
        else
            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_WITHOUT_ENCODER);

        telemetry.addData("Slide Debugger Running in mode: ", Mode);
        telemetry.update();

        while (opModeIsActive()) {
            if (Mode == 0) {
                rykUpSlide();
            } else if(Mode == 1){
                rykUpSlide_rue();
            } else if(Mode == 2) {
                rykSlideTester_rue();
            } else if(Mode == 3) {
                rykSlideTester_pid();
            }
            rykClaw();
        }
    }

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = rykDashboard;
        Claw_Position = Ryk_Robot.Claw_Close_Pos;
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

        telemetry.addData("rykUpSlide: Current Slide Position: ", Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
        telemetry.update();

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


    public void rykSlideTester_rue(){
        
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();

        // Setting the target position to HighJunction
        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, HighJunction);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() && /*Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE) &&*/ timer.milliseconds() < 5000 ) {
            telemetry.addLine("In High position....");
            telemetry.update();
            idle();
        }
        timer.reset();


        // Setting the target position to MidJunction
        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MidJunction);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Middle position....");
            telemetry.update();
            idle();
        }
        timer.reset();


        // Setting the target position to LowJunction
        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Low position....");
            telemetry.update();
            idle();
        }
        timer.reset();

        // Setting the target position to GroundJunction
        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Ground position....");
            telemetry.update();
            idle();
        }
        timer.reset();

        // Setting the target position to SlidesAtRest
        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, FloorPosition);
        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);

        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
            telemetry.addLine("In Rest position....");
            telemetry.update();
            idle();
        }
        timer.reset();

//        double SlideDir = 1;
//
//        if (Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) <= MotorPos){
//            SlideDir = 1;
//        } else{
//            SlideDir = -1;
//        }
//
//        if ((MotorPos - 10) < Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE)< (MotorPos+10)){
//            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);
//
//        } else {
//            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlideDir* SlidePower);
//        }

        telemetry.addData("Tester_rue: Current Slide Position: ", Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
        telemetry.update();

    }

    public void rykSlideTester_pid(){

        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();

        // Setting the target position to HighJunction
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower);
//        while(opModeIsActive() && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) <= HighJunction ) {
//            telemetry.addLine("Achieving High position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);
//        while(timer.time()<5000) {
//            telemetry.addLine("In High position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();


        int currPos = Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE);
        telemetry.addData("Current Position: ", currPos);
        telemetry.update();
        if( currPos >= HighJunction ) {
//            telemetry.addLine("In High position....");
//            telemetry.addData("killing power .. I think: ", 0);
//            telemetry.update();
            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);
        }
        else {
//            telemetry.addLine("Reaching High position....");
//            telemetry.update();
            double command = control.output(HighJunction, Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, command);
//            telemetry.addData("setting power: ", command);
//            telemetry.update();
        }

        //
//        int currPos = Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE);
//        while (currPos < HighJunction ) {
//            currPos = Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE);
//            double command = control.output(HighJunction, currPos);
//            telemetry.addData("setting power: ", command);
//            telemetry.update();
//            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, command);
//        }
//        while(timer.time()<5000) {
//            telemetry.addData("Current Position: ", currPos);
//            telemetry.addLine("In High position....");
//            telemetry.update();
//            idle();
//        }
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0 );
//        timer.reset();

//        while(timer.time()<5000) {
//            telemetry.addLine("In High position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();

//
//        // Setting the target position to MidJunction
//        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MidJunction);
//        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower);
//
//        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
//            telemetry.addLine("In Middle position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();
//
//
//        // Setting the target position to LowJunction
//        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, LowJunction);
//        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower);
//
//        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
//            telemetry.addLine("In Low position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();
//
//        // Setting the target position to GroundJunction
//        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, GroundJunction);
//        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower);
//
//        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
//            telemetry.addLine("In Ground position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();
//
//        // Setting the target position to SlidesAtRest
//        Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, SlidesAtRest);
//        Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
//        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower);
//
//        while(opModeIsActive() /*&& Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)*/ && timer.milliseconds() < 5000 ) {
//            telemetry.addLine("In Rest position....");
//            telemetry.update();
//            idle();
//        }
//        timer.reset();
//
//
//
////        double SlideDir = 1;
////
////        if (Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) <= MotorPos){
////            SlideDir = 1;
////        } else{
////            SlideDir = -1;
////        }
////
////        if ((MotorPos - 10) < Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE) && Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE)< (MotorPos+10)){
////            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, 0);
////
////        } else {
////            Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlideDir* SlidePower);
////        }

//        telemetry.addData("Current Slide Position: ", Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
//        telemetry.update();

    }

}









