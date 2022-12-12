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
 * SERVICES; LOSS OF USE, Line, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import static org.firstinspires.ftc.teamcode.Ryk_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.FloorPosition;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.IntakeInsidePos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.LeftMonkeyOutsidePos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.LowJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.MidJunction;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.MiddleCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.RightFunkyOutsidePos;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.TopMidCone;
import static org.firstinspires.ftc.teamcode.Ryk_Robot.ticks_stepSize;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
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


    public static int Mode = 1;
//    public static int BUTTON_TRIGGER_TIMER_MS = 500;

    boolean ServoTurn = false;
    double Claw_Position; // Start at halfway position
    double xSlide_Position;
    double Current_Intake_Position;
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
    private static ElapsedTime timer_gp2_dpad_up = new ElapsedTime(MILLISECONDS);
    private static ElapsedTime timer_gp2_dpad_down = new ElapsedTime(MILLISECONDS);
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
    private boolean assumingTopMidCone = false;
    private boolean assumingMiddleCone = false;
    private int currentSlidePos = FloorPosition;


    FtcDashboard rykDashboard;
    private double bumperSpeedAdjust = 8;
    private double dPadSpeedAdjust = 5;

    @Override
    public void runOpMode() {
    PhotonCore.enable();
        // Initialize the drive system vriables
        Mavryk.init(hardwareMap);

        initMavryk();
        waitForStart();


        while (opModeIsActive()) {
            RykManualDrive();
            RykUpSlide_rtp();
            RykClaw();
            RykIntake();
            RykXSlide();
        }
    }

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = rykDashboard;

        Claw_Position = Mavryk.Claw_Close_Pos;

        Current_Intake_Position = IntakeInsidePos;

        Mavryk.setPosition(Ryk_Robot.RykServos.FUNKY_MONKEY, Current_Intake_Position);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Status: Robot is ready to roll!");
        telemetry.update();

        return;
    }


    public void RykIntake() {
        boolean bIntake = gamepad1.right_trigger == 1f;

        if (bIntake && Current_Intake_Position == IntakeInsidePos) {
            boolean bHaveIRaisedTheClaw = false;
            if (currentSlidePos < MiddleCone)
            {
                Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
                Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                currentSlidePos = MiddleCone;
                bHaveIRaisedTheClaw = true;
            }
            Current_Intake_Position = RightFunkyOutsidePos;
            Mavryk.setPosition(Ryk_Robot.RykServos.FUNKY_MONKEY, Current_Intake_Position);
            Mavryk.setPosition(Ryk_Robot.RykServos.RIGHT_FUNKY, RightFunkyOutsidePos);
            Mavryk.setPosition(Ryk_Robot.RykServos.LEFT_MONKEY, LeftMonkeyOutsidePos);
            if (bHaveIRaisedTheClaw) {
                Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, FloorPosition);
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
            }
            telemetry.addLine("Intake out!");
            telemetry.update();
        }
        if (!bIntake && Current_Intake_Position == RightFunkyOutsidePos) {
            boolean bHaveIRaisedTheClaw = false;
            if (currentSlidePos < MiddleCone) {

                Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, MiddleCone);
                Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                currentSlidePos = MiddleCone;
                bHaveIRaisedTheClaw = true;
            }
            Current_Intake_Position = IntakeInsidePos;
            Mavryk.setPosition(Ryk_Robot.RykServos.FUNKY_MONKEY, Current_Intake_Position);
            if (bHaveIRaisedTheClaw) {
                Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, FloorPosition);
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
            }
            telemetry.addLine("Intake in!");
            telemetry.update();
        }

        if(bIntake) {
            Sweeper_Power = 0.5;
            Mavryk.setCRPower(Ryk_Robot.RykServos.CAR_WASH, Sweeper_Power);
            telemetry.addLine("Intake on!");
            telemetry.update();
        }
        else {
            //TODO: When Retracting from Intake - need to make sure claw is raised to provide clearance.
            Sweeper_Power = 0;
            Mavryk.setCRPower(Ryk_Robot.RykServos.CAR_WASH, Sweeper_Power);
            telemetry.addLine("Intake off!");
            telemetry.update();
        }

    }

    public void RykManualDrive() {
        if (gamepad1.dpad_left) {
            if (!changingWheelSpeed) {
                timer_gp1_dpad_left.reset();
                changingWheelSpeed = true;
            } else if (timer_gp1_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (dPadSpeedAdjust <= 1) {
                    dPadSpeedAdjust = 1;
                } else {
                    dPadSpeedAdjust -= 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
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
                if (dPadSpeedAdjust >= 10) {
                    dPadSpeedAdjust = 10;
                } else {
                    dPadSpeedAdjust += 1;
                }
                telemetry.addLine("Current speed: " + dPadSpeedAdjust);
                telemetry.update();
                changingWheelSpeed = false;
            }
        }

        if(gamepad1.right_bumper) {
            speedAdjust = bumperSpeedAdjust;
        }
        else {
            speedAdjust = dPadSpeedAdjust;
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
    public void RykXSlide() {
        if(gamepad1.dpad_up) {
            xSlide_Position = Mavryk.xSlideOutPos;

        }
        else if(gamepad1.dpad_down)
        {
            xSlide_Position = Mavryk.xSlideInPos;
        }
        Mavryk.setPosition(Ryk_Robot.RykServos.FLAMETHROWER, xSlide_Position);
    }
    public void RykClaw() {
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
                telemetry.addLine("Current slide speed: " + UpAdjust);
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
                telemetry.addLine("Current slide speed: " + UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        int newPos = currentSlidePos + (int) ( -gamepad2.left_stick_y * ticks_stepSize);
        if( newPos >= HighJunction)
            newPos = HighJunction;
        else if (newPos <= FloorPosition)
            newPos = FloorPosition;
        telemetry.addLine("newPos calc from gamePad2.left_stick_y: " + newPos);
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


        telemetry.addLine("newPos from Any button triggers: " + newPos);
        telemetry.update();

        if( newPos != currentSlidePos && newPos >=FloorPosition && newPos <= HighJunction ) {
            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, newPos);
            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
            if (newPos > currentSlidePos) {
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
                while (opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                    idle();
                }
            }
            else if (newPos < currentSlidePos) {
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
                while (opModeIsActive() && Mavryk.areMotorsBusy(Ryk_Robot.RykMotors.CAT_MOUSE)) {
                    idle();
                }
            }
            currentSlidePos = newPos;
            telemetry.addLine("currPos updated to: "+ currentSlidePos);
            telemetry.update();
        }

        telemetry.addLine("rykUpSlide_rue: Current Slide Position: " + Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
        telemetry.update();
    }

    public void RykUpSlide_rtp() {
        //Gamepad2 left -> Decrease Speed
        if (gamepad2.dpad_left) {
            if (!changingSlideSpeed) {
                timer_gp2_dpad_left.reset();
                changingSlideSpeed = true;
            } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                if (Ryk_Robot.UpAdjust <= 1) {
                    Ryk_Robot.UpAdjust = 1;
                } else {
                    Ryk_Robot.UpAdjust -= 1;
                }
                telemetry.addLine("Current Slide Speed: "+ Ryk_Robot.UpAdjust);
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
                if (Ryk_Robot.UpAdjust >= 10) {
                    Ryk_Robot.UpAdjust = 10;
                } else {
                    Ryk_Robot.UpAdjust += 1;
                }
                telemetry.addLine("Current Slide Speed: "+ Ryk_Robot.UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        int newPos = currentSlidePos + (int) ( -gamepad2.left_stick_y * ticks_stepSize);
        if( newPos >= HighJunction)
            newPos = HighJunction;
        else if (newPos <= FloorPosition)
            newPos = FloorPosition;
        telemetry.addLine("newPos calc from gamePad2.left_stick_y: "+ newPos);
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

        if(gamepad2.dpad_up) {
            if (!assumingTopMidCone) {
                timer_gp2_dpad_up.reset();
                assumingTopMidCone = true;
            } else if (timer_gp2_dpad_up.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_dpad_up triggered. Set Tom&Jerry to TopMidCone");
                telemetry.update();

                newPos = TopMidCone;
                assumingTopMidCone = false;
            }
        }

        if(gamepad2.dpad_down) {
            if (!assumingMiddleCone) {
                timer_gp2_dpad_down.reset();
                assumingMiddleCone = true;
            } else if (timer_gp2_dpad_down.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                telemetry.addLine("GP2_dpad_up triggered. Set Tom&Jerry to MiddleCone");
                telemetry.update();

                newPos = MiddleCone;
                assumingMiddleCone = false;
            }
        }

        telemetry.addLine("newPos from Any button triggers: " + newPos);
        telemetry.update();

        if( newPos != currentSlidePos && newPos >=FloorPosition && newPos <= HighJunction ) {
            Mavryk.setTargetPosition(Ryk_Robot.RykMotors.CAT_MOUSE, newPos);
            Mavryk.setRunMode(Ryk_Robot.RykMotors.CAT_MOUSE, RUN_TO_POSITION);
            if (newPos > currentSlidePos) {
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Up);
            }
            else if (newPos < currentSlidePos) {
                Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, SlidePower_Down);
            }
            currentSlidePos = newPos;
            telemetry.addLine("currPos updated to: "+ currentSlidePos);
            telemetry.update();
        }

        telemetry.addLine("rykUpSlide_rue: Current Slide Position: "+ Mavryk.getCurrentPosition(Ryk_Robot.RykMotors.CAT_MOUSE));
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
                telemetry.addLine("Current Slide Speed: "+ UpAdjust);
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
                telemetry.addLine("Current Slide Speed: "+ UpAdjust);
                telemetry.update();
                changingSlideSpeed = false;
            }
        }

        Mavryk.setPower(Ryk_Robot.RykMotors.CAT_MOUSE, -gamepad2.left_stick_y * (UpAdjust / 10));
    }
}









