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
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.BUTTON_TRIGGER_TIMER_MS;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.FloorPosition;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.imu;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.slideHeightSafetyBarrier;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turretIncrement;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turret_Range;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turret_fullRange;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.turret_restrictedRange;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideInPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideIncrement;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideOutPos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.xSlideSafetyBarrier;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name="MvrkManual", group="Manual mode")
//@Disabled
public class Mvrk_Manual extends LinearOpMode {

    // Declare OpMode members.
    Mvrk_Robot Mavryk = new Mvrk_Robot();
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
    double turret_Position;
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

    private static ElapsedTime timer_gp1_left_bumper = new ElapsedTime(MILLISECONDS);


    private boolean assumingHighPosition = false;
    private boolean assumingMidPosition = false;
    private boolean assumingLowPosition = false;
    private boolean assumingFloorPosition = false;
    private boolean assumingTopMidCone = false;
    private boolean assumingMiddleCone = false;
    private int currentSlidePos = FloorPosition;
    private boolean changing_drive_mode = false;
    private boolean fieldCentric = true;


    FtcDashboard mvrkDashboard;
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
            //MvrkUpSlide_rtp();
            MvrkClaw();
//            MvrkIntake();
            MvrkXSlide();
            MrvkTurret();

            if(gamepad1.left_bumper) {
                if (!changing_drive_mode) {
                    timer_gp1_left_bumper.reset();
                    changing_drive_mode = true;
                } else if (timer_gp1_left_bumper.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
                    fieldCentric = !fieldCentric;
                    changing_drive_mode = false;
                }
            }

            if(fieldCentric){
                MvrkManualDrive_FieldCentric();
                telemetry.addLine("Drive Mode: Field Centric");
                telemetry.update();
            }else{
                MvrkManualDrive();
                telemetry.addLine("Drive Mode: Forward Facing");
                telemetry.update();
            }
        }
    }

    public void initMavryk() {
        msStuckDetectStop = 2500;
        FtcDashboard Dash = mvrkDashboard;

        Claw_Position = Mavryk.Claw_Close_Pos;

        //Current_Intake_Position = IntakeInsidePos;

        //Mavryk.setPosition(Mvrk_Robot.MvrkServos.FUNKY_MONKEY, Current_Intake_Position);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);
        imu.initialize(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Status: Robot is ready to roll!");
        telemetry.update();

        return;
    }


//    public void MvrkIntake() {
//        boolean bIntake = gamepad1.right_trigger == 1f;
//
//        if (bIntake && Current_Intake_Position == IntakeInsidePos) {
//            boolean bHaveIRaisedTheClaw = false;
//            if (currentSlidePos < MiddleCone)
//            {
//                Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, MiddleCone);
//                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
//                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);
//                currentSlidePos = MiddleCone;
//                bHaveIRaisedTheClaw = true;
//            }
//            Current_Intake_Position = RightFunkyOutsidePos;
//            Mavryk.setPosition(Mvrk_Robot.MvrkServos.FUNKY_MONKEY, Current_Intake_Position);
//            Mavryk.setPosition(Mvrk_Robot.MvrkServos.RIGHT_FUNKY, RightFunkyOutsidePos);
//            Mavryk.setPosition(Mvrk_Robot.MvrkServos.LEFT_MONKEY, LeftMonkeyOutsidePos);
//            if (bHaveIRaisedTheClaw) {
//                Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, FloorPosition);
//                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);
//            }
//            telemetry.addLine("Intake out!");
//            telemetry.update();
//        }
//        if (!bIntake && Current_Intake_Position == RightFunkyOutsidePos) {
//            boolean bHaveIRaisedTheClaw = false;
//            if (currentSlidePos < MiddleCone) {
//
//                Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, MiddleCone);
//                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
//                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);
//                currentSlidePos = MiddleCone;
//                bHaveIRaisedTheClaw = true;
//            }
//            Current_Intake_Position = IntakeInsidePos;
//            Mavryk.setPosition(Mvrk_Robot.MvrkServos.FUNKY_MONKEY, Current_Intake_Position);
//            if (bHaveIRaisedTheClaw) {
//                Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, FloorPosition);
//                Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);
//            }
//            telemetry.addLine("Intake in!");
//            telemetry.update();
//        }
//
//        if(bIntake) {
//            Sweeper_Power = 0.5;
//            Mavryk.setCRPower(Mvrk_Robot.MvrkServos.CAR_WASH, Sweeper_Power);
//            telemetry.addLine("Intake on!");
//            telemetry.update();
//        }
//        else {
//            //TODO: When Retracting from Intake - need to make sure claw is raised to provide clearance.
//            Sweeper_Power = 0;
//            Mavryk.setCRPower(Mvrk_Robot.MvrkServos.CAR_WASH, Sweeper_Power);
//            telemetry.addLine("Intake off!");
//            telemetry.update();
//        }
//
//    }

    public void MvrkManualDrive() {
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

        if (gamepad1.right_bumper) {
            speedAdjust = bumperSpeedAdjust;
        } else {
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

    public void MvrkManualDrive_FieldCentric()
    {

        Localizer myLocaliizer = Mavryk.mecanumDrive.getLocalizer();
        myLocaliizer.getPoseVelocity();
        Pose2d poseEstimate = Mavryk.mecanumDrive.getPoseEstimate();
        Vector2d input = new Vector2d( -gamepad1.left_stick_y,
                -gamepad1.left_stick_x).rotated(-imu.getAngularOrientation().firstAngle);

        Mavryk.mecanumDrive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x));
        Mavryk.mecanumDrive.update();

//        telemetry.addData("gamepad 1 right stick y", -gamepad1.right_stick_y);
//        telemetry.update();

        return;
    }

    public void MvrkXSlide() {

        if (turret_restrictedRange[0] < Mavryk.Teacup.getPosition() && Mavryk.Teacup.getPosition() < turret_restrictedRange[1]) {
            Mavryk.xSlideMinExtension = xSlideInPos;
            Mavryk.xSlideMaxExtension = xSlideOutPos;
            telemetry.addData("Horizontal Slides", "Full range is ready to go!");
        } else {
            Mavryk.xSlideMinExtension = xSlideSafetyBarrier;
            Mavryk.xSlideMaxExtension = xSlideOutPos;
            telemetry.addData("Horizontal Slides", "Full retraction is locked. Turn the turret forward to unlock full range!");
        }

        if (gamepad1.dpad_up) {
            if (xSlide_Position >= Mavryk.xSlideMaxExtension) {
                xSlide_Position = Mavryk.xSlideMaxExtension;
            } else {
                xSlide_Position += xSlideIncrement;
            }

        }
        if (gamepad1.dpad_down) {
            if (xSlide_Position <= Mavryk.xSlideMinExtension) {
                xSlide_Position = Mavryk.xSlideMaxExtension;
            } else {
                xSlide_Position -= xSlideIncrement;
            }
            Mavryk.setPosition(Mvrk_Robot.MvrkServos.FLAMETHROWER, xSlide_Position);
        }
    }

    public void MvrkClaw() {
        ServoTurn = gamepad2.right_trigger == 1f;


        if (ServoTurn) {
            Claw_Position = Mavryk.Claw_Open_Pos;
        } else {
            Claw_Position = Mavryk.Claw_Close_Pos;
        }

        Mavryk.setPosition(Mvrk_Robot.MvrkServos.CARTOON, Claw_Position);

    }

//        public void MvrkUpSlide_rtp(){
//            //Gamepad2 left -> Decrease Speed
//            if (gamepad2.dpad_left) {
//                if (!changingSlideSpeed) {
//                    timer_gp2_dpad_left.reset();
//                    changingSlideSpeed = true;
//                } else if (timer_gp2_dpad_left.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    if (Mvrk_Robot.UpAdjust <= 1) {
//                        Mvrk_Robot.UpAdjust = 1;
//                    } else {
//                        Mvrk_Robot.UpAdjust -= 1;
//                    }
//                    telemetry.addLine("Current Slide Speed: " + Mvrk_Robot.UpAdjust);
//                    telemetry.update();
//                    changingSlideSpeed = false;
//                }
//            }
//
//            //Gamepad 2right -> Increase Speed
//            if (gamepad2.dpad_right) {
//                if (!changingSlideSpeed) {
//                    timer_gp2_dpad_right.reset();
//                    changingSlideSpeed = true;
//                } else if (timer_gp2_dpad_right.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    if (Mvrk_Robot.UpAdjust >= 10) {
//                        Mvrk_Robot.UpAdjust = 10;
//                    } else {
//                        Mvrk_Robot.UpAdjust += 1;
//                    }
//                    telemetry.addLine("Current Slide Speed: " + Mvrk_Robot.UpAdjust);
//                    telemetry.update();
//                    changingSlideSpeed = false;
//                }
//            }
//
//            int newPos = currentSlidePos + (int) (-gamepad2.left_stick_y * ticks_stepSize);
//            if (newPos >= HighJunction)
//                newPos = HighJunction;
//            else if (newPos <= FloorPosition)
//                newPos = FloorPosition;
//            telemetry.addLine("newPos calc from gamePad2.left_stick_y: " + newPos);
//            telemetry.update();
//
//            if (gamepad2.y) {
//                if (!assumingHighPosition) {
//                    timer_gp2_buttonY.reset();
//                    assumingHighPosition = true;
//                } else if (timer_gp2_buttonY.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    telemetry.addLine("GP2_Y triggered. Set Tom&Jerry to High position");
//                    telemetry.update();
//
//                    newPos = HighJunction;
//                    assumingHighPosition = false;
//                }
//            }
//
//            if (gamepad2.x) {
//                if (!assumingMidPosition) {
//                    timer_gp2_buttonX.reset();
//                    assumingMidPosition = true;
//                } else if (timer_gp2_buttonX.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    telemetry.addLine("GP2_X triggered. Set Tom&Jerry to Mid position");
//                    telemetry.update();
//                    newPos = MidJunction;
//                    assumingMidPosition = false;
//                }
//            }
//
//            if (gamepad2.a) {
//                if (!assumingLowPosition) {
//                    timer_gp2_buttonY.reset();
//                    assumingLowPosition = true;
//                } else if (timer_gp2_buttonA.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    telemetry.addLine("GP2_A triggered. Set Tom&Jerry to Low position");
//                    telemetry.update();
//
//                    newPos = LowJunction;
//                    assumingLowPosition = false;
//                }
//            }
//
//            if (gamepad2.b) {
//                if (!assumingFloorPosition) {
//                    timer_gp2_buttonB.reset();
//                    assumingFloorPosition = true;
//                } else if (timer_gp2_buttonB.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    telemetry.addLine("GP2_B triggered. Set Tom&Jerry to Floor position");
//                    telemetry.update();
//
//                    newPos = FloorPosition;
//                    assumingFloorPosition = false;
//                }
//            }
//
//            if (gamepad2.dpad_up) {
//                if (!assumingTopMidCone) {
//                    timer_gp2_dpad_up.reset();
//                    assumingTopMidCone = true;
//                } else if (timer_gp2_dpad_up.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    telemetry.addLine("GP2_dpad_up triggered. Set Tom&Jerry to TopMidCone");
//                    telemetry.update();
//
//                    newPos = TopMidCone;
//                    assumingTopMidCone = false;
//                }
//            }
//
//            if (gamepad2.dpad_down) {
//                if (!assumingMiddleCone) {
//                    timer_gp2_dpad_down.reset();
//                    assumingMiddleCone = true;
//                } else if (timer_gp2_dpad_down.time(TimeUnit.MILLISECONDS) > BUTTON_TRIGGER_TIMER_MS) {
//                    telemetry.addLine("GP2_dpad_up triggered. Set Tom&Jerry to MiddleCone");
//                    telemetry.update();
//
//                    newPos = MiddleCone;
//                    assumingMiddleCone = false;
//                }
//            }
//
//            telemetry.addLine("newPos from Any button triggers: " + newPos);
//            telemetry.update();
//
//            if (newPos != currentSlidePos && newPos >= FloorPosition && newPos <= HighJunction) {
//                Mavryk.setTargetPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE, newPos);
//                Mavryk.setRunMode(Mvrk_Robot.MvrkMotors.CAT_MOUSE, RUN_TO_POSITION);
//                if (newPos > currentSlidePos) {
//                    Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Up);
//                } else if (newPos < currentSlidePos) {
//                    Mavryk.setPower(Mvrk_Robot.MvrkMotors.CAT_MOUSE, SlidePower_Down);
//                }
//                currentSlidePos = newPos;
//                telemetry.addLine("currPos updated to: " + currentSlidePos);
//                telemetry.update();
//            }
//
//            telemetry.addLine("mvrkUpSlide_rue: Current Slide Position: " + Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE));
//            telemetry.update();
//        }
//
    public void MrvkTurret(){
        if (Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE) >= slideHeightSafetyBarrier || Mavryk.FlameThrower.getPosition() >= xSlideSafetyBarrier) {
            turret_Range[0] = turret_fullRange[0];
            turret_Range[1] = turret_fullRange[1];
            telemetry.addData("Turret", "Full rotation is ready to go!");
        } else {
            turret_Range[0] = turret_restrictedRange[0];
            turret_Range[1] = turret_restrictedRange[1];
            if(Mavryk.getCurrentPosition(Mvrk_Robot.MvrkMotors.CAT_MOUSE) >= slideHeightSafetyBarrier){
                telemetry.addData("Turret", "Full rotation is locked. Raise the slides to unlock full range!");
            }{
                telemetry.addData("Turret", "Full rotation is locked. Extend the horizontal slides to unlock full range!");
            }
            telemetry.update();
        }

        if (gamepad2.dpad_left) {
            if (turret_Position >= turret_Range[1]) {
                turret_Position = turret_Range[1];
            } else {
                turret_Position += turretIncrement;
            }

        }
        if (gamepad2.dpad_right) {
            if (turret_Position <= turret_Range[0]) {
                turret_Position = turret_Range[0];
            } else {
                turret_Position -= turretIncrement;
            }
            Mavryk.setPosition(Mvrk_Robot.MvrkServos.TEACUP, turret_Position);
        }
    }
}

