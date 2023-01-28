package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;

import android.graphics.Color;
import android.widget.Switch;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

@Config
public class Mvrk_ClawController {
    private Servo claw;
    private NormalizedColorSensor color = null;
    private DistanceSensor distance = null;
    private double currPos;
    public static double ConeDetectDistanceMM = 45;
    public static double ColorSensorGainTuner = 1;
    public static double ColorSensorRedThreshold = .7f;
    public static double ColorSensorBlueThreshold = .7f;
    float[] hsvValues = new float[3];
    final float values[] = hsvValues;
    enum clawState
    {
        Open,
        Close,
        Auto
    }
    clawState currState;
    clawState targetState;
    Telemetry telemetry;

    public Mvrk_ClawController(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "Looney_Toons");
        color = hardwareMap.get(NormalizedColorSensor.class, "Retina");
        distance = hardwareMap.get(DistanceSensor.class, "Aura");
        currState = clawState.Open;
    }

    public void setTelemetry(Telemetry tele)
    {
        telemetry = tele;
    }

    public void setTargetState(clawState state) {
        targetState = state;
    }

    public void update()
    {
        //  Open -> Open: No-op
        //  Open -> Close: Close, update curr
        //  Open -> Auto: Set in Auto, update curr
        //  Close -> Open: Open, update curr
        //  Close->Close: No-op
        //  Close->Auto: set in Auto, update curr
        //  Auto->Open: Open, update curr
        //  Auto->Close: Close, update curr
        //  Auto->Auto : Auto, update curr
        if(currState != clawState.Auto && currState == targetState)
            return;

        switch(targetState) {
            case Open:
                claw.setPosition(Claw_Open_Pos);

                currState = clawState.Open;
                break;
            case Close:
                claw.setPosition(Claw_Close_Pos);
                currState = clawState.Close;
                break;
            case Auto:
                // If something is in my grasp
                // And it is red or blue
                // Close the claw
                if (updateDistance()) {

                    if(telemetry != null) {
                        telemetry.addLine("Distance triggered");
                        telemetry.update();
                    }

                    if (updateColor()) {
                        if(telemetry != null) {
                            telemetry.addLine("Color triggered");
                            telemetry.update();
                        }
                        claw.setPosition(Claw_Close_Pos);
                   }

                } else {
                    if(telemetry != null) {
                        telemetry.addLine("Nothing detected");
                        telemetry.update();
                    }
                    claw.setPosition(Claw_Open_Pos);
                }
                currState = clawState.Auto;
                break;
        }

        return;
    }

    private boolean updateDistance() {

        double dist = distance.getDistance(DistanceUnit.MM);
        if(telemetry != null) {
            telemetry.addData("Distance: %f", dist);
            telemetry.update();
        }
        return (dist < ConeDetectDistanceMM);
    }

    private boolean updateColor() {

        boolean bReturn = false;

        color.setGain((float)ColorSensorGainTuner);

        NormalizedRGBA colors = color.getNormalizedColors();

        if(telemetry != null) {
            telemetry.addData("Normalized", colors);
            telemetry.addData("Alpha", "%.3f", colors.alpha);
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.update();
        }

        Color.colorToHSV(colors.toColor(), hsvValues);

        if(color instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) color;
            light.enableLight(true);
            if(telemetry != null) {
                telemetry.addLine("Cone not detected");
                telemetry.addData("Normalized", color.getNormalizedColors());
                telemetry.addData("Alpha", "%.3f", colors.alpha);
                telemetry.addData("Red", "%.3f", colors.red);
                telemetry.addData("Green", "%.3f", colors.green);
                telemetry.addData("Blue", "%.3f", colors.blue);
                telemetry.update();
            }

        }

        if( (colors.red > ColorSensorRedThreshold && colors.blue < ColorSensorBlueThreshold) ||
                (colors.blue > ColorSensorBlueThreshold && colors.red < ColorSensorRedThreshold)) {
            bReturn = true;
            if(telemetry != null) {
                telemetry.addLine("Cone detected");
                telemetry.addData("Normalized", color.getNormalizedColors());
                telemetry.addData("Alpha", "%.3f", colors.alpha);
                telemetry.addData("Red", "%.3f", colors.red);
                telemetry.addData("Green", "%.3f", colors.green);
                telemetry.addData("Blue", "%.3f", colors.blue);
                telemetry.update();
            }
        }

        if(color instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) color;
            light.enableLight(false);
        }
        return bReturn;
    }
}


