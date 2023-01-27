package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Close_Pos;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.Claw_Open_Pos;

import android.graphics.Color;
import android.widget.Switch;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Mvrk_ClawController {
    private Servo claw;
    private NormalizedColorSensor color = null;
    private DistanceSensor distance = null;
    private double currPos;
    public static double ConeDetectDistanceMM = 25;
    public static float ColorSensorGainTuner = 2;
    public static float ColorSensorRedThreshold = 200;
    public static float ColorSensorBlueThreshold = 200;

    enum clawState
    {
        Open,
        Close,
        Auto
    }
    clawState currState;
    clawState targetState;


    public Mvrk_ClawController(HardwareMap hardwareMap) {
        claw = hardwareMap.get(Servo.class, "Teacup");
//        color = hardwareMap.get(NormalizedColorSensor.class, "Colorific");
//        distance = hardwareMap.get(DistanceSensor.class, "Distanciate");
        currState = clawState.Open;
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
                    if (updateColor()) {
                        currPos = Claw_Close_Pos;
                    }
                } else {
                    currPos = Claw_Open_Pos;
                }
                currState = clawState.Auto;
                break;
        }

        return;
    }

    private boolean updateDistance() {
        return distance.getDistance(DistanceUnit.MM) < ConeDetectDistanceMM;
    }

    private boolean updateColor() {

        boolean bReturn = false;

        color.setGain(ColorSensorGainTuner);

        if(color instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) color;
            light.enableLight(true);
        }

        NormalizedRGBA colors = color.getNormalizedColors();
        if( (colors.red > ColorSensorRedThreshold && colors.blue < ColorSensorBlueThreshold) ||
                (colors.blue > ColorSensorBlueThreshold && colors.red < ColorSensorRedThreshold)) {
            bReturn = true;
        }

        if(color instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) color;
            light.enableLight(false);
        }
        return bReturn;
    }
}


