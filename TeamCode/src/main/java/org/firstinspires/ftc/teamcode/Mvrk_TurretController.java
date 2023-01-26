package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Mvrk_TurretController {
    private Servo turret;
    double tgtPos;
    double currPos;
    public static MvrkPIDController turretController = new MvrkPIDController(0.5, 0, 0 , 0);

    public Mvrk_TurretController(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "Teacup");
        tgtPos = 0;
        currPos = 0;
    }

    public void setTargetPosition(double target) {
        tgtPos = target;
    }

    public void update() {
        if(currPos != tgtPos ) {
            double posDelta = turretController.output(tgtPos, currPos);
            turret.setPosition(currPos+posDelta);
            currPos = turret.getPosition();
        }
    }
}
