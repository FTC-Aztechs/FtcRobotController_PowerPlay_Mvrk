package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.HighJunction;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.LowerLimit;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Down;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.SlidePower_Up;
import static org.firstinspires.ftc.teamcode.Mvrk_Robot.UpperLimit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Mvrk_LiftController {
    private DcMotor tom;
    private DcMotor jerry;
    double tgtPos;
    double currPos;
    public static MvrkPIDController liftController = new MvrkPIDController(11, 0, 0.25, 3600);

    public Mvrk_LiftController(HardwareMap hardwareMap) {
        jerry = hardwareMap.get(DcMotor.class, "Jerry");
        tom = hardwareMap.get(DcMotor.class, "Tom");
        tgtPos = 0;
        currPos = 0;
    }

    public void setTargetPosition(double target) {
        if(target >= LowerLimit && target <= UpperLimit)
            tgtPos = target;
    }

    public void update() {
        if(currPos != tgtPos ) {
            double command = liftController.output(tgtPos, tom.getCurrentPosition());
            double slidePower = (tgtPos < currPos) ?
                    Math.max(command / HighJunction, SlidePower_Down) :
                    Math.min(command / HighJunction, SlidePower_Up);
            jerry.setPower(slidePower);
            tom.setPower(slidePower);
            currPos = tom.getCurrentPosition();
        }
    }
}

