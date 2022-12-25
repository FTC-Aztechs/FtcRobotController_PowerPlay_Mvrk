package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class MvrkPose2d {
    public static double x;
    public static double y;
    public static double heading;
    Pose2d myPose2D;
    MvrkPose2d(double inX, double inY, double inHeading) { x = inX; y = inY; heading = inHeading;
        myPose2D = new Pose2d(x, y, Math.toRadians(heading)); }
    public Pose2d pose2d() { return myPose2D; }
}
