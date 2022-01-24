package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

public class Sensors {
    /*
    Re-Mapping Everything Because Threads Hate Us :)
     */
    public DistanceSensor GroundFront = AutonomousPrime2021.GroundFront;
    public double GroundFrontDist = 0;
    public DistanceSensor GroundBack = AutonomousPrime2021.GroundBack;
    public double GroundBackDist = 0;

    public DistanceSensor Left = AutonomousPrime2021.Left;
    public double LeftDist = 0;
    public DistanceSensor Right = AutonomousPrime2021.Right;
    public double RightDist = 0;

    public DistanceSensor FrontLeft = AutonomousPrime2021.FrontLeft;
    public double FrontLeftDist = 0;
    public DistanceSensor FrontRight = AutonomousPrime2021.FrontRight;
    public double FrontRightDist = 0;

    public DistanceSensor BackLeft = AutonomousPrime2021.BackLeft;
    public double BackLeftDist = 0;
    public DistanceSensor BackRight = AutonomousPrime2021.BackRight;
    public double BackRightDist = 0;

    //However I do the IMU here


}
