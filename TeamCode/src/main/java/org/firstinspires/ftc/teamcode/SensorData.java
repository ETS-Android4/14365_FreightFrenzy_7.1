package org.firstinspires.ftc.teamcode;

public class SensorData {
    public double getGroundFrontDist() {
        return GroundFrontDist;
    }

    public void setGroundFrontDist(double groundFrontDist) {
        GroundFrontDist = groundFrontDist;
    }

    public double getGroundBackDist() {
        return GroundBackDist;
    }

    public void setGroundBackDist(double groundBackDist) {
        GroundBackDist = groundBackDist;
    }

    public double getLeftDist() {
        return LeftDist;
    }

    public void setLeftDist(double leftDist) {
        LeftDist = leftDist;
    }

    public double getRightDist() {
        return RightDist;
    }

    public void setRightDist(double rightDist) {
        RightDist = rightDist;
    }

    public double getFrontLeftDist() {
        return FrontLeftDist;
    }

    public void setFrontLeftDist(double frontLeftDist) {
        FrontLeftDist = frontLeftDist;
    }

    public double getFrontRightDist() {
        return FrontRightDist;
    }

    public void setFrontRightDist(double frontRightDist) {
        FrontRightDist = frontRightDist;
    }

    public double getBackLeftDist() {
        return BackLeftDist;
    }

    public void setBackLeftDist(double backLeftDist) {
        BackLeftDist = backLeftDist;
    }

    public double getBackRightDist() {
        return BackRightDist;
    }

    public void setBackRightDist(double backRightDist) {
        BackRightDist = backRightDist;
    }

    public double getIMUAngle() {
        return IMUAngle;
    }

    public void setIMUAngle(double IMUAngle) {
        this.IMUAngle = IMUAngle;
    }

    private double GroundFrontDist;
    private double GroundBackDist;
    private double LeftDist;
    private double RightDist;
    private double FrontLeftDist;
    private double FrontRightDist;
    private double BackLeftDist;
    private double BackRightDist;

    private double IMUAngle;


}
