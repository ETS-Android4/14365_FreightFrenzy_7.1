package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class SensorThread extends AutonomousPrime2021 implements Runnable {

    @Override
    public void run() {
        /*
        Re-Mapping Everything Because Threads Hate Us :)
        */
        DistanceSensor GroundFront = AutonomousPrime2021.GroundFront;
        DistanceSensor GroundBack = AutonomousPrime2021.GroundBack;

        DistanceSensor Left = AutonomousPrime2021.Left;
        DistanceSensor Right = AutonomousPrime2021.Right;

        DistanceSensor FrontLeft = AutonomousPrime2021.FrontLeft;
        DistanceSensor FrontRight = AutonomousPrime2021.FrontRight;

        DistanceSensor BackLeft = AutonomousPrime2021.BackLeft;
        DistanceSensor BackRight = AutonomousPrime2021.BackRight;

        BNO055IMU imu = AutonomousPrime2021.imu;

        SensorData ThreadUpload = new SensorData();

        while (!Thread.currentThread().isInterrupted()) {
            ThreadUpload.setBackLeftDist(BackLeft.getDistance(DistanceUnit.CM));
            ThreadUpload.setBackRightDist(BackRight.getDistance(DistanceUnit.CM));

            ThreadUpload.setFrontLeftDist(FrontLeft.getDistance(DistanceUnit.CM));
            ThreadUpload.setFrontRightDist(FrontRight.getDistance(DistanceUnit.CM));

            ThreadUpload.setLeftDist(Left.getDistance(DistanceUnit.CM));
            ThreadUpload.setRightDist(Right.getDistance(DistanceUnit.CM));

            ThreadUpload.setGroundFrontDist(GroundFront.getDistance(DistanceUnit.CM));
            ThreadUpload.setGroundBackDist(GroundBack.getDistance(DistanceUnit.CM));

            ThreadUpload.setIMUAngle(getAngle());

            AutonomousPrime2021.SensorData=ThreadUpload;



        }

    }

    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < 0) {
            deltaAngle += 360;
        } else if(deltaAngle > 360) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;
        if(globalAngle > 360) {
            globalAngle -= 360;
        } else if(globalAngle < 0) {
            globalAngle += 360;
        }

        lastAngles = angles;
        //telemetry.addData("Global Angle:", globalAngle);
        //telemetry.update();
        return globalAngle;
    }
    public void setAngle(double angle) {
        globalAngle = angle;
        //telemetry.addData("New Global Angle:", globalAngle);
        //telemetry.update();
    }


}
