package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// TODO: General test program, update with whatever tasks as necessary

@Autonomous(name="PositionTestBlue", group="linearOpMode")
public class PositionTestBlue extends AutonomousPrime2021 {
    @Override
    public void runOpMode(){
        mapObjects();
        waitForStart();

        duckSpin(1,5);
        pause(2);



        /*while(!isStopRequested()) {
            updateAllDist();
            telemetry.addData("RightDist: ", RightDist);
            telemetry.addData("LeftDist: ", LeftDist);

            telemetry.addData("FrontRightDist: ", FrontRightDist);
            telemetry.addData("FrontLeftDist: ", FrontLeftDist);

            telemetry.addData("BackRightDist: ", BackRightDist);
            telemetry.addData("BackLeftDist: ", BackLeftDist);

            telemetry.addData("GroundFrontDist: ", GroundFrontDist);
            telemetry.addData("GroundBackDist: ", GroundBackDist);

            telemetry.update();



        }*/

        /*rightEncoder(90, 0.5);
        pause(1);
        leftEncoder(90, 0.5);*/

        /*chute(0);
        pause(4);


        chute(0.1);
        pause(4);

        chute(0.2);
        pause(4);

        chute(0.3);
        pause(4);

        chute(0.4);
        pause(4);

        chute(0.5);
        pause(4);

        chute(0.6);
        pause(4);

        chute(0.7);
        pause(4);

        chute(0.8);
        pause(4);

        chute(0.9);
        pause(4);

        chute(1);
        pause(4);*/



        /*while(!isStopRequested()) {
            //program to test getAngle, and then test if it properly sets the angle with setAngle()
            getAngle();
            pause(2);
            rightEncoder(20, 0.5);
            getAngle();
            pause(2);
            rightEncoder(20, 0.5);
            getAngle();
            pause(2);
            setAngle(100);
            pause(2);
            rightEncoder(20, 0.5);
            getAngle();
            pause(2);
        }*/

        //rightEncoder(135, 0.5);

        //dArm(-5,0.25);
        //pause(5);

        //chute(10,0.5);

        //hi logan -Jacob
        //What da dog doin? -Nate
        //Ur mom -Abraham Lincoln
    }
}