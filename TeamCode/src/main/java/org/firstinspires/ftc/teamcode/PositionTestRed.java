package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

// TODO: General test program, update with whatever tasks as necessary

@Autonomous(name="PositionTestRed", group="linearOpMode")
public class PositionTestRed extends AutonomousPrime2021 {
    @Override
    public void runOpMode(){
        mapObjects();
        waitForStart();
        //chute(10);

//        while(!isStopRequested()){
//            duckSpin(1,1);
//        }
        forwardEncoder(50, 0.5);



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