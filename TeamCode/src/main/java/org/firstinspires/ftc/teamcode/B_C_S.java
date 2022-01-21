package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;

// TODO: General test program, update with whatever tasks as necessary

@Autonomous(name="Blue_Carousel_Storage", group="linearOpMode")
public class B_C_S extends AutonomousPrime2021 {
    @Override
    public void runOpMode(){
        mapObjects();
        waitForStart();

        //boolean CarouselSpun = false;

        forwardEncoder(5,1);

        rightEncoder(90,1);

        forwardEncoder(53, 0.75);
        pause(0.25);
       // duckSpin(5, 1);
        //CarouselSpun = true;
        strafeLeftEncoder(10, 1);
        leftEncoder(90, 1);
        pause(1.5);
        forwardEncoder(40, 1);
        pause(0.25);
        leftEncoder(90,1);
        forwardEncoder(100,1);
        pause(0.25);

        rightEncoder(90,1);

        //zeroBotEncoder(0.5);
        strafeRightEncoder(115,1);
        forwardEncoder(45, 1);



        //hi logan -Jacob
    }
}