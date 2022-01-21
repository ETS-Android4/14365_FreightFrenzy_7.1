package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="BlueFreight", group="linearOpMode")
public class BlueFreight extends AutonomousPrime2021 {
    @Override
    public void runOpMode() {
        mapObjects();
        waitForStart();


        leftEncoder(90,0.15);
        pause(3);
        rightEncoder(90,0.15);


    }
}