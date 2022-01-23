package org.firstinspires.ftc.teamcode;

public class ArmController extends AutonomousPrime2021 implements Runnable {



    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {

            int CurrentSlidePos = linearSlidePos();

            if(ArmDump){
                chute(0.5); //Dump Pos
            }
            else if((CurrentSlidePos<1250&&CurrentSlidePos>400)){ //was 1250, 200; 1250, 300; 1250, 400
                chute(0.15);
            }
            else if((CurrentSlidePos<=400 && CurrentSlidePos>-25)){ //was 200, 300, (mech change), 400,
                chute(0.0);
            }
            else if(CurrentSlidePos >= 1250){ //was 550
                chute(0.2);
            }



        }
    }
}
