package org.firstinspires.ftc.teamcode;

public class NewArmController extends AutonomousPrime2021 implements Runnable {



    @Override
    public void run() {
        linearSlide = AutonomousPrime2021.linearSlide;

        intake = AutonomousPrime2021.intake;

        /*linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setTargetPositionTolerance(10);
        linearSlide.getCurrentPosition();*/

        chute = AutonomousPrime2021.chute;

        //GoingUp=true;


        while (!Thread.currentThread().isInterrupted()) {

            ArmDump=NewAutonomousProgramBase.ArmDump;

            //GoingUp=AutonomousProgramBase.GoingUp;

            int CurrentSlidePos = linearSlide.getCurrentPosition();

            if(NewAutonomousProgramBase.intakeSpin){
                if(NewAutonomousProgramBase.GoingUp){
                    intake.setPower(0.25);
                }
                else{
                    intake.setPower(-0.5);
                }
            }
            else{
                intake.setPower(0);
            }



            if(ArmDump){
                chute.setPosition(0.5); //Dump Pos
            }
            else if((CurrentSlidePos<1250&&CurrentSlidePos>400)){ //was 1250, 200; 1250, 300; 1250, 400
                chute.setPosition(0.15);
            }
            else if((CurrentSlidePos<=400 && CurrentSlidePos>-25)){ //was 200, 300, (mech change), 400,
                chute.setPosition(0.0);
            }
            /*else if((CurrentSlidePos<=400 && CurrentSlidePos>-25) && gamepad2.left_stick_y>0.1){ //was 200
                chuteEx.setPwmDisable();
            }*/
            else if(CurrentSlidePos >= 1250){ //was 550
                chute.setPosition(0.2);
            }



        }
    }
}
