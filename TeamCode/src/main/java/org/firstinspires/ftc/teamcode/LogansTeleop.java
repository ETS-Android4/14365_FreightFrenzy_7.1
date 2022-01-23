package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class LogansTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        int ArmMinLimit = -25;
        //int ArmMaxLimit = 2025;
        int ArmMaxLimit = 2750;



        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        DcMotor intake = hardwareMap.dcMotor.get("intake");

//        DcMotorEx linearSlide = null;
//
//        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
//        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx linearSlide = hardwareMap.get(DcMotorEx.class,("linearSlide"));
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setTargetPositionTolerance(10);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Servo intakeDrop = hardwareMap.servo.get("intakeDrop");

        Servo chute = hardwareMap.servo.get("chute");
        ServoImplEx chuteEx = (ServoImplEx) chute;


        waitForStart();

        double linearSlidePosition = 0;

        if(isStopRequested()) return;

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        int ArmPosDPad = 0;

        boolean AutoArm = false;

        int linearSlideCoefficient = 7; //WAS 5; MAY BREAK THINGS, CHANGE BACK IF IT DOES

        double chutePos=0;

        while(opModeIsActive()) {
            double coefficient = 1;
            if(gamepad1.right_bumper){
                coefficient=0.25;
            }
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((-y - x - rx) / denominator) * coefficient;
            double backLeftPower = ((-y + x - rx) / denominator) * coefficient;
            double frontRightPower = ((y - x - rx) / denominator) * coefficient;
            double backRightPower = ((y + x - rx) / denominator) * coefficient;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            linearSlide.setPower(gamepad1.right_stick_y);



            if(timer.time()>150){
                if(gamepad2.dpad_up){
                    ArmPosDPad++;
                    timer.reset();
                }
                else if(gamepad2.dpad_down){
                    ArmPosDPad--;
                    timer.reset();
                }

            }

            if(gamepad2.a){
                intake.setPower(-1);
            }
            else if(gamepad2.y){
                intake.setPower(1);
            }
            else{
                intake.setPower(0);
            }


            if(gamepad2.dpad_up || gamepad2.dpad_down){
                if(ArmPosDPad==0){
                    //Bucket collect position
                    linearSlidePosition=0;
                }
                else if (ArmPosDPad==1){
                    //1st Wobble Level
                    linearSlidePosition=202; //was 740
                }
                else if(ArmPosDPad==2){
                    //2nd Wobble Level
                    linearSlidePosition=251; //was 1140
                }
                else if (ArmPosDPad==3){
                    //3rd Wobble Level
                    linearSlidePosition=325; //was 1640
                }
                else if(ArmPosDPad<0){
                    //D-Pad accidentally went too far down; reset to 0
                    ArmPosDPad=0;
                }
                else{
                    //D-Pad accidentally went too far up; reset to 3
                    ArmPosDPad=3;
                }
            }

            int CurrentSlidePos = linearSlide.getCurrentPosition();






            if(gamepad2.b){
                chuteEx.setPwmEnable();
                chute.setPosition(0.5); //Dump Pos
            }
            else if((CurrentSlidePos<1250&&CurrentSlidePos>400)){ //was 1250, 200; 1250, 300; 1250, 400
                chuteEx.setPwmEnable();
                chute.setPosition(0.15);
            }
            else if((CurrentSlidePos<=400 && CurrentSlidePos>-25)){ //was 200, 300, (mech change), 400,
                chuteEx.setPwmEnable();
                chute.setPosition(0.0);
            }
            /*else if((CurrentSlidePos<=400 && CurrentSlidePos>-25) && gamepad2.left_stick_y>0.1){ //was 200
                chuteEx.setPwmDisable();
            }*/
            else if(CurrentSlidePos >= 1250){ //was 550
                chuteEx.setPwmEnable();
                chute.setPosition(0.2);
            }
            /*else if(CurrentSlidePos>130 && CurrentSlidePos<137 && gamepad2.left_stick_y>0.1) {
                chuteEx.setPwmEnable();
                chute.setPosition(0.4);
            }*/







            if(gamepad2.left_stick_y>0.1 || gamepad2.left_stick_y<-0.1){
                linearSlidePosition-=gamepad2.left_stick_y; //"Subtract" gamepad value from linear slide position (needed due to arm motor being reversed)


                //"If gone above limit, don't" -Jacob
                if((linearSlidePosition*linearSlideCoefficient)>ArmMaxLimit && gamepad2.left_stick_y<-0.1){  //IF (linear slide position > arm max) AND (gamepad is going UP)
                    linearSlidePosition+=gamepad2.left_stick_y; //"Add" gamepad value from linear slide position (negating what it does above ^)
                    telemetry.addData("Past Position And Moving Wrong: ", true); //Telemetry read out
                }
                else if((linearSlidePosition*linearSlideCoefficient)<ArmMinLimit && gamepad2.left_stick_y>0.1) { //IF (linear slide position < arm max) AND (gamepad is going DOWN)
                    linearSlidePosition += gamepad2.left_stick_y; //"Add" gamepad value from linear slide position (negating what it does above ^)
                    telemetry.addData("Past Position And Moving Wrong: ", true); //Telemetry read out
                }
            }
            //Everything below sets target position based off of linear slide position above ^
            linearSlide.setTargetPosition((int) (linearSlideCoefficient * linearSlidePosition));
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setPower(1);


            telemetry.addData("Linear Slide Raw Ideal Pos:", linearSlidePosition);
            telemetry.addData("Linear Slide True Pos:", linearSlide.getTargetPosition());
            telemetry.addData("DPad Level:", ArmPosDPad);
            telemetry.update();



        }
    }
}
