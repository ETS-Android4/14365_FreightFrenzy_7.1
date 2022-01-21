//*********************************************************************************
//                             -- CHANGELOG --                                    *
//  ____________________________________________________________________________  *
// |   DATE    |     NAME     |                   DESCRIPTION                   | *
// |___________|______________|_________________________________________________| *
// | 1/7/2021  | Jack         | Changed the drive system to utilize 2 gamepads  | *
// |           |              | to allow for 2 drivers                          | *
// |___________|______________|_________________________________________________| *
// | 1/22/2021 | Jack         | Updated to the second iteration of the advancer,| *
// |           |              | assorted bug fixes and style improvements       | *
// |___________|______________|_________________________________________________| *
//*********************************************************************************

package org.firstinspires.ftc.teamcode;

// TODO: Test variation of "LinearTeleOpMain" for getting distance sensor Return to Point working (LONG TERM)

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "RTPLinearTeleop", group = "Current")

public class LinearTeleOpDist extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor intake = null;

    private DcMotorEx launchLeft = null;
    private DcMotorEx launchRight = null;

    private double pusherPos = 0.35;
    private Servo pusher = null;

    protected DcMotorEx grabber = null;
    private Servo latch = null;

    double DriveSpeed=1;

    /*protected BNO055IMU imu;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    protected Orientation lastAngles = new Orientation();
    protected double globalAngle;
    protected double initialAngle;*/

    protected final double countPerRotation=537.6; //Was 753.2
    protected final double countPerDegree=0.07205357141; //Was 0.05142857142

    protected DistanceSensor backDist;
    protected double readBackDist;

    protected DistanceSensor frontDist;
    protected double readFrontDist;

    protected DistanceSensor leftDist;
    protected double readLeftDist;

    protected DistanceSensor rightDist;
    protected double readRightDist;

    protected double launchRightVelocity = 1020;
    protected double launchLeftVelocity = launchRightVelocity-20;

    protected BNO055IMU imu;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    protected Orientation lastAngles = new Orientation();
    protected double globalAngle;
    protected double initialAngle;

    protected double savedRightDist;
    protected double savedLeftDist;
    protected double savedFrontDist;
    protected double savedBackDist;

    protected double idealRightWall=65; //71
    protected double idealLeftWall=140; //128
    protected double idealFrontWall=185; //216
    protected double idealBackWall=128; //103

    double grabberPos = 0;

    public void updateLeftDist(){
        readLeftDist=leftDist.getDistance(DistanceUnit.CM);
    }

    public void updateDist(){
        readBackDist=backDist.getDistance(DistanceUnit.CM);
        readRightDist=rightDist.getDistance(DistanceUnit.CM);
        readFrontDist=frontDist.getDistance(DistanceUnit.CM);
        readLeftDist=leftDist.getDistance(DistanceUnit.CM);
    }

    public void zeroBotEncoder(double MotorPower){
        double newAngle = getAngle();
        telemetry.addData("zeroBot Initial ",initialAngle);

        telemetry.addData("New ",newAngle);
        telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
        //telemetry.update();
        while (Math.abs(newAngle - initialAngle) > 1){
            telemetry.addData("Zerobot Adj Initial ",initialAngle);
            telemetry.addData("New ",newAngle);
            telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
            //telemetry.update();
            newAngle = getAngle();
            ///*** Better to come up with a formula for how long to turn based on difference in the angle
            //** ie how many degress would rightTurn(0.1) get me and calculate value based on Math.abs(newAngle - initialAngle)
            //if (newAngle > initialAngle + 10 || newAngle > initialAngle - 10){
            if (newAngle > initialAngle){
                rightEncoder(Math.abs(newAngle - initialAngle), MotorPower);
            }else {
                leftEncoder(Math.abs(newAngle - initialAngle), MotorPower);
            }
        }

    }
    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }
    public void leftEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition((int)(pos/countPerDegree));
        frontLeft.setTargetPosition((int)(pos/countPerDegree));
        backRight.setTargetPosition((int)(pos/countPerDegree));
        backLeft.setTargetPosition((int)(pos/countPerDegree));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(MotorPower);
        frontLeft.setPower(MotorPower);
        backRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);

        while (frontLeft.isBusy()){

        }
    }
    public void strafeLeftEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(-cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(-cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void strafeRightEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(-cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(-cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void reverseEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(-cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(-cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void forwardEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(-cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(-cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void rightEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        frontRight.setTargetPosition((int)(-pos/countPerDegree));
        frontLeft.setTargetPosition((int)(-pos/countPerDegree));
        backRight.setTargetPosition((int)(-pos/countPerDegree));
        backLeft.setTargetPosition((int)(-pos/countPerDegree));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(MotorPower);
        frontLeft.setPower(MotorPower);
        backRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);

        while (frontLeft.isBusy()){

        }
    }
    public void launch(){
        pusher.setPosition(0.2);
        pause(0.5);
        pusher.setPosition(0.35);
        //pause(1);
    }
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs){

        }
    }
    public void safeLaunch(){
        while (opModeIsActive()) {
            if (!((launchLeft.getVelocity() >= launchLeftVelocity - 20 && launchLeft.getVelocity() <= launchLeftVelocity + 20) && (launchRight.getVelocity() >= launchRightVelocity - 20 && launchRight.getVelocity() <= launchRightVelocity + 20))) {
                sleep(100);
            } else {
                launch();
                break;
            }
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");

        launchLeft = hardwareMap.get(DcMotorEx.class, "launchLeft");
        launchRight = hardwareMap.get(DcMotorEx.class, "launchRight");

        pusher = hardwareMap.get(Servo.class, "intakeAdvance");

        grabber = hardwareMap.get(DcMotorEx.class, "grabber");
        latch = hardwareMap.get(Servo.class, "latch");

        backDist = hardwareMap.get(DistanceSensor.class, "backDist");
        //readBackDist=backDist.getDistance(DistanceUnit.CM);

        rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");
        //readRightDist=backDist.getDistance(DistanceUnit.CM);

        frontDist = hardwareMap.get(DistanceSensor.class, "frontDist");
        //readFrontDist=backDist.getDistance(DistanceUnit.CM);

        leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
        //readLeftDist=backDist.getDistance(DistanceUnit.CM);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        launchLeft.setDirection(DcMotor.Direction.REVERSE);
        launchRight.setDirection(DcMotor.Direction.FORWARD);

        //**** The IMU and associated variables ************

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        initialAngle = getAngle();
        telemetry.addData("Status", "Initialized");

        launchLeft.setPower(1);
        launchRight.setPower(1);
        launchLeft.setVelocity(1020);
        launchRight.setVelocity(1020);

        runtime.reset();
        pusher.setPosition(pusherPos);
        grabber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            if ((launchLeft.getVelocity() >= launchLeftVelocity - 20 && launchLeft.getVelocity() <= launchLeftVelocity + 20) && (launchRight.getVelocity() >= launchRightVelocity - 20 && launchRight.getVelocity() <= launchRightVelocity + 20)) {
                telemetry.addData("-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------" +
                        "-----------------------------------------------------", launchRightVelocity);
            }
            telemetry.addData("Left Velocity: ", launchLeft.getVelocity());
            telemetry.addData("Right Velocity: ", launchRight.getVelocity());
            //telemetry.update();

            // <Driver 1>

            //telemetry.addData("Real Grabber position ",grabber.getCurrentPosition()); //jacob wanted this- arm
            //telemetry.addData("Expected Grabber position ",grabberPos);

            double norm = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if (gamepad1.right_trigger >= 0.2) {
                DriveSpeed = 0.35;
            } else {
                DriveSpeed = 1;
            }

            frontLeft.setPower((norm - yaw + strafe) * DriveSpeed);
            backLeft.setPower(-(-norm + yaw + strafe) * DriveSpeed);
            frontRight.setPower(-(norm + yaw - strafe) * DriveSpeed);
            backRight.setPower((-norm - yaw - strafe) * DriveSpeed);

        /*telemetry.addData("frontLeft",(norm + strafe + yaw));
        telemetry.addData("backLeft",(norm - strafe + yaw));
        telemetry.addData("frontRight",(norm - strafe - yaw));
        telemetry.addData("backRight",(norm + strafe - yaw));*/

        /*if (gamepad1.dpad_left) {
            initialAngle = getAngle();
            updateDist();
            savedBackDist=readBackDist;
            savedFrontDist=readFrontDist;
            savedRightDist=readBackDist;
            savedLeftDist=readFrontDist;


            double count = 0;
            if(readBackDist>500 && readFrontDist>500 || readRightDist>500 && readLeftDist>500 && count<4){
                updateDist();
                count++;
            }
            if(count==4){
                count=0;
                telemetry.addData("Angle Get: ", "Failed!");
                telemetry.update();
            }
        }*/

            if (gamepad1.dpad_left) {
                initialAngle = getAngle();
            }
            if (gamepad1.dpad_right) {
                launchLeftVelocity=1000;
                launchRightVelocity=1020;
                launchLeft.setVelocity(launchLeftVelocity);
                launchRight.setVelocity(launchRightVelocity);

                zeroBotEncoder(1);
                updateDist();
                if (readRightDist > 500 && readLeftDist > 500) {
                    strafeLeftEncoder(30, 1);
                    updateDist();
                } else {
                    if (readLeftDist > 500) {
                        double moveRightDist = readRightDist - idealRightWall;
                        strafeRightEncoder(moveRightDist, 1);
                    } else if (readRightDist > 500) {
                        double moveLeftDist = readLeftDist - idealLeftWall;
                        moveLeftDist = moveLeftDist * -1;
                        strafeRightEncoder(moveLeftDist, 1);
                    }
                }

                zeroBotEncoder(1);
                updateDist();
                if (readFrontDist > 500 && readBackDist > 500) {
                    reverseEncoder(30, 1);
                    updateDist();
                } else {
                    if (readBackDist > 500) {
                        double moveBackDist = readBackDist - idealBackWall;
                        reverseEncoder(moveBackDist, 1);
                    } else if (readFrontDist > 500) {
                        double moveFrontDist = readFrontDist - idealFrontWall;
                        moveFrontDist = moveFrontDist * -1;
                        reverseEncoder(moveFrontDist, 1);
                    }
                }
                safeLaunch();
                safeLaunch();
                safeLaunch();
            }

            /*if(savedRightDist>500){
                double moveDist = 182.88-(readLeftDist+37);
                strafeRightEncoder(moveDist, 1);
            }else{
                double moveDist = readRightDist - savedRightDist;
                strafeRightEncoder(moveDist, 1);
            }
            zeroBotEncoder(1);

            if(savedBackDist>500){
                double moveDist = 365.76-(readFrontDist+37);
                reverseEncoder(moveDist, 1);
            }else{
                double moveDist = readBackDist - savedBackDist;
                reverseEncoder(moveDist, 1);
            }
            zeroBotEncoder(1);

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/


            // <Driver 2>
            // CONTROLS:
            // left trigger:  slow down the launcher
            // x:             run advancer
            // right bumper:  run intake backward
            // y:             KILL EVERYTHING

            if (gamepad2.dpad_up) {
                launchLeft.setPower(1);
                launchRight.setPower(1);
                launchRightVelocity = 1020;
                launchLeftVelocity = launchRightVelocity - 20;
            } else if (gamepad2.dpad_down) {
                launchLeft.setPower(1);
                launchRight.setPower(1);
                launchRightVelocity = 960;
                launchLeftVelocity = launchRightVelocity - 20;
            } else if (gamepad2.dpad_right) {
                launchLeft.setPower(1);
                launchRight.setPower(1);
                launchRightVelocity = 1000;
                launchLeftVelocity = launchRightVelocity - 20;
            } else if (gamepad2.dpad_left) {
                launchLeft.setPower(1);
                launchRight.setPower(1);
                launchRightVelocity = 1000;
                launchLeftVelocity = launchRightVelocity - 20;
            }
            launchLeft.setVelocity(launchLeftVelocity);
            launchRight.setVelocity(launchRightVelocity);

//        if(gamepad2.a){
//            strafeLeftEncoder(19.0, 1.0);
//            launch();
//            strafeLeftEncoder(20.0, 1.0); //was 19.0
//            launch();
//        }

        /*if(!gamepad2.y) {
            if(gamepad2.x || gamepad1.x) {
                pusherPos = 0.2;
            } else {
                pusherPos = 0.35;
            }
            pusher.setPosition(pusherPos);
            if(!gamepad2.left_bumper && !gamepad1.left_bumper) {
                if(gamepad2.right_bumper) {
                    intake.setPower(-0.5);
                } else {
                    intake.setPower(1);
                }
            } else {
                intake.setPower(0);
            }
        }*/
            if (gamepad1.x || gamepad2.x) {
                pusherPos = 0.2;
            } else {
                pusherPos = 0.35;
            }
            pusher.setPosition(pusherPos);

            if (gamepad1.b || gamepad2.b) {
                while (opModeIsActive()) {
                    if (!((launchLeft.getVelocity() >= launchLeftVelocity - 20 && launchLeft.getVelocity() <= launchLeftVelocity + 20) && (launchRight.getVelocity() >= launchRightVelocity - 20 && launchRight.getVelocity() <= launchRightVelocity + 20))) {
                        sleep(5);
                    } else {
                        launch();
                        break;
                    }
                }
            }

            if (!gamepad2.left_bumper && !gamepad1.left_bumper) {
                if (gamepad2.right_bumper) {
                    intake.setPower(-0.75);
                } else {
                    intake.setPower(1);
                }
            } else {
                intake.setPower(0);
            }


            if (gamepad2.right_stick_y < 0.5) {
                grabberPos += 4;
                grabber.setTargetPosition((int) (grabberPos));
            }

            if (gamepad2.right_stick_y > -0.5) {
                grabberPos -= 4;
                grabber.setTargetPosition((int) (grabberPos));
            }
            grabber.setPower(1);
            grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (gamepad2.left_trigger > 0.05) {
                latch.setPosition(0.95);
            } else {
                latch.setPosition(0.6);
            }
            telemetry.update();
        }
    }
}