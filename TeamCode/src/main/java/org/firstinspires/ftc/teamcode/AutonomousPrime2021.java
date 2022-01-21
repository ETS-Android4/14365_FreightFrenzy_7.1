package org.firstinspires.ftc.teamcode;
import android.os.Environment;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Disabled
@Autonomous(name="AutonomousPrime2021", group="Linear Opmode")
public class AutonomousPrime2021 extends LinearOpMode {

    /*
     ********************************
     *   SETUP POSITION VARIABLES   *
     ********************************
     */
    public int x = 0;
    public int y = 0;
    public int angle = 0;

    /*
     ***************************
     *   SETUP WRITE TO FILE   *
     ***************************
     */

    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line = "";
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    /*
     **************************
     *   SETUP DRIVE MOTORS   *
     **************************
     */

    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;
    protected double MotorPower = 1.0;

    protected final double  COUNT_PER_ROTATION = 15.641025641;
    protected final double  COUNT_PER_DEGREE = 0.16;



    /*
     *************************
     *   SETUP MISC MOTORS   *
     *************************
     */





    protected DcMotorEx intake = null;
    protected DcMotorEx linearSlide = null;
    protected DcMotorEx dArm = null;





    /*
     ********************
     *   SETUP SERVOS   *
     ********************
     */

    //protected Servo intakeDrop;
    protected CRServo duckSpinny = null;
    protected Servo chute = null;



    /*
     *****************
     *   SETUP IMU   *
     *****************
     */

    protected BNO055IMU imu;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    protected double globalAngle;
    protected double initialAngle;
    protected Orientation angles = new Orientation();
    double cleanedUpAngle = 0;

    protected Orientation lastAngles = new Orientation();


    /*
     ******************************
     *   SETUP DISTANCE SENSORS   *
     ******************************
     */

    protected DistanceSensor GroundFront;
    protected double GroundFrontDist = 0;
    protected DistanceSensor GroundBack;
    protected double GroundBackDist = 0;

    protected DistanceSensor Left;
    protected double LeftDist = 0;
    protected DistanceSensor Right;
    protected double RightDist = 0;

    protected DistanceSensor FrontLeft;
    protected double FrontLeftDist = 0;
    protected DistanceSensor FrontRight;
    protected double FrontRightDist = 0;

    protected DistanceSensor BackLeft;
    protected double BackLeftDist = 0;
    protected DistanceSensor BackRight;
    protected double BackRightDist = 0;

    /**
     * Mapping all empty objects to control hub objects
     */
    public void mapObjects(){

        /*
         ************************
         *   TELEMETRY READOUT  *
         ************************
         */

        telemetry.addData("Status","Initialized");
        telemetry.update();

        /*
         *****************
         *   MAP MOTORS  *
         *****************
         */

        frontLeft=hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRight=hardwareMap.get(DcMotorEx.class,"frontRight");
        backLeft=hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight=hardwareMap.get(DcMotorEx.class,"backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setTargetPositionTolerance(50);
        backLeft.setTargetPositionTolerance(50);
        frontRight.setTargetPositionTolerance(50);
        backRight.setTargetPositionTolerance(50);





        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);

        linearSlide = hardwareMap.get(DcMotorEx.class, "linearSlide");
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);



        dArm = hardwareMap.get(DcMotorEx.class, "dArm");



        /*
         *****************
         *   MAP SERVOS  *
         *****************
         */

        duckSpinny=hardwareMap.get(CRServo.class,"duckSpinny");
        duckSpinny.setDirection(CRServo.Direction.FORWARD);

        chute = hardwareMap.get(Servo.class, "chute");


        //intakeDrop = hardwareMap.get(Servo.class, "intakeDrop");



        /*
         ***************************
         *   MAP DISTANCE SENSORS  *
         ***************************
         */

        //GroundFront, GroundBack, Left, Right, FrontLeft, FrontRight, BackLeft, BackRight

        GroundFront=hardwareMap.get(DistanceSensor.class,"GroundFront");
        GroundFrontDist=GroundFront.getDistance(DistanceUnit.CM);

        GroundBack = hardwareMap.get(DistanceSensor.class, "GroundBack");

        Left = hardwareMap.get(DistanceSensor.class, "Left");
        LeftDist = Left.getDistance(DistanceUnit.CM);

        Right = hardwareMap.get(DistanceSensor.class, "Right");
        RightDist = Right.getDistance(DistanceUnit.CM);

        FrontLeft = hardwareMap.get(DistanceSensor.class, "FrontLeft");
        FrontLeftDist = FrontLeft.getDistance(DistanceUnit.CM);

        FrontRight = hardwareMap.get(DistanceSensor.class, "FrontRight");
        FrontRightDist = FrontRight.getDistance(DistanceUnit.CM);

        BackLeft = hardwareMap.get(DistanceSensor.class, "BackLeft");
        BackLeftDist = BackLeft.getDistance(DistanceUnit.CM);

        BackRight = hardwareMap.get(DistanceSensor.class, "BackRight");
        BackRightDist = BackRight.getDistance(DistanceUnit.CM);


        /*
         ****************
         *   IMU SETUP  *
         ****************
         */

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(50);
            idle();
        }
        initialAngle = getAngle();


    }

    /**
     * Necessary override to inherit data within mapObjects()
     */
    @Override
    public void runOpMode(){
        mapObjects();
    }

    /**
     * Begin logging to file
     */
    public void Log(String filename, boolean logTime) {
        if (logTime) startTime = System.nanoTime();
        this.logTime = logTime;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Stop logging to file
     */
    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Spin duck for seconds
     */
    public void duckSpin(double power, double secs){
        duckSpinny.setPower(power);
        pause(secs);
        duckSpinny.setPower(0);
    }

    /**
     * Move chute conveyor
     */


    public void chute(double pos){
        chute.setPosition(pos);
        telemetry.addData("Servo Pos: ", pos);
        telemetry.update();
    }

    /**
     * Move linear slide to given position
     */

    public void linearSlide(double pos, double MotorPower) {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        linearSlide.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlide.setPower(MotorPower);
    }

//    public void chuteTime(double secs, double MotorPower){
//        chute.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        chute.setPower(MotorPower);
//
//        pause(secs);
//        chute.setPower(0);
//    }

    /**
     * Drop intake device
     */
    public void dropIntake(double position){
        //intakeDrop.setPosition(position);
    }
    /**
     * Spin intake
     */
    public void spinIntake(double MotorPower){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(MotorPower);
    }
    /**
     * Get intake speed across seconds
     */
    public void speedOfIntake(double seconds){
        int prevPosition = intake.getCurrentPosition();
        ElapsedTime readTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        readTime.reset();
        if(readTime.time()<=seconds){
            double speed = (double) (intake.getCurrentPosition()-prevPosition)/readTime.time();
            telemetry.addData("Speed: ", speed);
            telemetry.update();
            prevPosition = intake.getCurrentPosition();
            timer.reset();
        }
    }
    /**
     * Stop spinning intake
     */
    public void stopSpinIntake(){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
    }
    /**
     * Update All Dist Sensor Values
     */
    public void updateAllDist(){
       GroundFrontDist=GroundFront.getDistance(DistanceUnit.CM);
        GroundBackDist = GroundBack.getDistance(DistanceUnit.CM);
        LeftDist = Left.getDistance(DistanceUnit.CM);
        RightDist = Right.getDistance(DistanceUnit.CM);
        FrontLeftDist = FrontLeft.getDistance(DistanceUnit.CM);
        FrontRightDist = FrontRight.getDistance(DistanceUnit.CM);
        BackLeftDist = BackLeft.getDistance(DistanceUnit.CM);
        BackRightDist = BackRight.getDistance(DistanceUnit.CM);
    }

    /**
     * Update All Side Sensor Values
     */
    public void updateSideDist(){
        LeftDist = Left.getDistance(DistanceUnit.CM);
        RightDist = Right.getDistance(DistanceUnit.CM);
        FrontLeftDist = FrontLeft.getDistance(DistanceUnit.CM);
        FrontRightDist = FrontRight.getDistance(DistanceUnit.CM);
        BackLeftDist = BackLeft.getDistance(DistanceUnit.CM);
        BackRightDist = BackRight.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Left Sensor Value
     */
    public void updateLeftDist(){
        LeftDist = Left.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Right Sensor Value
     */
    public void updateRightDist(){
        RightDist = Right.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Front Sensor Values
     */
    public void updateFrontDist(){
        FrontLeftDist = FrontLeft.getDistance(DistanceUnit.CM);
        FrontRightDist = FrontRight.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Back Sensor Values
     */
    public void updateBackDist(){
        BackLeftDist = BackLeft.getDistance(DistanceUnit.CM);
        BackRightDist = BackRight.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Front Left Sensor Value
     */
    public void updateFrontLeftDist(){
        FrontLeftDist = FrontLeft.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Front Right Sensor Value
     */
    public void updateFrontRightDist(){
        FrontRightDist = FrontRight.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Back Left Sensor Value
     */
    public void updateBackLeftDist(){
        BackLeftDist = BackLeft.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Back Right Sensor Value
     */
    public void updateBackRightDist(){
        BackRightDist = BackRight.getDistance(DistanceUnit.CM);
    }

    /**
     * Update Ground Sensor Values
     */

    //Can't use this function because the build team messed up plugging in these distance sensors, thanks build team! :) -Cole   \   /
    public void updateGroundDist(){                                                                                           //  o  o
        GroundFrontDist=GroundFront.getDistance(DistanceUnit.CM);                                                             //  ----\
        GroundBackDist = GroundBack.getDistance(DistanceUnit.CM);
    }



    /**
     * Update file with String line, then newline
     */
    public void update() {
        if (disabled) return;
        try {
            if (logTime) {
                long timeDifference = System.nanoTime()-startTime;
                line = timeDifference/1E9+","+line;
            }
            fileWriter.write(line+"\n");
            line = "";
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Add data to String line
     */
    public void addData(String data) {
        if (disabled){
            return;
        }
        if (!line.equals("")) {
            line += ",";
        }
        line += data;
    }
    public void addData(Object data) { addData(data.toString()); }
    public void addData(boolean data) { addData(String.valueOf(data)); }
    public void addData(byte data) { addData(String.valueOf(data)); }
    public void addData(char data) { addData(String.valueOf(data)); }
    public void addData(short data) { addData(String.valueOf(data)); }
    public void addData(int data) { addData(String.valueOf(data)); }
    public void addData(long data) { addData(String.valueOf(data)); }
    public void addData(float data) { addData(String.valueOf(data)); }
    public void addData(double data) { addData(String.valueOf(data)); }



    /*
     *******************************************************************************************
     *   WRITE AN IMU FUNCTION THAT ALLOWS US TO RESET THE IMU ANGLE READING THRU A VARIABLE   *
     *******************************************************************************************
     */

    //NOTES: Should track 0-360 degrees and reset at 360; make an IMU "setAngle()" that takes IN an angle and sets that to globalAngle (so IMU tracks from there)

    /**
     * Resetting the robots current angle to 0
     */

    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Telling us the robots current angle
     */

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
        telemetry.addData("Global Angle:", globalAngle);
        telemetry.update();
        return globalAngle;

    }

    /**
     * Setting the robots current angle to the angle given
     */

    public void setAngle(double angle) {
        globalAngle = angle;
        telemetry.addData("New Global Angle:", globalAngle);
        telemetry.update();
    }


    /*public double getAngleOld(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        if(globalAngle>=360){
            globalAngle -= 360;
            return globalAngle;
        }
        else if (globalAngle<=0){
            globalAngle += 360;
            return globalAngle;
        }
        else{
            return globalAngle;
        }

    }*/

    /**
     * Get heading readout from IMU
     */
//    public double getAngle(){
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        telemetry.addData("Initial Relative Heading: ", angles.firstAngle);
//        double cleanedUpAngle = 0;
//
//        if(angles.firstAngle<0){
//            cleanedUpAngle=angles.firstAngle+=360;
//        }
//        else{
//            cleanedUpAngle=angles.firstAngle;
//        }
//        telemetry.addData("Cleaned Up Relative Heading: ", cleanedUpAngle);
//        telemetry.update();
//        return cleanedUpAngle;
//    }



    /**
     * Get angle readout from IMU + a passed offset value
     */
    /*public double getAngleOffset(double angle){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Initial Relative Heading: ", angles.firstAngle);

        cleanedUpAngle = angle + angles.firstAngle;

        if(cleanedUpAngle<0){
            cleanedUpAngle=cleanedUpAngle+=360;
        }
        else if(cleanedUpAngle>=360){
            cleanedUpAngle=cleanedUpAngle-=360;
        }
        telemetry.addData("Cleaned Up Relative Heading: ", cleanedUpAngle);
        return(cleanedUpAngle);


    }*/

    /**
     * Turn robot to set angle based off current angle;
     * Takes angleGoal, your universal angle you want to face; angleOffset, or a value determined by some other position tracking device to tell your angle; and MotorPower
     * STILL NEEDS EXTENSIVE TESTING TO BE READY FOR USE.
     */
    public void zeroBotEncoderOffset(double angleGoal, double angleOffset, double MotorPower){

    }

    /**
     * Zerobot, moving the robot to the initial angle
     */

    public void zeroBotEncoder(double MotorPower){
        double newAngle = getAngle();
        double deltaAngle = initialAngle-newAngle;
        if(deltaAngle<=-180){
            deltaAngle+=360;
        }
        else if(deltaAngle>180){
            deltaAngle+=360;
        }
        leftEncoder(deltaAngle,MotorPower);
    }

    /**
     * Zerobot, except it moves to the initial angle which is offset by a given value
     */

    public void zeroBotEncoderOffset(double AngleOffset, double MotorPower){
        double newAngle = getAngle();
        double deltaAngle = (initialAngle-newAngle)+AngleOffset;
        if(deltaAngle<=-180){
            deltaAngle+=360;
        }
        else if(deltaAngle>180){
            deltaAngle+=360;
        }
        leftEncoder(deltaAngle,MotorPower);
    }


    /*public double getAngleOffset(double offset){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        if(globalAngle + offset >=360){
            globalAngle -= 360;
            return globalAngle + offset;
        }
        else if (globalAngle + offset <=0){
            globalAngle += 360;
            return globalAngle + offset;
        }
        else{
            return globalAngle + offset;
        }

    }*/

    /**
     * Zero Bot to "initial angle" using IMU angle readout
     */
    /*public void oldZeroBot(double MotorPower){
        double newAngle = getAngleOld();
        telemetry.addData("zeroBot Initial ",initialAngle);
        telemetry.addData("New ",newAngle);
        telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
        telemetry.update();
        while (Math.abs(newAngle - initialAngle) > 3 && opModeIsActive()){ //was >5
            telemetry.addData("Zerobot Adj Initial ",initialAngle);
            telemetry.addData("New ",newAngle);
            telemetry.addData("Diff ", (newAngle - initialAngle));
            telemetry.update();
            newAngle = getAngleOld();
            if (newAngle - initialAngle <=180 ){
                rightEncoder(Math.abs(newAngle - initialAngle),MotorPower); //Works
            }else {
                leftEncoder((newAngle - initialAngle)-180,MotorPower); //Doesn't work
            }
        }
    }*/

    /**
     * Move arm up by pos at the speed passed
     */
    public void dArm(double pos, double MotorPower){
        dArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        dArm.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        dArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dArm.setPower(MotorPower);

        /*while (opModeIsActive() && (dArm.isBusy()){
            telemetry.addData("D Arm ",dArm.isBusy());
            telemetry.update();
        }*/

    }

    /**
     * Move arm up by pos at the speed passed
     */
    public void dArmWait(double pos, double MotorPower){
        dArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        dArm.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        dArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dArm.setPower(MotorPower);

        while (opModeIsActive() && (dArm.isBusy())){
            telemetry.addData("D Arm ",dArm.isBusy());
            telemetry.update();
        }

    }

    /**
     * Move robot forwards by cm
     */
    public void forwardEncoder(double pos, double MotorPower){ //1 pos = 25 cm
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        frontLeft.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        frontRight.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        backLeft.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        backRight.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backRight.setPower(MotorPower);
        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())){
            telemetry.addData("FL ",frontLeft.isBusy());
            telemetry.addData("FR ",frontRight.isBusy());
            telemetry.addData("BL ", backLeft.isBusy());
            telemetry.addData("BR ",backRight.isBusy());
            telemetry.update();
        }

    }

    /**
     * Move robot backwards by cm
     */
    public void reverseEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        frontLeft.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));
        frontRight.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));
        backLeft.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));
        backRight.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backRight.setPower(MotorPower);
        while (opModeIsActive() && (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())){
            telemetry.addData("FL ",frontLeft.isBusy());
            telemetry.addData("FR ",frontRight.isBusy());
            telemetry.addData("BL ", backLeft.isBusy());
            telemetry.addData("BR ",backRight.isBusy());
            telemetry.update();
        }
    }

    /**
     * Strafe robot left by cm
     */
    public void strafeLeftEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        frontLeft.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));
        frontRight.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        backLeft.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        backRight.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }

    /**
     * Strafe robot right by cm
     */
    public void strafeRightEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        frontLeft.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        frontRight.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));
        backLeft.setTargetPosition((int)(-cmOffset* COUNT_PER_ROTATION));
        backRight.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (opModeIsActive() && frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }

    /**
     * Turn robot left by degrees
     */
    public void leftEncoder(double degrees, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition((int)(degrees/ COUNT_PER_DEGREE));
        frontLeft.setTargetPosition((int)(-degrees/ COUNT_PER_DEGREE));
        backRight.setTargetPosition((int)(degrees/ COUNT_PER_DEGREE));
        backLeft.setTargetPosition((int)(-degrees/ COUNT_PER_DEGREE));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(MotorPower);
        frontLeft.setPower(MotorPower);
        backRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);

        while (opModeIsActive() && frontLeft.isBusy()){

        }
    }

    /**
     * Turn robot right by degrees
     */
    public void rightEncoder(double degrees, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setTargetPosition((int)(-degrees/ COUNT_PER_DEGREE));
        frontLeft.setTargetPosition((int)(degrees/ COUNT_PER_DEGREE));
        backRight.setTargetPosition((int)(-degrees/ COUNT_PER_DEGREE));
        backLeft.setTargetPosition((int)(degrees/ COUNT_PER_DEGREE));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(MotorPower);
        frontLeft.setPower(MotorPower);
        backRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);

        while (opModeIsActive() && frontLeft.isBusy()){

        }
    }

    /**
     * Going backwards for a given amount of time
     */

    public void reverseTime(double secs, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setPower(-1 * MotorPower);
        frontRight.setPower(-1 * MotorPower);
        backLeft.setPower(-1 * MotorPower);
        backRight.setPower(-1 * MotorPower);

        pause(secs);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }



    /**
     * Pause for seconds passed
     */
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs  && opModeIsActive() ){

        }
    }


}
