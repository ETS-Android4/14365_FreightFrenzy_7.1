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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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

// TODO: Prepare  code specifically for the mechanical ideas we came up with

@Disabled
@Autonomous(name="AutonomousPrime2020", group="Linear Opmode")
public class AutonomousPrime2020 extends LinearOpMode {

    /*
    *************
    *   SETUP   *
    *************
    */

    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line = "";
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;

    protected DcMotorEx launchLeft = null;
    protected DcMotorEx launchRight = null;

    protected DcMotorEx intake = null;
    protected DcMotorEx grabber = null;

    protected Servo latch;

    protected double MotorPower = 1.0;

    protected final double  COUNT_PER_ROTATION = 537.6; //Was 753.2
    protected final double  COUNT_PER_DEGREE = 0.07205357141; //Was 0.05142857142

    /*protected static  double NEW_P = 6.0;// was 8.0
    protected static  double NEW_I = 0.05;
    protected static  double NEW_D = 0.0;
    protected static  double NEW_F = 12.0;
    protected static int tolerance = 10;*/

    protected BNO055IMU imu;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    protected Orientation lastAngles = new Orientation();
    protected double globalAngle;
    protected double initialAngle;

    protected Servo intakeAdvance;

    protected Servo wobbleRelease;

    protected DistanceSensor backDist;
    protected double readBackDist;

    protected DistanceSensor frontDist;
    protected double readFrontDist;

    protected DistanceSensor leftDist;
    protected double readLeftDist;

    protected DistanceSensor rightDist;
    protected double readRightDist;

    protected ColorSensor indicator;

    public void mapObjects(){
        /*
        ****************************************
        *   MAP OBJECTS (WITHIN CONTROL HUB)   *
        ****************************************
        */

        telemetry.addData("Status","Initialized");
        telemetry.update();

        indicator = hardwareMap.get(ColorSensor.class, "indicator");

        frontLeft=hardwareMap.get(DcMotorEx.class,"frontLeft");
        frontRight=hardwareMap.get(DcMotorEx.class,"frontRight");
        backLeft=hardwareMap.get(DcMotorEx.class,"backLeft");
        backRight=hardwareMap.get(DcMotorEx.class,"backRight");

        launchLeft=hardwareMap.get(DcMotorEx.class,"launchLeft");
        launchRight=hardwareMap.get(DcMotorEx.class,"launchRight");

        intake=hardwareMap.get(DcMotorEx.class,"intake");

        grabber=hardwareMap.get(DcMotorEx.class,"grabber");
        latch=hardwareMap.get(Servo.class,"latch");

        backDist=hardwareMap.get(DistanceSensor.class, "backDist");
        readBackDist=backDist.getDistance(DistanceUnit.CM);

        rightDist=hardwareMap.get(DistanceSensor.class, "rightDist");
        readRightDist=backDist.getDistance(DistanceUnit.CM);

        frontDist=hardwareMap.get(DistanceSensor.class, "frontDist");
        readFrontDist=backDist.getDistance(DistanceUnit.CM);

        leftDist=hardwareMap.get(DistanceSensor.class, "leftDist");
        readLeftDist=backDist.getDistance(DistanceUnit.CM);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launchLeft.setDirection(DcMotor.Direction.REVERSE);

        intakeAdvance=hardwareMap.get(Servo.class,"intakeAdvance");

        wobbleRelease=hardwareMap.get(Servo.class,"wobbleRelease");

        //**** The IMU and associated variables ************
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

    /*
     ********************
     *   MAIN METHODS   *
     ********************
     */

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

    public void addData(Object data) {
        addData(data.toString());
    }
    public void addData(boolean data) {
        addData(String.valueOf(data));
    }
    public void addData(byte data) {
        addData(String.valueOf(data));
    }
    public void addData(char data) {
        addData(String.valueOf(data));
    }
    public void addData(short data) {
        addData(String.valueOf(data));
    }
    public void addData(int data) {
        addData(String.valueOf(data));
    }
    public void addData(long data) {
        addData(String.valueOf(data));
    }
    public void addData(float data) {
        addData(String.valueOf(data));
    }
    public void addData(double data) {
        addData(String.valueOf(data));
    }

    /**
     * Spin launch wheels based off of passed Velocity
     */
    public void velocitySpin(double MotorPower, double Velocity){
        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchLeft.setPower(MotorPower);
        launchRight.setPower(MotorPower);
        launchLeft.setVelocity(Velocity);
        launchRight.setVelocity(Velocity);
    }

    /**
     * Spin launch wheels (with an offset of 60 radians on launchLeft) based off of passed Velocity
     */
    public void velocitySpinSixty(double MotorPower, double Velocity){
        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launchLeft.setPower(MotorPower);
        launchRight.setPower(MotorPower);
        launchLeft.setVelocity(Velocity-60);
        launchRight.setVelocity(Velocity);
    }

    /**
     * Spin launch wheels and don't shoot until the encoder value = desired value
     */
    public void safeLaunch(double Velocity){
        double launchLeftVelocity = Velocity;
        double launchRightVelocity = Velocity;
        while(opModeIsActive()) {
            if (!(launchRight.getVelocity()==launchRightVelocity && launchLeft.getVelocity()==launchLeftVelocity)) {
                pause(0.05);
            } else {
                launchAdvanceFast();
                break;
            }
        }
    }

    /**
     * Spin launch wheels and don't shoot until the encoder value = desired value (both have an offset of 60 radians on launchLeft)
     */
    public void safeLaunchSixty(double Velocity){
        double launchLeftVelocity = Velocity-60;
        double launchRightVelocity = Velocity;
        while(opModeIsActive()) {
            if (!(launchRight.getVelocity()==launchRightVelocity && launchLeft.getVelocity()==launchLeftVelocity)) {
                pause(0.05);
            } else {
                launchAdvanceFast();
                break;
            }
        }
    }

    /**
     * Get angle readout from IMU
     */
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

    /**
     * Zero Bot to "initial angle" using IMU angle readout
     */
    public void zeroBotEncoder(double MotorPower){
        double newAngle = getAngle();
        telemetry.addData("zeroBot Initial ",initialAngle);
        telemetry.addData("New ",newAngle);
        telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
        telemetry.update();
        while (Math.abs(newAngle - initialAngle) > 3 && opModeIsActive()){ //was >5
            telemetry.addData("Zerobot Adj Initial ",initialAngle);
            telemetry.addData("New ",newAngle);
            telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
            telemetry.update();
            newAngle = getAngle();
            if (newAngle > initialAngle ){
                rightEncoder(Math.abs(newAngle - initialAngle),MotorPower);
            }else {
                leftEncoder(Math.abs(newAngle - initialAngle),MotorPower);
            }
        }
    }


    public void runOpMode() {
        mapObjects();
        //ACTUAL AUTONOMOUS PROGRAM GOES HERE
    }

    /**
     * Advance Launcher arm to launch ring
     */
    public void launchAdvanceFast(){ //pause were 0.25
        intakeAdvance.setPosition(0.2);
        pause(0.25);
        //Was 0.5, then 0.75
        intakeAdvance.setPosition(0.325);
        //Was 0.35, then 0.25, then 0.285
        pause(0.1);
        //Was 0,
    }

    /**
     * Drop wobble from mini servo latch
     */
    public void wobbleRelease() {
        wobbleRelease.setPosition(0.75);
        pause(0.1);
        //Was 0.2
    }

    /**
     * Lock wobble onto mini servo latch
     */
    public void wobbleLock(){
        wobbleRelease.setPosition(0.15);
    }

    /**
     * Set Wobble Arm to down position
     */
    public void wobbleGrabDown(double MotorPower){
        grabber.setMode(DcMotor.RunMode.RESET_ENCODERS);
        grabber.setTargetPosition((int)(-650)); //was 750
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(MotorPower);
        /*while (opModeIsActive() && (grabber.isBusy())){
            telemetry.addData("GB ",grabber.isBusy());
            telemetry.update();
        }*/
    }

    /**
     * Set Wobble Arm to up position
     */
    public void wobbleGrabUp(double MotorPower){
        grabber.setMode(DcMotor.RunMode.RESET_ENCODERS);
        grabber.setTargetPosition((int)(75));
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabber.setPower(MotorPower);
        /*while (opModeIsActive() && (grabber.isBusy())){
            telemetry.addData("GB ",grabber.isBusy());
            telemetry.update();
        }*/

    }

    /**
     * Set Wobble Arm to latch
     */
    public void wobbleLatch(){
        latch.setPosition(0.95);
        pause(0.5);
    }

    /**
     * Set Wobble Arm to release
     */
    public void wobbleLatchRelease(){
        latch.setPosition(0.4);
        //pause(0.5);
    }

    /**
     * Test function for sideways movement; probably shouldn't use in real code (but I do)
     */
    public void VennisFunctEnhanced(double pos, double StandardMotorPower, double OffMotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        frontRight.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        backLeft.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));
        backRight.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(StandardMotorPower);
        backLeft.setPower(StandardMotorPower*OffMotorPower);
        frontRight.setPower(StandardMotorPower*OffMotorPower);
        backRight.setPower(StandardMotorPower);
        while (opModeIsActive() && (frontLeft.isBusy() && backRight.isBusy())){
            telemetry.addData("FL ",frontLeft.isBusy());
            telemetry.addData("FR ",frontRight.isBusy());
            telemetry.addData("BL ", backLeft.isBusy());
            telemetry.addData("BR ",backRight.isBusy());
            telemetry.update();
        }
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Move robot forwards by cm
     */
    public void forwardEncoder(double pos, double MotorPower){ //1 pos = 25 cm
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

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
     * Fire up intake at passed power
     */
    public void intakeStart(double MotorPower){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(MotorPower);
    }

    /**
     * Stop intake
     */
    public void intakeEnd(){
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);
    }

    /**
     * Move robot backwards by cm
     */
    public void reverseEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

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
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

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
     * Update all distance sensors
     */
    public void updateDist(){
        readBackDist=backDist.getDistance(DistanceUnit.CM);
        readRightDist=rightDist.getDistance(DistanceUnit.CM);
        readFrontDist=frontDist.getDistance(DistanceUnit.CM);
        readLeftDist=leftDist.getDistance(DistanceUnit.CM);
        telemetry.addData("Back Dist, ",readBackDist);
        telemetry.addData("Right Dist, ",readRightDist);
        telemetry.addData("Left Dist, ",readLeftDist);
        telemetry.addData("Front Dist, ",readFrontDist);
        telemetry.update();
    }

    /**
     * Update right distance sensor
     */
    public void updateRightDist(){
        readRightDist=rightDist.getDistance(DistanceUnit.CM);
        //telemetry.addData("Right Dist, ",readRightDist);
        //telemetry.update();
    }

    /**
     * Update back distance sensor
     */
    public void updateBackDist(){
        readBackDist=backDist.getDistance(DistanceUnit.CM);
        //telemetry.addData("Back Dist, ",readBackDist);
        //telemetry.update();
    }

    /**
     * Update left distance sensor
     */
    public void updateLeftDist(){
        readLeftDist=leftDist.getDistance(DistanceUnit.CM);
        telemetry.addData("Left Dist, ",readLeftDist);
        telemetry.update();
    }

    /**
     * Strafe robot right by cm
     */
    public void strafeRightEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

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
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

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
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

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
     * Pause for seconds passed
     */
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs  && opModeIsActive() ){

        }
    }

}