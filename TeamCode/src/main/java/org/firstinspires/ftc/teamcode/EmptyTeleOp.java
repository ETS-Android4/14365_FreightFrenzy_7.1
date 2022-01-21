package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp
public class EmptyTeleOp extends LinearOpMode {

    protected DcMotorEx frontLeft = null;
    protected DcMotorEx frontRight = null;
    protected DcMotorEx backLeft = null;
    protected DcMotorEx backRight = null;
    protected DcMotorEx intake = null;

    protected double MotorPower = 1.0;
    protected final double  COUNT_PER_ROTATION = 15.641025641;
    protected final double  COUNT_PER_DEGREE = 0.16;


    protected DistanceSensor BackLeft;
    protected double BackLeftDist = 0;
    protected DistanceSensor BackRight;
    protected double BackRightDist = 0;
    protected Servo chute = null;
    protected DcMotorEx linearSlide = null;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = "Aba+gBH/////AAABma/0sYDZakYVhtjb1kH5oBVmYfYsDZXTuEZL9m7EdnFKZN/0v/LvE/Yr0NsXiJo0mJmznKAA5MK6ojvgtV1e1ODodBaMYZpgE1YeoAXYpvvPGEsdGv3xbvgKhvwOvqDToPe3x5w6gsq7a4Ullp76kLxRIoZAqaRpOuf1/tiJJQ7gTBFf8MKgbCDosmMDj7FOZsclk7kos4L46bLkVBcD9E0l7tNR0H0ShiOvxBwq5eDvzvmzsjeGc1aPgx9Br5AbUwN1T+BOvqwvZH2pM2HDbybgcWQJKH1YvXH4O62ENsYhD9ubvktayK8hSuu2CpUd1FVU3YQp91UrCvaKPYMiMFu7zeQCnoc7UOpG1P/kdFKP";
    private static String labelName;
    private static int noLabel;
    private TFObjectDetector tfod;

    double DuckRightPos = -1;
    int DuckPosition = 0;

    int chutePos = 0;

        /*
     ********************
     *   SETUP VUFORIA  *
     ********************
     */
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    private boolean targetVisible       = false;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    double VufXPos = 0;
    double VufYPos = 0;
    double VufHeading = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        //CRServo intakeDrop = hardwareMap.crservo.get("intakeDrop");

        //DcMotor duckSpinny = hardwareMap.dcMotor.get("duckSpinny");

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double coefficient = 1;
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (-y - x - rx) / denominator;
            double backLeftPower = (-y + x - rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            spinIntake(0.5);

            if(gamepad1.right_trigger > 0.1) {
                coefficient = 0.5;
            }



            frontLeft.setPower(coefficient * frontLeftPower);
            frontRight.setPower(coefficient * frontRightPower);
            backLeft.setPower(coefficient * backLeftPower);
            backRight.setPower(coefficient * backRightPower);

            if(gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            if(gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            int checkInterval = 200;
            int prevPosition = intake.getCurrentPosition();
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            timer.reset();
                if (timer.time() > checkInterval) {
                    double speed2 = 0;
                    double speed = (double) (intake.getCurrentPosition() - prevPosition) / timer.time();
                    prevPosition = intake.getCurrentPosition();
                    timer.reset();
                    if(speed < speed2) {
                        chutePos += 10;
                    }
                    speed2 = speed;
                }

            if(gamepad1.b) {
                chutePos+=0.5;
            } else {
                chutePos = 0;
            }
            chute.setPosition(chutePos);

            if(gamepad1.a) {
                //Setting up variables of where we want to be
                int idealDist = 125;
                int idealXPos = 66; //Change This Value
                int idealYPos = 3; //Change This Value
                int idealHeading = 275; //Change This Valu4

                //Getting distance senor information
                updateBackDist();

                //Using distance sensor information to go the vuforia target

                linearSlide(40, 0.5);

                strafeLeftEncoder(20, 0.3);
                reverseEncoder(idealDist - BackLeftDist, 0.5);
                //vuforiaTrack();
                //Backing up until the vuforia target is in view
//                while(!targetVisible) {
//                    strafeLeftEncoder(10, 0.5);
//                    vuforiaTrack();
//                }

                //Going to the wobble to drop off element, using vuforia information
//                if(targetVisible) {
//                    vuforiaTrack();
//                    reverseEncoder(VufXPos - idealXPos, 0.25); //Was forward encoder, but I think it would be going backwards, not forwards, I haven't tested this though
//                    strafeLeftEncoder(VufYPos - idealYPos, 0.25);
//                }
                //Turning and going to the wobble
                leftEncoder(45, 0.5);
                forwardEncoder(80, 0.5);


                //Drop off the wobble
                chute(50);

            }

            /*if(gamepad1.dpad_up) {
                intakeDrop.setPower(1);
            }
            else if(gamepad1.dpad_down) {
                intakeDrop.setPower(-1);
            } else{
                intakeDrop.setPower(0);
            }*/

            /*if(gamepad1.left_trigger > 0.1) {
                duckSpinny.setPower(1);
            } else {
                duckSpinny.setPower(0);
            }*/
        }



    }



    private void initVuforia() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine; but it is now!
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");
        allTrackables.addAll(targets);

        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.9f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        targets.activate();


    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.85f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    void    identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    private void vuforiaTrack(){
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            VufXPos=translation.get(0) / mmPerInch;
            VufYPos=translation.get(2) / mmPerInch;

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle + 180);
            VufHeading = rotation.thirdAngle + 180;
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
    }

    private void tfodTrack(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                noLabel  = updatedRecognitions.size();
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    labelName = recognition.getLabel();
                    if(labelName.equals("Duck")){
                        DuckRightPos=recognition.getRight();
                    }

                }


            }
        }
    }

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

    public void updateBackDist(){
        BackLeftDist = BackLeft.getDistance(DistanceUnit.CM);
        BackRightDist = BackRight.getDistance(DistanceUnit.CM);
    }

    public void chute(double pos){
        //chute.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chute.setPosition(pos);
//Code from when it used to be a motor, can probably delete
//        double cmOffset = pos;
//
//        chute.setTargetPosition((int)(cmOffset));
//
//        chute.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        chute.setPower(MotorPower);
//
//        while (opModeIsActive() && (chute.isBusy())){
//            telemetry.addData("Chute: ",chute.isBusy());
//            telemetry.update();
//        }
    }

    public void linearSlide(double pos, double MotorPower) {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double cmOffset = pos;

        linearSlide.setTargetPosition((int)(cmOffset* COUNT_PER_ROTATION));

        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linearSlide.setPower(MotorPower);
    }

    public void spinIntake(double MotorPower){
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setPower(MotorPower);
    }

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
}
