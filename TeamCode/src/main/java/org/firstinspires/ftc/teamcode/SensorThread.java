package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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
import static org.firstinspires.ftc.teamcode.ComputerVisionBase.tfod;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


class SensorThread extends AutonomousPrime2021 implements Runnable {

    /*
    Re-Mapping Everything Because Threads Hate Us :)
    */

    /*
     ***********************
     *   SETUP TENSORFLOW  *
     ***********************
     */
    String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    String VUFORIA_KEY = "Aba+gBH/////AAABma/0sYDZakYVhtjb1kH5oBVmYfYsDZXTuEZL9m7EdnFKZN/0v/LvE/Yr0NsXiJo0mJmznKAA5MK6ojvgtV1e1ODodBaMYZpgE1YeoAXYpvvPGEsdGv3xbvgKhvwOvqDToPe3x5w6gsq7a4Ullp76kLxRIoZAqaRpOuf1/tiJJQ7gTBFf8MKgbCDosmMDj7FOZsclk7kos4L46bLkVBcD9E0l7tNR0H0ShiOvxBwq5eDvzvmzsjeGc1aPgx9Br5AbUwN1T+BOvqwvZH2pM2HDbybgcWQJKH1YvXH4O62ENsYhD9ubvktayK8hSuu2CpUd1FVU3YQp91UrCvaKPYMiMFu7zeQCnoc7UOpG1P/kdFKP";
    String labelName;
    int noLabel;
    TFObjectDetector tfod;

    double DuckRightPos = -1;
    int DuckPosition = 0;

    /*
     ********************
     *   SETUP VUFORIA  *
     ********************
     */
    float mmPerInch        = 25.4f;
    float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    float halfField        = 72 * mmPerInch;
    float halfTile         = 12 * mmPerInch;
    float oneAndHalfTile   = 36 * mmPerInch;

    // Class Members
    OpenGLMatrix lastLocation   = null;
    VuforiaLocalizer vuforia    = null;
    VuforiaTrackables targets   = null ;
    WebcamName webcamName       = null;

    boolean targetVisible       = false;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    double VufXPos = 0;
    double VufYPos = 0;
    double VufHeading = 0;

    @Override
    public void run() {

        webcamName = VuforiaTensorflowIMUCombined.webcamName;

        //webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        initVuforia();

        mapObjects();
        waitForStart();


        
        DistanceSensor GroundFront = AutonomousPrime2021.GroundFront;
        DistanceSensor GroundBack = AutonomousPrime2021.GroundBack;

        DistanceSensor Left = AutonomousPrime2021.Left;
        DistanceSensor Right = AutonomousPrime2021.Right;

        DistanceSensor FrontLeft = AutonomousPrime2021.FrontLeft;
        DistanceSensor FrontRight = AutonomousPrime2021.FrontRight;

        DistanceSensor BackLeft = AutonomousPrime2021.BackLeft;
        DistanceSensor BackRight = AutonomousPrime2021.BackRight;

        BNO055IMU imu = AutonomousPrime2021.imu;

        SensorData ThreadUpload = new SensorData();

        while (!Thread.currentThread().isInterrupted()) {
            ThreadUpload.setBackLeftDist(BackLeft.getDistance(DistanceUnit.CM));
            ThreadUpload.setBackRightDist(BackRight.getDistance(DistanceUnit.CM));

            ThreadUpload.setFrontLeftDist(FrontLeft.getDistance(DistanceUnit.CM));
            ThreadUpload.setFrontRightDist(FrontRight.getDistance(DistanceUnit.CM));

            ThreadUpload.setLeftDist(Left.getDistance(DistanceUnit.CM));
            ThreadUpload.setRightDist(Right.getDistance(DistanceUnit.CM));

            ThreadUpload.setGroundFrontDist(GroundFront.getDistance(DistanceUnit.CM));
            ThreadUpload.setGroundBackDist(GroundBack.getDistance(DistanceUnit.CM));



            vuforiaTrack();

            if(targetVisible){
                ThreadUpload.setVufHeading(VufHeading);
                ThreadUpload.setVufYPos(VufYPos);
                ThreadUpload.setVufXPos(VufXPos);
                setAngle(VufHeading);
            }

            ThreadUpload.setIMUAngle(getAngle());



            AutonomousPrime2021.SensorData=ThreadUpload;



        }

    }



    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
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
        //telemetry.addData("Global Angle:", globalAngle);
        //telemetry.update();
        return globalAngle;
    }
    public void setAngle(double angle) {
        globalAngle = angle;
        //telemetry.addData("New Global Angle:", globalAngle);
        //telemetry.update();
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


}
