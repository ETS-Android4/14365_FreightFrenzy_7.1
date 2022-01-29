package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous(name="AutonomousProgramBase", group="linearOpMode")
public class AutonomousProgramBase extends AutonomousPrime2021 {

    private static final String VUFORIA_KEY =
            "Aba+gBH/////AAABma/0sYDZakYVhtjb1kH5oBVmYfYsDZXTuEZL9m7EdnFKZN/0v/LvE/Yr0NsXiJo0mJmznKAA5MK6ojvgtV1e1ODodBaMYZpgE1YeoAXYpvvPGEsdGv3xbvgKhvwOvqDToPe3x5w6gsq7a4Ullp76kLxRIoZAqaRpOuf1/tiJJQ7gTBFf8MKgbCDosmMDj7FOZsclk7kos4L46bLkVBcD9E0l7tNR0H0ShiOvxBwq5eDvzvmzsjeGc1aPgx9Br5AbUwN1T+BOvqwvZH2pM2HDbybgcWQJKH1YvXH4O62ENsYhD9ubvktayK8hSuu2CpUd1FVU3YQp91UrCvaKPYMiMFu7zeQCnoc7UOpG1P/kdFKP";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float cmPerInch        = 2.54f; //Was 25.4f for mm, changing to cm now
    private static final float mmTargetHeight   = 6 * cmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * cmPerInch;
    private static final float halfTile         = 12 * cmPerInch;
    private static final float oneAndHalfTile   = 36 * cmPerInch;

    // Class Members
    private static OpenGLMatrix lastLocation   = null;
    private VuforiaLocalizer vuforia    = null;
    private VuforiaTrackables targets   = null ;
    private WebcamName webcamName       = null;

    public static boolean targetVisible       = false;

    public static VectorF translationG;
    public static Orientation rotationG;

    private static List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public static double VufXPos;
    public static double VufYPos;
    public static double VufHeading;
    public static double VufVisible;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static String labelName;
    private static int noLabel;
    private TFObjectDetector tfod;


    /*
     *********************
     *   SETUP ODOMETRY  *
     *********************
     */
    public static int left_encoder_pos = 0; //THIS NEEDS TO BE UPDATED BY SENSORS
    public static int prev_left_encoder_pos = 20;

    public static int right_encoder_pos = 20; //THIS NEEDS TO BE UPDATED BY SENSORS
    public static int prev_right_encoder_pos = 0;

    public static int center_encoder_pos = 20; //THIS NEEDS TO BE UPDATED BY SENSORS
    public static int prev_center_encoder_pos = 0;

    //Distance between right x sensor left x sensor
    public static int trackwidth = 19; //in cm
    public static int forward_offset = 0;

    public static int delta_x = 0;



    public static double heading = 0; //MIGHT NEED TO CHANGE BACK TO INT IN TESTING IDK



    public static int delta_y = 0;



    public static double x_pos = 0; //MIGHT NEED TO CHANGE BACK TO INT IN TESTING IDK
    public static double y_pos = 0; //MIGHT NEED TO CHANGE BACK TO INT IN TESTING IDK



    //Assume delta
    public static int delta_left_encoder_pos = 0;
    public static int delta_right_encoder_pos = 0;
    public static int delta_center_encoder_pos = 0;
    //new heading
    public static int phi = 0;
    public static int delta_middle_pos = 0;
    public static int delta_perp_pos = 0;

    public static double vuforia_x = 0;
    public static double vuforia_y = 0;

    @Override
    public void runOpMode() {
        mapObjects();
        vuforiaInit();
        initTfod(0.6);
        tfod.activate();
        telemetry.addData("Robot: ", "Ready!");
        telemetry.update();
        waitForStart();

        Thread t1 = new Thread(new SensorThread());
        t1.start();

        Thread t2 = new Thread(new ArmController());
        t2.start();



        /*while(!isStopRequested()){
            telemetry.addData("Odo X Pos: ", SensorData.getOdoXPos());
            telemetry.addData("Odo Y Pos: ", SensorData.getOdoYPos());
            telemetry.addData("Odo Heading: ", SensorData.getOdoHeading());

        }*/


        strafeRightEncoder(22, 0.5);
        rightEncoder(90,0.5);

        dArm(6,0.4);

        double idealWobblePos=10.5; //was 16.4, 12.5, 3.5, 7.5,
        pause(1);
        double rightDist = SensorData.getRightDist();

        strafeRightEncoder(rightDist-idealWobblePos,0.25);

        duckSpin(1,4);

        forwardEncoder(65,0.5); //Was 45, 35, 38, 48, 53, 58, 68, 65

        zeroBotEncoderOffset(80, 0.5); //was 90, 120, 70, 80, 90, 95

        dArm(0,0.45);

        pause(1);

        strafeRightEncoder(70,0.5);


    }


    public void vuforiaInit(){
        // Connect to the camera we are to use.  This name must match what is set up in Robot Configuration
        webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        // We also indicate which camera we wish to use.
        parameters.cameraName = webcamName;

        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = this.vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targets);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage",       -halfField,  oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall",  halfTile,   halfField,      mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage",        -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall",   halfTile,  -halfField,      mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */

        final float CAMERA_FORWARD_DISPLACEMENT  = 0.0f * cmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.9f * cmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0.0f * cmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        /**  Let all the trackable listeners know where the camera is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, cameraLocationOnRobot);
        }

        /*
         * WARNING:
         * In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
         * This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
         * CONSEQUENTLY do not put any driving commands in this loop.
         * To restore the normal opmode structure, just un-comment the following line:
         */

        // waitForStart();

        /* Note: To use the remote camera preview:
         * AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
         * Tap the preview window to receive a fresh image.
         * It is not permitted to transition to RUN while the camera preview window is active.
         * Either press STOP to exit the OpMode, or use the "options menu" again, and select "Camera Stream" to close the preview window.
         */

        targets.activate();
    }


    public static void vuforiaTrack(){
        //while (!isStopRequested()) {

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
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
            translationG = translation;
            //telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
            System.out.println(translation.get(0) / cmPerInch + translation.get(1) / cmPerInch + translation.get(2) / cmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            rotationG = rotation;
            //telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        }
        else {
            //telemetry.addData("Visible Target", "none");
        }
        //telemetry.update();
    }

    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    private void initTfod(double conf) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = (float) conf;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void TfodTrack(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                noLabel = updatedRecognitions.size();
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    labelName = recognition.getLabel();

                }
                //telemetry.update();
            }
        }
    }

    public static void calculatePos(int leftEncoder, int rightEncoder, int centerEncoder) {

        delta_left_encoder_pos=leftEncoder;
        delta_right_encoder_pos=rightEncoder;
        delta_center_encoder_pos=centerEncoder;

        left_encoder_pos = delta_left_encoder_pos + prev_left_encoder_pos;
        right_encoder_pos = delta_right_encoder_pos + prev_right_encoder_pos;
        center_encoder_pos = delta_center_encoder_pos + prev_center_encoder_pos;
        phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth;

        //Assuming this is right
        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        delta_perp_pos = delta_center_encoder_pos - forward_offset * phi;
        delta_x = (int) (delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading));
        delta_y = (int) (delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading));


        //Updating robot position on the field
        x_pos += delta_x;
        y_pos += delta_y;
        heading += phi;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*telemetry.addData("X Position", x_pos);
        telemetry.addData("Y Position", y_pos);
        telemetry.addData("Left Encoder Position", left_encoder_pos);
        telemetry.addData("Right Encoder Position", right_encoder_pos);
        telemetry.addData("Heading", heading);*/
        //telemetry.update();
    }

    public static void calculatePosVuf(int leftEncoder, int rightEncoder, int centerEncoder) {

        delta_left_encoder_pos=leftEncoder;
        delta_right_encoder_pos=rightEncoder;
        delta_center_encoder_pos=centerEncoder;

        left_encoder_pos = delta_left_encoder_pos + prev_left_encoder_pos;
        right_encoder_pos = delta_right_encoder_pos + prev_right_encoder_pos;
        center_encoder_pos = delta_center_encoder_pos + prev_center_encoder_pos;

        phi = (delta_left_encoder_pos - delta_right_encoder_pos) / trackwidth;

        //Assuming this is right
        delta_middle_pos = (delta_left_encoder_pos + delta_right_encoder_pos) / 2;
        delta_perp_pos = delta_center_encoder_pos - forward_offset * phi;
        delta_x = (int) (delta_middle_pos * Math.cos(heading) - delta_perp_pos * Math.sin(heading));
        delta_y = (int) (delta_middle_pos * Math.sin(heading) + delta_perp_pos * Math.cos(heading));


        //Updating robot position on the field
        x_pos = vuforia_x;
        y_pos = vuforia_y;
        heading = angle;

        prev_left_encoder_pos = left_encoder_pos;
        prev_right_encoder_pos = right_encoder_pos;
        prev_center_encoder_pos = center_encoder_pos;

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*telemetry.addData("X Position", x_pos);
        telemetry.addData("Y Position", y_pos);
        telemetry.addData("Left Encoder Position", left_encoder_pos);
        telemetry.addData("Right Encoder Position", right_encoder_pos);
        telemetry.addData("Heading", heading);*/
        //telemetry.update();
    }


}
