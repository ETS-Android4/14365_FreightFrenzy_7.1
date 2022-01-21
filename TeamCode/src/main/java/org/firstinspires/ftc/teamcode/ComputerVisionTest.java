package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import java.util.List;

// TODO: Get working with custom .TFLITE models (LONG TERM)

@Autonomous(name="ComputerVisionTest", group="linearOpMode")
public class ComputerVisionTest extends AutonomousPrime2021 {
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
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    @Override
    public void runOpMode(){
        initVuforia();
        initTfod();
        tfod.activate();


        mapObjects();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //pause(2);
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

                        }
                        if (noLabel == 0){
                            //forwardEncoder(80, 0.5);
                            //pause(100);
                        }
                        else if (labelName.equals("Single")){
                            //forwardEncoder(50, 0.5);
                        }
                        else if (labelName.equals("Quad")){
                            //forwardEncoder(120, 0.5);
                        }
                        //String label = recognition.getLabel();
                        telemetry.update();
                        /* if(label == "single"){

                        }
                        else if (label == "quad"){

                        } else{

                        }*/

                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }



        /*
        OPTION ONE
        CV Scan for ring num (log in var)
        Move to line
        Shoot 3x at 80% power
        Go back to wobble and grab
        Drop off goal in correct zone based off of var
        Park
         */
        /*
        OPTION TWO
        CV Scan for ring num (log in var)
        Grab wobble
        Move to line
        Shoot 3x at 80% power
        Drop off goal in correct zone based off of var
        Park
         */
        /*
        OPTION ONE PROS
        -Potentially more consistent
        -Easier points in beginning (if fails later, have more points still)
        OPTION ONE CONS
        -More time to go back for wobble and drop it off (TIME CONCERN)
        OPTION TWO PROS
        -Faster and more fluid
        -Still gets same amount of points as option one but with chance to pick up and shoot "floor rings"
        OPTION TWO CONS
        -Points weighted towards end; less points if it fails at any moment
        -Wobble may cause drag to interfere
         */

}