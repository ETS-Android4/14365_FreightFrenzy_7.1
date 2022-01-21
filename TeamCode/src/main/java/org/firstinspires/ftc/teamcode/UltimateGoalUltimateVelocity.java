package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// TODO: Save and use as reference of CV usage

@Autonomous(name="-----FRANK-----", group="linearOpMode")
public class UltimateGoalUltimateVelocity extends AutonomousPrime2020 {
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
        for(int a = 0; a<2; a++){
            if (tfod != null) {
                pause(1);
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
                        telemetry.update();
                    }
                }
            }
        }
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()){
                if (noLabel == 0){ //NO RINGS
                    /*
                    HIT 3 PS
                     */
                    wobbleLock(); //Servo locks to wobble
                    intakeAdvance.setPosition(0.35); //Set intake advance arm to neutral position
                    velocitySpinSixty(1, 1040);
                    //Was at 980
                    forwardEncoder(160, 1); //Approach first PS
                    zeroBotEncoder(1); //Zero angle
                    //updateDist(); //Get updated distances
                    for(int i = 0; i<3; i++) {
                        updateRightDist();
                        double rightWallDist = readRightDist - 114; //Calculate how much & what direction to move in
                        //Was 109
                        strafeRightEncoder(rightWallDist, 0.5); //Move the distance above
                        pause(0.25);
                        zeroBotEncoder(0.5);
                        //pause(0.5);
                    }
                    zeroBotEncoder(1);
                    safeLaunchSixty(1040); //Hit first PS

                    updateLeftDist();

                    double leftWallDistFirst = readLeftDist-71;
                    //Was 79, then 74


                    strafeLeftEncoder(leftWallDistFirst, 0.5);
                    zeroBotEncoder(1); //Zero angle
                    pause(0.25); //Pause for launch arm to move
                    safeLaunchSixty(1040); //Hit second PS

                    velocitySpinSixty(1, 1020);

                    updateLeftDist();

                    double leftWallDistSecond = readLeftDist-51.5;
                    //Was 55, then 50


                    strafeLeftEncoder(leftWallDistSecond, 0.5);
                    zeroBotEncoder(1); //Zero angle
                    pause(0.25); //Pause for launch arm to move
                    safeLaunchSixty(1020); //Hit third PS
                    /*
                    DELIVER WOBBLE
                     */
                    forwardEncoder(20, 1); //Move forward to be in line with zone
                    pause(0.2); //Pause in between movements to avoid extreme slippage


                    strafeRightEncoder(122, 1); //Strafe right to zone
                    //Was 122, then 112, then 102


                    pause(0.2); //Pause in between movements to avoid extreme slippage
                    wobbleRelease(); //Release wobble
                    pause(0.75); //Pause for wobble to drop
                    /*
                    WOBBLE PICKUP
                     */
                    zeroBotEncoder(1); //Zero angle
                    wobbleGrabDown(1); //Move wobble arm down
                    wobbleLatchRelease();

                    //reverseEncoder(107, 0.25); //Move back towards second wobble

                    reverseEncoder(50, 1);
                    pause(0.1);
                    zeroBotEncoder(1);
                    strafeRightEncoder(20, 1);
                    //Was 20, then 25, then 15
                    pause(0.1);
                    zeroBotEncoder(1);
                    reverseEncoder(45, 0.5);
                    //Was 55


                    //Was 115, 100
                    pause(0.5);//Pause to make sure the wobble isn't still wobbling
                    wobbleLatch(); //Grab the wobble
                    /*
                    DELIVER SECOND WOBBLE
                     */
                    pause(1); //Pause for grabbing the wobble
                    zeroBotEncoder(1);
                    wobbleGrabUp(1); //Move the arm up


                    forwardEncoder(72, 1); //Move towards the zone
                    //Was 85, 75, then 82


                    rightEncoder(170, 0.65); //Turn to drop the wobble
                    //Was 180 and 1 MP
                    wobbleLatchRelease(); //Drop the wobble
                    forwardEncoder(20, 1); //Go forward to release the wobble
                    strafeRightEncoder(50, 1); //Strafe to park away from the wobble
                    reverseEncoder(55, 1); //Park
                    pause(100); //Pause so that the program does not run again
                }
                else if (labelName.equals("Single")){ //ONE RING
                    /*
                    HIT 3 PS
                     */
                    wobbleLock(); //Servo locks to wobble
                    intakeAdvance.setPosition(0.35); //Set intake advance arm to neutral position
                    velocitySpinSixty(1, 1040);
                    //Was at 980
                    forwardEncoder(160, 1); //Approach first PS
                    zeroBotEncoder(1); //Zero angle
                    //updateDist(); //Get updated distances
                    for(int i = 0; i<3; i++) {
                        updateRightDist();
                        double rightWallDist = readRightDist - 114; //Calculate how much & what direction to move in
                        //Was 109
                        strafeRightEncoder(rightWallDist, 0.5); //Move the distance above
                        pause(0.25);
                        zeroBotEncoder(0.5);
                        //pause(0.5);
                    }
                    zeroBotEncoder(1);
                    safeLaunchSixty(1040); //Hit first PS

                    updateLeftDist();

                    double leftWallDistFirst = readLeftDist-71;
                    //Was 79, then 74


                    strafeLeftEncoder(leftWallDistFirst, 0.5);
                    zeroBotEncoder(1); //Zero angle
                    pause(0.25); //Pause for launch arm to move
                    safeLaunchSixty(1040); //Hit second PS

                    velocitySpinSixty(1, 1020);

                    updateLeftDist();

                    double leftWallDistSecond = readLeftDist-51.5;
                    //Was 55, then 50


                    strafeLeftEncoder(leftWallDistSecond, 0.5);
                    zeroBotEncoder(1); //Zero angle
                    pause(0.25); //Pause for launch arm to move
                    safeLaunchSixty(1020); //Hit third PS
                    /*
                    DELIVER WOBBLE
                     */
                    velocitySpin(1, 1020);
                    wobbleGrabDown(1); //Move wobble arm down
                    wobbleLatchRelease();
                    forwardEncoder(75, 1); //Move forward to be in line with zone
                    pause(0.1); //Pause in between movements to avoid extreme slippage
                    strafeRightEncoder(40, 1); //Strafe right to zone
                    //Was 60
                    wobbleRelease(); //Release wobble
                    /*
                    SHOOT GROUND RING
                     */
                    zeroBotEncoder(1);
                    intakeStart(1); //Start intake wheels
                    //Was 0.45
                    reverseEncoder(50, 1); //Move away from wobble
                    pause(0.1); //Pause in between movements to avoid extreme slippage
                    /*updateDist(); //Update distance sensor values
                    double rightWallDist=readRightDist-72; //Calculate how much to move
                    rightWallDist=Math.abs(rightWallDist); //Absolute value the distance- since we know we always will be too far left
                    strafeRightEncoder(rightWallDist, 0.5); //Move the distance above*/

                    strafeRightEncoder(35, 1);
                    //Was 25

                    pause(0.1);
                    reverseEncoder(80, 1); //Move back to intake ring
                    zeroBotEncoder(1); //Zero angle
                    updateDist(); //Update distance sensor values
                    double rightWallDist=readRightDist-67; //Calculate how much to move
                    //Was 72
                    rightWallDist=Math.abs(rightWallDist); //Absolute value the distance- since we know we always will be too far left
                    strafeRightEncoder(rightWallDist, 0.5); //Move the distance above
                    zeroBotEncoder(1);
                    pause(0.1); //Pause to have time for intake
                    launchAdvanceFast(); //Shoot into high goal
                    /*
                    WOBBLE PICKUP
                     */
                    intakeEnd(); //Stop intake wheels
                    updateDist(); //Update distance sensor values
                    if(rightWallDist>500){ //Failsafe if walls are not detected
                        strafeRightEncoder(15, 1); //Strafe right slightly closer to in line with wobble
                    } else{ //If walls ARE detected...
                        rightWallDist=readRightDist-40; //Calculate how much to move
                        rightWallDist=Math.abs(rightWallDist); //Absolute value the distance- since we know we always will be too far left
                        strafeRightEncoder(rightWallDist, 0.5); //Move the distance above
                    }
                    pause(0.1); //Pause in between movements to avoid extreme slippage
                    reverseEncoder(27, 1); //Reverse closer to wobble
                    //Was 37
                    updateDist(); //Update distance sensor values
                    double count = 0; //Set counting variable for loop
                    while(readRightDist>=10 && readBackDist >=30){ //When the wobble is detected (With a failsafe to avoid being too close to the wall)
                        strafeRightEncoder(3, 1);  //Strafe right in small increments
                        updateDist(); //Update distance sensor values
                        count++; //Increase the count variable
                        if(count==5){ //If you've strafed 13 times (missed the wobble)...
                            break; //...then break out of the loop
                        }
                    }
                    //reverseEncoder(5, 0.25); //Go back to get grip on wobble
                    //strafeRightEncoder(1, 1); //Strafe to ensure hook latches
                    wobbleLatch(); //Grab the wobble
                    /*
                    DELIVER SECOND WOBBLE
                     */
                    wobbleGrabUp(1); //Move the arm up
                    forwardEncoder(145, 1); //Move towards the zone
                    pause(0.1); //Pause in between movements to avoid extreme slippage
                    rightEncoder(120, 0.65); //Turn to drop the wobble
                    //Was 1 MP
                    reverseEncoder(10, 1);
                    wobbleLatchRelease(); //Drop the wobble
                    pause(0.5);
                    forwardEncoder(20, 1); //Go forward to release the wobble & secure parking
                    //wobbleGrabUpLarge(1); //Move arm up to make sure hook isn't still caught
                    pause(100); //Pause so that the program does not run again
                }
                else if (labelName.equals("Quad")){ //FOUR RINGS
                    /*
                    DELIVER WOBBLE
                     */
                    wobbleLock();
                    intakeAdvance.setPosition(0.325);
                    //Was 0.35
                    velocitySpin(1, 1100);
                    //Was 1060, then 1080


                    forwardEncoder(130, 1);
                    VennisFunctEnhanced(220, 1, 0.35);
                    //forwardEncoder(300, 1);
                    //strafeRightEncoder(90, 1);

                    wobbleRelease();
                    zeroBotEncoder(1);
                    /*
                    SHOOT ONBOARD RINGS
                     */
                    reverseEncoder(140, 1);
                    //Was 150, 145
                    //pause(0.1);
                    zeroBotEncoder(1);
                    updateRightDist();
                    double moveDist = 83-readRightDist; //Was 73
                    strafeLeftEncoder(moveDist, 1);
                    //pause(0.1);
                    safeLaunch(1100);

                    //pause(0.1);

                    safeLaunch(1100);

                    //pause(0.1);

                    safeLaunch(1100);

                    //pause(0.1);

                    /*
                    SHOOT RING STACK
                     */
                    velocitySpin(1, 1060);
                    wobbleGrabDown(0.25);
                    intakeStart(1);
                    reverseEncoder(35, 0.5);
                    //Was 25, 30
                    zeroBotEncoder(1);
                    updateRightDist();
                    double moveShootDist = 58-readRightDist;
                    //Was 64, 74, 84
                    strafeLeftEncoder(moveShootDist, 1);
                    zeroBotEncoder(1);

                    //pause(0.25);

                    safeLaunch(1060);
                    velocitySpin(1, 1060);
                    //Was 1040
                    strafeLeftEncoder(10, 1);
                    pause(0.1);
                    reverseEncoder(9, 1);
                    pause(0.2);
                    reverseEncoder(9, 1);
                    pause(0.2);
                    reverseEncoder(9, 1);
                    pause(0.2);
                    zeroBotEncoder(1);
                    updateRightDist();
                    double moveShootDistSecond = 58-readRightDist;
                    //Was 64, 73, 84
                    strafeLeftEncoder(moveShootDistSecond, 1);

                    //zeroBotEncoder(1);

                    pause(0.1);
                    safeLaunch(1060);

                    //pause(0.25);

                    safeLaunch(1060);

                    //pause(0.1);

                    safeLaunch(1060);

                    //pause(0.1);

                    /*
                    PICKUP WOBBLE
                     */


                    //forwardEncoder(20, 1);


                    updateRightDist();
                    double moveWobbleDist = 33-readRightDist; //was 38, then 28
                    strafeLeftEncoder(moveWobbleDist, 0.5);
                    latch.setPosition(0.6);
                    pause(0.25);
                    reverseEncoder(25, 1); //Was 15
                    pause(0.1);
                    latch.setPosition(0.95);
                    pause(0.15); //was .25, .175
                    wobbleGrabUp(1);
                    rightEncoder(183, 1);
                    pause(0.1);
                    reverseEncoder(200, 1);
                    wobbleLatchRelease();
                    //pause(0.25);
                    forwardEncoder(70,1);


                    pause(100);


                }
            }
        }
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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
}