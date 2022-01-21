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

// TODO: Prepare whatever needs to be done with tele-op as it comes up

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

@TeleOp(name = "-----GEORGE-----", group = "Current")

public class LinearTeleOpMain extends LinearOpMode {
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

    protected double launchRightVelocity = 1020;
    protected double launchLeftVelocity = launchRightVelocity-20;

    double grabberPos = 0;

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