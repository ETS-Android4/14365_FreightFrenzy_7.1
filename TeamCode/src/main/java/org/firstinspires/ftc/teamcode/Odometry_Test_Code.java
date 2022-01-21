package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Map;

@Autonomous(name = "Odometry_Test_Code", group = "linearOpMode")
public class Odometry_Test_Code extends AutonomousPrime2021 {
    // todo: write your code here

    int left_encoder_pos = 0; //THIS NEEDS TO BE UPDATED BY SENSORS
    int prev_left_encoder_pos = 20;

    int right_encoder_pos = 20; //THIS NEEDS TO BE UPDATED BY SENSORS
    int prev_right_encoder_pos = 0;

    int center_encoder_pos = 20; //THIS NEEDS TO BE UPDATED BY SENSORS
    int prev_center_encoder_pos = 0;

    //Distance between right x sensor left x sensor
    int trackwidth = 15;
    int forward_offset = 0;

    int delta_x = 0;
    int heading = 0;
    int delta_y = 0;
    int x_pos = 0;
    int y_pos = 0;
    //Assume delta
    int delta_left_encoder_pos = 0;
    int delta_right_encoder_pos = 0;
    int delta_center_encoder_pos = 0;
    //new heading
    int phi = 0;
    int delta_middle_pos = 0;
    int delta_perp_pos = 0;

    //"Why is it called scope? Why isn't it just called 'variables go bye'?" -Mason Moyle
    @Override
    public void runOpMode() {
        mapObjects();
        waitForStart();
        //https://gm0.org/en/latest/docs/software/odometry.html

        calculatePos();

    }

    public void calculatePos() {
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

        telemetry.addData("X Position", x_pos);
        telemetry.addData("Y Position", y_pos);
        telemetry.addData("Left Encoder Position", left_encoder_pos);
        telemetry.addData("Right Encoder Position", right_encoder_pos);
        telemetry.addData("Heading", heading);
        telemetry.update();
    }

    public void goTo(double x, double y) {
        double go_x = x - x_pos;
        double go_y = y - y_pos;
        double c = Math.pow(go_x, 2) + Math.pow(go_y, 2);
        c = Math.sqrt(c);
        double turn = Math.tan(go_x / go_y);
        rightEncoder(turn, 1);
        forwardEncoder(c, 1);
    }


}