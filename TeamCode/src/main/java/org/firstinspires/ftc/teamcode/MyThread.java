package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.time.format.DateTimeFormatter;
import java.time.LocalDateTime;

@Autonomous(name="MyThread", group="LinearOpMode")
public class MyThread extends AutonomousPrime2021  {
	@RequiresApi(api = Build.VERSION_CODES.O)
	@Override
	public void runOpMode() {
		System.out.println("MyThread running");
		//this.curTime =  "F";
		int counter = 0;
		while (this.stopRunning) {
			//System.out.println("MyThread running2");
			DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
			LocalDateTime now = LocalDateTime.now();
			counter++;
			this.curTime = now.toString();
			//this.curTime+= "F";
			//System.out.println(now.toString());
		}
	}

	public  boolean stopRunning = true;
	public  String curTime = "D";
}
