package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter; 
public class Runner {

	@RequiresApi(api = Build.VERSION_CODES.O)
	public static void main(String[] args) throws InterruptedException {
		// TODO Auto-generated method stub
		DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss");
		LocalDateTime now = LocalDateTime.now();
		System.out.println("Initial time = " + now);
		MyThread myThread = new MyThread();
		myThread.start();
		Thread.sleep(2000);
		System.out.println("read of time 1-" + myThread.curTime);
		Thread.sleep(6000);
		System.out.println("read of time 2-" + myThread.curTime);
		Thread.sleep(2000);
		System.out.println("read of time 3-" + myThread.curTime);
		//This method shoots the thread with an assault rifle
		myThread.stop();
		System.exit(0);

	}

}
