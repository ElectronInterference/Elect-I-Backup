/* Copyright (c) 2017 FIRST. All rights reserved. */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.Timer;
import java.util.TimerTask;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MoldugaHardware;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;



/**
 * Our autonomous program for the starting locations close to the depots
 */
@Autonomous(name="DepotAuto", group="Linear Opmode")

public class HenryJr3 extends LinearOpMode {

    //access our hardware program
    MoldugaHardware robot   = new MoldugaHardware();

    //create a timer to send whether the program is stopped to the hardware class
    Timer timer = new Timer("Check For Stop");
    TimerTask checkForStop = new TimerTask() {
        @Override
        public void run() {
            //If needed, change a variable in MoldugaHardware that stores whether stop is requested
            robot.isStopRequested = isStopRequested();
            
            //Stop this timer if the program is stopped.
            if(robot.isStopRequested) {
                timer.cancel();
            }
        }
    };
    
    //Wait 50 millis between each loop of the timer and don't delay anything besides that.
    long timerPeriod = 50L;
    long delay = 0L;
    
    
    
    @Override
    public void runOpMode() {
        
        //Schedule a time to tell MoldugaHardware whether the program is stopped
        timer.schedule(checkForStop, delay, timerPeriod );
        
        //Create a boolean so that our program is only run once
        boolean done = false;
            
        //use our hardwaremap program
        robot.init(hardwareMap);
        
        //tell the drivers we have initialized
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        //wait for the program to start
        waitForStart();
        
        while(opModeIsActive()) {
            
            //If we aren't done, do all of the functions below in order
            if(!done) {
                Land();
                KnockOffCheeseblock();
                //navigateToDepot();
                DeployMarker();
                NavigateToCrater();
                
                //stop the timer and tell the loop we are done
                timer.cancel();
                done = true;
            }
        }
    }
    
    //land and reorient ourselves
     public void Land(){
         //Move the lift up
        robot.liftUp();
        //strafe off of the latch
        robot.driveRight(1,6);
        //turn away from the lander
        robot.turnDegrees(0);
        
        //drive slightly away from the lander
        robot.driveForward(0.5, 3);
    }
    public void KnockOffCheeseblock(){
        
        //turn to the right of the cheeseblock
        robot.turnDegrees(35);
        
        int location = -10;
        int minX = 250;
        int maxX = 350;
        
        //all of our angles
        float ab = 27;
        float ad = 30;
        float bc = -1;
        float ac = -1;
        float cd = -1;
        float abc = -1;
        float bca = -1;
        float acd = -1;
        float bcd = -1;
        float dce = -1;
        boolean reversed = false;
        
        
        robot.leftfrontDrive.setPower(-0.18);
        robot.leftrearDrive.setPower(-0.18);
        robot.rightfrontDrive.setPower(0.18);
        robot.rightrearDrive.setPower(0.18);
        while(!(location > minX && location < maxX) && opModeIsActive() && robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -30) {
            //robot.turnDegrees(turnInc);
            location = robot.findCheeseblockX();
            //turnInc = turnInc+5;
            
            if (location >= 0) {
                telemetry.addData ("location", location);
                telemetry.update(); 
            }
            
           robot.wait(10);
        }
        robot.stopDrive();
        //telemetry.addData ("status", "Found");
        telemetry.update(); 
        
        
        /*our math to calculate where we need to go:
        * it is assumed that the gold mineral is closest to the line directly at the center mineral
        */
        abc = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // the angle made by the depot, the robot, and the gold mineral
        if(abc < 0) {
            reversed = true;
            abc = abc * -1;
        }
        bc = (float)(27 / Math.cos(Math.toRadians(abc))); // the distance between the robot and the gold mineral
        ac = (float)(Math.sqrt((bc*bc) - (ab*ab))); //
        cd = (float)(Math.sqrt((ac*ac) + (ad*ad)));
        bca = 180 - (90 + abc);
        acd = (float)(Math.toDegrees(Math.atan(ad / ac)));
        bcd = bca + acd;
        dce = 180 - bcd - abc;
        if(reversed == true) {
            abc = abc * -1;
            dce = dce * -1;
        }
        
        telemetry.addData("abc", abc);
        telemetry.addData("bc", bc);
        telemetry.addData("ac", ac);
        telemetry.addData("cd", cd);
        telemetry.addData("bca", bca);
        telemetry.addData("acd", acd);
        telemetry.addData("bcd", bcd);
        telemetry.addData("dce", dce);
        telemetry.update();
        
        robot.driveForward(0.5,(int)(bc));
        robot.turnDegrees((int)(-dce));
        robot.driveForward(0.5,(int)(cd));
    }
    
    public void DeployMarker() {
        //Turn so our team marker can only fall into the depot
        robot.turnDegrees(-60);
        //release the marker
        robot.markerRelease();  
        //wait until the marker has definitely fallen
        robot.wait(350);
        //strafe away from the team marker
        robot.driveLeft(0.7, 4);
    }
    
    public void NavigateToCrater() {
        robot.driveBackward(0.7, 10);
        robot.turnDegrees(-51);
        robot.driveBackward(1, 115);
    }
}
    

      
