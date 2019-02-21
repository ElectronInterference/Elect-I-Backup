/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="DelayedCraterAuto", group="Linear Opmode")

public class HenryJr4WithDelay extends LinearOpMode {

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
                robot.wait(3500);
                Land();
                navigateToDepot();
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
        robot.driveRight(1,5);
        //turn away from the lander
        robot.turnDegrees(0);
    }
    
    //Go to our alliance's depot
    public void navigateToDepot() {
        //Drive away from the crater
        robot.driveForward(0.6,12);
        //Turn left towards the Depot(but not all the way or we would hit the lander's legs)
        robot.turnDegrees(90);
        //Drive between the crater and the lander
        robot.driveForward(0.6,47);
        //Turn towards the depot
        robot.turnDegrees(135);
        //Go into the depot
        robot.driveForward(0.6,36);
    }
    
    //Deploy our team marker
    public void DeployMarker() {
        //Turn so our team marker can only fall into the depot
        robot.turnDegrees(0);
        //release the marker
        robot.markerRelease();  
        //wait until the marker has definitely fallen
        robot.wait(1500);
    }
    
    //Drive into the crater
    public void NavigateToCrater(){
        
        //Turn towards the crater
        robot.turnDegrees(-45);
        //Drive into the crater
        robot.driveForward(0.8,102);

    }
    

}
    

      
