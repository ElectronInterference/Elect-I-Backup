//Copyright (c) 2017 FIRST. All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MoldugaHardware;



/* 
 *  This is our robot test program.
 */
 
 /* 
                                ~~Controller map~~
                      --Driver Controls ((GamePad 1))
                        * Strafe left:  X
                        * Strafe Right: B
                        * Left Wheels: Left_Stick
                        * Right Wheels: Right_Stick
*                      -- Lift Controls ((GamePad 2))
                        * Asend the lift Right_Bumper
                        * Desend the lift Left_Bumper
                        * (Prototype Lift Servo) Opens the Servo, Closes the servo Right_trigger 
 */
@TeleOp(name="TeleOp Test", group="Teleop")

public class Controller_test extends LinearOpMode 
{

    //Accesses our hardware program
    MoldugaHardware robot   = new MoldugaHardware();
    
    @Override
    public void runOpMode() {
        //Initalizes the electronics
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        //updates the status
        telemetry.update();

        waitForStart();
        
        while(opModeIsActive()) {
            // Send power to wheels based on controller 1's joysticks
            robot.leftfrontDrive.setPower(gamepad1.left_stick_y);
            robot.leftrearDrive.setPower(gamepad1.left_stick_y);
            robot.rightfrontDrive.setPower(gamepad1.right_stick_y);
            robot.rightrearDrive.setPower(gamepad1.right_stick_y);
    
            
            //Strafes when the X button is pressed
            if(gamepad1.x == true) {
                robot.leftfrontDrive.setPower(-0.5);
                robot.leftrearDrive.setPower(0.5);
                robot.rightfrontDrive.setPower(0.5);
                robot.rightrearDrive.setPower(-0.5);
            //Strafes when the B button is pressed
            } else if(gamepad1.b == true) {
                robot.leftfrontDrive.setPower(0.5);
                robot.leftrearDrive.setPower(-0.5);
                robot.rightfrontDrive.setPower(-0.5);
                robot.rightrearDrive.setPower(0.5);
            }
            
            
            //opens the Releaser mechanism
            if (gamepad2.left_trigger == 1){
                
            //sets the servo to 180 degrees
                robot.mineralLift.setPower (2500);
    
            }
            //Closes the releaser mechanism
            else if (gamepad2.right_trigger == 1){
                //Lowers the mineral lift
                robot.mineralLift.setPower  (-2500);
            } else {
                //If the button isn't pressed, then the servo does nothing.
            robot.mineralLift.setPower (0);
            }if (gamepad2.y ==true) {
                //when the Y button is pressed, then it moves 85 degrees.
                robot.Releaser.setPower(2500);
            } else if(gamepad2.a == true) {
                //when the Y button is pressed, then it moves -40 degrees.
                robot.Releaser.setPower(-500);
            } else {
                //if the button isn't pressed,then the servo will power down
                robot.Releaser.setPower(0);
            }
        
    
    
    
            if (gamepad1.left_trigger == 1) {
                 //Pivots the robot right
                 robot.turnDegrees(90);
            }
             else if (gamepad1.right_trigger== 1) {
                //Pivots the robot left
                 robot.turnDegrees(-90);
            }
            
            
            // moves the lift upwards
            if (gamepad2.right_bumper== true ) {
                raiseLift();
            }
            // If The left bumper is pressed the lift would move downwards
            else if (gamepad2.left_bumper ==true ) {
                lowerLift();
                
            } 
                
             else {
                 //Stops the lift when the button isn't pressed
                stopLift();
            }
            
        }
    }
    // Raise the lift
    public void raiseLift() { 
        robot.lift.setPower (.80);
    }
    //Lower the lift
    public void lowerLift() {
        robot.lift.setPower (-.80);
    }
    // Turn off the lift 
    public void stopLift() {
        robot.lift.setPower (0);
    }
}
