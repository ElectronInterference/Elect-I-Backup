//Copyright (c) 2017 FIRST. All rights reserved.

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MoldugaHardware;



/* 
 *  This is our teleop program.
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

public class Controller_test extends OpMode
{

    //access our hardware program
    MoldugaHardware robot   = new MoldugaHardware();
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        
        robot.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Send power to wheels based on controller 1's joysticks
        robot.leftfrontDrive.setPower(gamepad1.left_stick_y);
        robot.leftrearDrive.setPower(gamepad1.left_stick_y);
        robot.rightfrontDrive.setPower(gamepad1.right_stick_y);
        robot.rightrearDrive.setPower(gamepad1.right_stick_y);

        
        //strafe if the x or b buttons are pressed
        if(gamepad1.x == true) {
            robot.leftfrontDrive.setPower(-0.5);
            robot.leftrearDrive.setPower(0.5);
            robot.rightfrontDrive.setPower(0.5);
            robot.rightrearDrive.setPower(-0.5);
            
        } else if(gamepad1.b == true) {
            robot.leftfrontDrive.setPower(0.5);
            robot.leftrearDrive.setPower(-0.5);
            robot.rightfrontDrive.setPower(-0.5);
            robot.rightrearDrive.setPower(0.5);
        }
        
        
        //opens the Releaser mechanism
        if (gamepad2.left_trigger == 1){
            
        //sets the servo to 180 degrees
            robot.mineralLift.setPower (1);

        }
        //Closes the releaser mechanism
        else if (gamepad2.right_trigger == 0){
            //
            robot.mineralLift.setPower  (0);
            
        }

        if (gamepad1.left_trigger==1) {
             // Pivots the robot right
             robot.turnDegrees(90);
        }
        else if (gamepad1.right_trigger== 0) {
            // pivots the robot left
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
            stopLift();
        }
            
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    // Raise the lift
    public void raiseLift() { 
        robot.lift.setPower (1);
    }
    //Lower the lift
    public void lowerLift() {
        robot.lift.setPower (-1);
    }
    // Turn off the lift 
    public void stopLift() {
        robot.lift.setPower (0);
    }
}
