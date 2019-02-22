//Copyright (c) 2017 FIRST. All rights reserved.\\
// Libaries \\
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
 *  This is our teleop program.
 */

@TeleOp(name="TeleOp", group="Teleop")

public class Elizabeth_I extends LinearOpMode 
{

    //access our hardware program
    MoldugaHardware robot   = new MoldugaHardware();
    
    @Override
    public void runOpMode() {
        
        //initialize the electronics
        robot.init(hardwareMap);
    
        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        // updates the status
        telemetry.update();
        //
        waitForStart();
        
        while(opModeIsActive() && !isStopRequested()) {
    
            //Sends power to wheels based on controller 1's joysticks
            robot.leftfrontDrive.setPower(gamepad1.left_stick_y);
            robot.leftrearDrive.setPower(gamepad1.left_stick_y);
            robot.rightfrontDrive.setPower(gamepad1.right_stick_y);
            robot.rightrearDrive.setPower(gamepad1.right_stick_y);
            
            if(gamepad1.left_stick_y == 0 && gamepad1.right_stick_y == 0) {
                //Sends power to wheels based on controller 1's joysticks
                robot.leftfrontDrive.setPower(gamepad2.left_stick_y / 3);
                robot.leftrearDrive.setPower(gamepad2.left_stick_y / 3);
                robot.rightfrontDrive.setPower(gamepad2.right_stick_y / 3);
                robot.rightrearDrive.setPower(gamepad2.right_stick_y / 3);
            }
    
            
            //strafe using the x button.
            if(gamepad1.x == true) {
                robot.leftfrontDrive.setPower(-0.5);
                robot.leftrearDrive.setPower(0.5);
                robot.rightfrontDrive.setPower(0.5);
                robot.rightrearDrive.setPower(-0.5);
            //Strafe using the b button.    
            } else if(gamepad1.b == true) {
                robot.leftfrontDrive.setPower(0.5);
                robot.leftrearDrive.setPower(-0.5);
                robot.rightfrontDrive.setPower(-0.5);
                robot.rightrearDrive.setPower(0.5);
            }
            
            
            //opens the Releaser mechanism
            if (gamepad2.left_trigger == 1){
                robot.mineralLift.setPower(1);
            }
            //Closes the releaser mechanism
            else if (gamepad2.right_trigger == 1){
                robot.mineralLift.setPower(-1);
            } else {
                robot.mineralLift.setPower(0);
            }
             //Lifts the aquisition arm
            if (gamepad2.dpad_left == true) {
                robot.acquisition.setPower(0.5);
            }
            // Lowers the aquisition arm
            else if (gamepad2.dpad_right == true) {
                robot.acquisition.setPower(-1);
            } else {
                robot.acquisition.setPower(0);
            }
    
            //Moves the lift upwards
            if (gamepad2.right_bumper == true && !robot.touch.isPressed()) {
                raiseLift();
            }
            //If The left bumper is pressed move the lift downwards
            else if (gamepad2.left_bumper == true  ) {
                lowerLift();
                
            } 
            //Stops the lift
             else {
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
