/* Copyright (c) 2017 FIRST. All rights reserved.
*/
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MoldugaHardware;


/* When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Drivemode2", group="Teleop")

public class Drivemode2 extends OpMode
{

    //acces our hardware program
    MoldugaHardware robot   = new MoldugaHardware();
    
    //create a timer
    private ElapsedTime runtime = new ElapsedTime();
    
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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftfrontPower;
        double rightfrontPower;
        double leftrearPower;
        double rightrearPower;
        double TopLeft;
        double TopRight;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.


        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double driveLeft  = gamepad1.left_stick_y;
        double driveRight   =  gamepad1.right_stick_y;
      
         
        leftfrontPower    = Range.clip(driveLeft, -1.0, 1.0) ;
        leftrearPower    = Range.clip(driveLeft, -1.0, 1.0) ;
        rightfrontPower   = Range.clip(driveRight, -1.0, 1.0) ;
        rightrearPower   = Range.clip(driveRight, -1.0, 1.0) ;
        

        // Send calculated power to wheels
        robot.leftfrontDrive.setPower(leftfrontPower / 2);
        robot.leftrearDrive.setPower(leftrearPower / 2);
        robot.rightfrontDrive.setPower(rightfrontPower / 2);
        robot.rightrearDrive.setPower(rightrearPower / 2);

        
        //strafe if the x or b buttons are pressed
        if(gamepad1.b == true) {
            robot.leftfrontDrive.setPower(-0.5);
            robot.leftrearDrive.setPower(0.5);
            //rightfrontDrive.setPower(0.5);
            //rightrearDrive.setPower(-0.5);
            
        } else if(gamepad1.x == true) {
            robot.leftfrontDrive.setPower(0.5);
            robot.leftrearDrive.setPower(-0.5);
            //rightfrontDrive.setPower(-0.5);
            //rightrearDrive.setPower(0.5);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front(%.2f), right front(%.2f), left rear(%.2f), right rear(%.2f)", leftfrontPower, rightfrontPower,leftrearPower,rightrearPower);
        
        
        // moves the lift upwards
        if(gamepad2.right_bumper) {
            topright();
        }
        // If The left bumper is pressed the lift would move downwards
        else if (gamepad2.left_bumper) {
            topleft();
            
        } else if (gamepad2.right_trigger == 1) {
            toprightreverse();
            
        } else if (gamepad2.left_trigger == 1) {
            topleftreverse();
            
        } else {
            stopLift();
        }
            
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    
    //Power for the lift to go up
    public void topright() { 
        robot.TopLeft.setPower (0);
        robot.TopRight.setPower (1);
    }
    // power for the lift to go down
    public void topleft() { 
        robot.TopLeft.setPower (1);
        robot.TopRight.setPower (0);
    }
    public void topleftreverse() {
        robot.TopLeft.setPower (-1);
        robot.TopRight.setPower (0);
    }
    public void toprightreverse() {
        robot.TopLeft.setPower (0);
        robot.TopRight.setPower (-1);
    }
    // Turns off the lift 
    public void stopLift() {
        robot.TopLeft.setPower (0);
        robot.TopRight.setPower (0);
    }
}
