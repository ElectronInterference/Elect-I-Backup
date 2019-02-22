/* Copyright (c) 2017 FIRST. All rights reserved.*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/* This is the hardware program for our 2018-2019 Rover Ruckus robot, Molduga.
*
*This is the progam that declares all of our hardware, and has several
*useful functions that could be used by any opmode.
*
* --HARDWARE DEVICES--
*
*DEFINITE:
*
*leftrearDrive -- REV HD 40:1 hex motor (drive train)
*leftfrontDrive -- REV HD 40:1 hex motor (drive train)
*rightrearDrive -- REV HD 40:1 hex motor (drive train)
*rightfrontDrive -- REV HD 40:1 hex motor (drive train)
*acquisition -- REV HD 40:1 hex motor (mineral arm)
*lift -- Tetrix TorqueNado (lift)
*mineralLift -- REV smart cr servo (mineral pusher)
*
*
*releaser -- REV servo (team marker deployment)
*
*gyro -- the internal gyro in the REV hub
*touch -- the touch sensor used as an endstop for the lift
*
*
*SUBJECT TO CHANGE:
*
*N/A
*
*PLANNED:
*
*N/A
*
* --FUNCTIONS--
*
*StopDrive -- stops all drive motors
*StopAll -- stops all motors and servos
*LiftUp -- raise the lift
*LiftDown -- lower the lift
*DriveForward -- drive forward(inches)
*DriveBackward -- drive backward(inches)
*DriveLeft -- strafe left(inches)
*DriveRight -- strafe right(inches)
*Wait -- wait(millis)
*ReleaseMarker -- move Releaser servo up
*HoldMarker -- move Releaser servo down
*TurnDegrees -- turn a number of degrees(relative to the starting position)
*DetectCheeseBlock -- Detect which position the cheeseblock is in
*Turn -- Turn at a specific speed
*/




public class MoldugaHardware {
    
    
    //public opMode members
    
    //drive motors
    public DcMotor  leftrearDrive   = null;
    public DcMotor  rightrearDrive  = null;
    public DcMotor  leftfrontDrive     = null;
    public DcMotor  rightfrontDrive    = null;
    
    //lift and acquisition motors
    public DcMotor  lift    = null;
    public DcMotor  acquisition =null;
    
    //servos
    public CRServo  Releaser   = null;
    public CRServo  mineralLift = null;
    
    //sensors
    public BNO055IMU imu;
    public TouchSensor touch = null;
    
    // the tensorFlow files for detecting the minerals
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    

    // stores an instance of the vuforia localizer engine
    private VuforiaLocalizer vuforia;

    // stores an instance of the TensorFlow Object Detector engine
    private TFObjectDetector tfod = null;
    
    // the key that allows us to use vuforia
    private static final String VUFORIA_KEY = "ARZ9jK3/////AAABmUUaXNvhrkixjdVoQTApe8pKkOSgBAzrh4w49x76evPSBhJPD9ODNLDRoctaDi+4NXihmNZvGN0xSQfWqVc43szbn4ZQzKGAqkQo/GgHbVqADcuwbyEfGYnm7bC9FbbWQmik6swfX1uQo//lK+zHLote6GO5p63tORZ9VWZQjiBMMU8oqKWZmNmLGxsOz/8Xw1mrZodbIu7hUMkjEVbgxhONut4XPReM2Q2sipOZKy0YxSiWBTaXNvLi2egkhXAFL8F9DOatuzQyZdobssqwbDQ7emn1EP+OqgqbzJdV9C0YibdGb2+Sxzeppy+wUYAqBiCs7eHGVLuzqq6IPHxSTtvNYC5+JhWWJNx1Qyz7p1zm";
    
    //Variables for calculating drive distance with encoders
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ; 
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 6.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    
    //stores whether stop has been requested
    static boolean isStopRequested = false;
    
    //local opMode members
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MoldugaHardware(){
        
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        
        // Save reference to Hardware map
        hwMap = ahwMap;

        //define and initialize motors
        leftfrontDrive  = hwMap.get(DcMotor.class, "leftfrontDrive");
        rightfrontDrive = hwMap.get(DcMotor.class, "rightfrontDrive");
        leftrearDrive   = hwMap.get(DcMotor.class, "leftrearDrive");
        rightrearDrive  = hwMap.get(DcMotor.class, "rightrearDrive");
        lift            = hwMap.get(DcMotor.class, "lift");
        acquisition     = hwMap.get(DcMotor.class, "acquisition");
        
        //set the direction of the motors
        leftrearDrive.setDirection(DcMotor.Direction.FORWARD); 
        rightrearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftfrontDrive.setDirection(DcMotor.Direction.FORWARD); 
        rightfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to zero power
        leftrearDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightrearDrive.setPower(0);
        rightfrontDrive.setPower(0);
        lift.setPower(0);
        
        // Set drive motors to run without encoders.
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //Set the acquisition motor to run with an encoder
        acquisition.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        acquisition.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Define and initialize touch sensor.
        touch = hwMap.get(TouchSensor.class, "touch");
        
        //Define and initialize CR servos.
        mineralLift = hwMap.get(CRServo.class, "mineralLift");
        Releaser  = hwMap.get(CRServo.class, "Releaser");
        
        //set the direction of CR servos
        Releaser.setDirection(DcMotor.Direction.FORWARD);
        mineralLift.setDirection(DcMotor.Direction.FORWARD);
        
        //create parameters for initializing the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //We want to be able to use both the accelerometer and gyroscope, and 
        //measure angles in degrees and acceleration in m/s^2
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        //initialize the IMU based on our parameters
        imu.initialize(parameters);
        
        //initialize Vuforia and Tensorflow
        initVuforia();
        initTfod();
        
        //activate Tensorflow object detector
        if(tfod != null) {
            tfod.activate();
        }
    }
    
    
    /*Functions*/
    
    //wait for a number of seconds
    public void wait(int delay) {
        
        //create a variable to store how long we have waited
        int time = 0;
        
        //a while loop because if we only sleep, when the program is stopped it may crash.
        while(time < delay && !isStopRequested) {
            
            //sleep and ignore errors
            try {
            Thread.sleep(50);
            } catch (Exception e) {
                
            }
            time = time + 50;
            
            
        }

    }
    
    //stop all drive motors
    public void stopDrive() {
        leftrearDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightrearDrive.setPower(0);
        rightfrontDrive.setPower(0);
    }
    
    //stop all motors
    public void stopAll() {
        
        //stop using an encoder.
        leftrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //turn off all of the motors
        leftrearDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightrearDrive.setPower(0);
        rightfrontDrive.setPower(0);
        lift.setPower(0);
    }
    
    //drive forward a number of inches
    public void driveForward(double power, int targetDistance) {
        
        //reset the encoders
        leftrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //set the motors to run to a position
        leftrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set the target position to target inches times the number of encoder counts in an inch
        leftrearDrive.setTargetPosition((int)(COUNTS_PER_INCH * targetDistance));
        rightrearDrive.setTargetPosition((int)(COUNTS_PER_INCH * targetDistance));
        leftfrontDrive.setTargetPosition((int)(COUNTS_PER_INCH * targetDistance));
        rightfrontDrive.setTargetPosition((int)(COUNTS_PER_INCH * targetDistance));
        
        //start the motors at the inputted power
        leftfrontDrive.setPower(Math.abs(power * 0.7));  //TODO: THIS NEEDS TO BE FIXED BETTER
        rightfrontDrive.setPower(Math.abs(power));
        leftrearDrive.setPower(Math.abs(power));
        rightrearDrive.setPower(Math.abs(power));
        
        //Wait for one of the motors to reach its target or the program to be stopped
        while(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftrearDrive.isBusy() && rightrearDrive.isBusy() && !isStopRequested) {
            
        }
        //stop all motors
        stopAll();
        
    }
    
    //drive backward for a number of inches
    public void driveBackward(double power, double targetDistance) {
        
        //reset the drive motor encoders
        leftrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //set the motors to run to a position
        leftrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set the target position to target inches times the number of encoder steps in an inch
        leftrearDrive.setTargetPosition((int)(-COUNTS_PER_INCH * targetDistance));
        rightrearDrive.setTargetPosition((int)(-COUNTS_PER_INCH * targetDistance));
        leftfrontDrive.setTargetPosition((int)(-COUNTS_PER_INCH * targetDistance));
        rightfrontDrive.setTargetPosition((int)(-COUNTS_PER_INCH * targetDistance));
        
        //start the motors at the inputted power
        leftfrontDrive.setPower(Math.abs(power * 0.7)); //TODO: THIS NEEDS TO BE FIXED BETTER
        rightfrontDrive.setPower(Math.abs(power));
        leftrearDrive.setPower(Math.abs(power));
        rightrearDrive.setPower(Math.abs(power));
        
        //do nothing while none of the motors have reached their goal and the program hasn't been stopped.
        while(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftrearDrive.isBusy() && rightrearDrive.isBusy() && !isStopRequested) {
            //Loop body can be empty
        }
        
        //stop the motors
        stopAll(); 
    }
    
    //move the Releaser servo  back into the robot
    public void markerHold() {
        Releaser.setPower(-0.7); //Make the releaser move
        wait(200); //wait for the servo to go inside the robot
        Releaser.setPower(0); //stop the releaser
    }
    
    //Push the team marker off of its pedestal
    public void markerRelease() {
        Releaser.setPower(2000); //Make the releaser move
        wait(1000); // wait for the releaser to push the team marker
        Releaser.setPower(0); // stop the releaser
    }
    
    //Move the lift up. This is designed to be started while the lift is in the lowest position
    public void liftUp() {
        lift.setPower(-0.5); // start the lift
        wait(2900); // wait for the lift to be raised to where it can unlatch from the lander
        lift.setPower(0); // stop the lift
    }
    
    //move the lift down until it reaches the button endstop
    public void liftDown() {
        lift.setPower(0.8); // start the lift
        
        //wait while the button hasn't been pressed and the program hasn't been stopped
        while(!touch.isPressed() && !isStopRequested) {
            
        }
        lift.setPower(0); // stop the lift
    }
    
    public void turn(double power) {
        
        //tell the motors not to use an encoder.
        leftrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //Set the motors to the specified power
        leftrearDrive.setPower(power);
        leftfrontDrive.setPower(power * 0.7);
        rightrearDrive.setPower(power);
        rightfrontDrive.setPower(power);
    }
    
    //strafe left a number of inches
    public void driveLeft(double power, double targetDistance) {
        
        //Reset the motor encoders
        leftrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //set the motors to run to a position
        leftrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set the target position to target inches times the number of encoder counts in an inch
        int target = (int)(COUNTS_PER_INCH * targetDistance);
        leftrearDrive.setTargetPosition(target);
        rightrearDrive.setTargetPosition(-target);
        leftfrontDrive.setTargetPosition(-target);
        rightfrontDrive.setTargetPosition(target);
        
        //start the motors at the inputted power
        leftfrontDrive.setPower(Math.abs(power * 0.7));
        rightfrontDrive.setPower(Math.abs(power));
        leftrearDrive.setPower(Math.abs(power));
        rightrearDrive.setPower(Math.abs(power));
        
        //Wait until one of the motors has reached it's goal or the program has been stopped
        while(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftrearDrive.isBusy() && rightrearDrive.isBusy() && !isStopRequested) {
            //Loop body can be empty
        }
        //stop the motors
        stopAll();
    }
    
    //Strafe right a number of inches
    public void driveRight(double power, double targetDistance) {
        
        //Reset the motor encoders
        leftrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //set the motors to run to a position
        leftrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightrearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftfrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //set the target position to target inches times the number of encoder counts in an inch
        int target = (int)(COUNTS_PER_INCH * targetDistance);
        leftrearDrive.setTargetPosition(-target);
        rightrearDrive.setTargetPosition(target);
        leftfrontDrive.setTargetPosition(target);
        rightfrontDrive.setTargetPosition(-target);
        
        //start the motors at the inputted power
        leftfrontDrive.setPower(Math.abs(power * 0.7));
        rightfrontDrive.setPower(Math.abs(power));
        leftrearDrive.setPower(Math.abs(power));
        rightrearDrive.setPower(Math.abs(power));
        
        //Wait until one of the motors has reached it's goal or the program has been stopped
        while(leftfrontDrive.isBusy() && rightfrontDrive.isBusy() && leftrearDrive.isBusy() && rightrearDrive.isBusy() && !isStopRequested) {
            //Loop body can be empty
        }
        //Stop all the motors
        stopDrive();
    }
    
    
    
    //Turn a desired number of degrees
    public void turnDegrees(int target) {
        //try to turn four times. This improves precision
        for(int i = 0; i < 4; i ++) {
            
            //Run the motors without encoders
            leftrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightrearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftfrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            leftrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            rightrearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            leftfrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            
            //find our current heading
            float currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            //if we need to turn left...
            if(currentAngle > target) {
                //turn left
                leftrearDrive.setPower(-0.3);
                leftfrontDrive.setPower(-0.3);
                rightrearDrive.setPower(0.3);
                rightfrontDrive.setPower(0.3);
                
                //wait until the robot has turned far enough
                while (currentAngle > target + 5 && !isStopRequested) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            
            //if we need to turn right...
            }else if(currentAngle < target) {
                //turn right
                leftrearDrive.setPower(0.3);
                leftfrontDrive.setPower(0.3);
                rightrearDrive.setPower(-0.3);
                rightfrontDrive.setPower(-0.3);
                //wait until the robot has turned far enough
                while (currentAngle < target -5 && !isStopRequested) {
                    currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                }
            //if we do not know which way to turn, just stop
            }else{
                stopAll();
            }
            //stop after we have finished
            stopAll();
        }
    }
    
    //functions for Vuforia
    
        /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        
        
        //tell vuforia that we have a key and which camera to use
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }



    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL);
    }
    


    //Find the distance the cheeseblock is to the left or right.
    public int findCheeseblockX() {
    
    //if tensorflow is initialized...
    if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                
                //if there is no error...
                if (updatedRecognitions.size() >= 0) {
                      
                    //create variables for the gold mineral's location
                    int goldMineralX = -1;
                    float minY = 9999;
                    
                    //store a variable for the lowest mineral found
                    Recognition lowestMineral = null;
                    
                    //loop once for each mineral found
                    for (Recognition recognition : updatedRecognitions) {
                        
                        //make sure the mineral is not higher than the last mineral or too low
                        if(recognition.getImageHeight() - recognition.getBottom() < minY) {
                            if(recognition.getImageHeight() - recognition.getBottom() > 600) {
                                
                                //set the lowest mineral found to be the the chosen one
                                minY = recognition.getImageHeight() - recognition.getBottom();
                                lowestMineral = recognition;
                            }
                        }
                    }
                    //if we found a mineral that fit our parameters...
                    if(lowestMineral != null) {
                        
                        //get the mineral's location on the screen and return it
                        goldMineralX = (int)(lowestMineral.getLeft());
                        return ((int)(lowestMineral.getLeft()));

                    }
        //if something didn't work, return an error
                } else {
                    return(-3);
                }
            } else {
                return(-2);
            }
        }
        return(-1);
    }
}
