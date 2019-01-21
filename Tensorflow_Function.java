package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.MoldugaHardware;
//The libraries for tensorflow
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

public class Tensorflow_Function {

    // todo: write your code here
    
     //access our hardware program
    MoldugaHardware robot   = new MoldugaHardware();
        public TFObjectDetector tfod;
    public VuforiaLocalizer vuforia;


  public int findCheeseblockX() {
    
    if (tfod != null) {
      // getUpdatedRecognitions() will return null if no new information is available since
      // the last time that call was made.
      List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
      if (updatedRecognitions != null) {
        //telemetry.addData("# Object Detected", updatedRecognitions.size());
        if (updatedRecognitions.size()  >= 1) {
          int goldMineralX = -1;
          for (Recognition recognition : updatedRecognitions) {
            if (recognition.getLabel().equals(robot.LABEL_GOLD_MINERAL)) {
              goldMineralX = (int) recognition.getLeft();
            return goldMineralX;
            }
          }
        }
      }
    }else{
      return -1;
    }
    return -1;
  }

        
    
}

