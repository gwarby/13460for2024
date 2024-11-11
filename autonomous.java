package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "autonomous")
public class autonomous extends LinearOpMode 
{
  // keep this constant, even though it is duplicated in AutoCommon  
  final static double DRIVE_POWER = 0.5;
 
 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
    ElapsedTime programTime = new ElapsedTime();
    
    AutoCommon lib = new AutoCommon(
      hardwareMap);
    
    //lib.initAprilTag();
      
    //double distanceToWall = lib.getAprilTagDetection(13, "y");

    while (!opModeIsActive() & !isStopRequested())
    {
      telemetry.addLine("Waiting for start");
      //telemetry.addData("Distance to wall", distanceToWall);
      telemetry.update();
    }
    lib.closeGrabber();
    
    // wait for user to press start on Driver Station
    waitForStart();
    if (opModeIsActive()) {
      /************************************************************************
       * START / RUN OPMODE CODE:
       *   -perform autonomous driving based on identified april tag
       ************************************************************************/
      drive(0, 2, 0, 0.3);                          // Get off the wall
      int[] IdArray = lib.getCurrentAprilTagIds();  // Check to see if we see any april tags
      if (IdArray.length > 0) {                     // If we do, enter conditional
        int mainTagId = IdArray[0];                 // Set the first (should be only) id to be our main id
        telemetry.addData("April tag found", mainTagId);
        double distanceFromWall = lib.getAprilTagDetection(mainTagId).FtcPose.y;  // set our distance from the wall to a new variable
        lib.setArmHeight(43, 1);                    // raise the arm up
        lib.setArmAngle(90, 0.4);                   // rotate the arm up
        lib.drive(distanceFromWall - 30, 0, 0, 0.6);      // drive so we're 30" off the wall (while arm is moving)
        while (armraise.isBusy()) {                 // Wait to make sure arm is done moving
          sleep(10);
        }
        lib.drive(5, 0, 0, 0.3);                    // Drive forward 5" to get grabber over basket
        lib.openGrabber();                          // Open grabber
        lib.drive(-6, 0, 0, 0.3);                   // Back up 6" to get grabber outside bucket
        lib.setArmHeight(0, 1);                     // lower the arm down
        lib.setArmAngle(0, 0.05);                   // rotate the arm down
        lib.drive(0, 12, 0, 0.7);                   // slide right to look at april tag
        double deltaX = lib.getAprilTagDetection(mainTagId).FtcPose.x;    // Get position relative to april tag
        double deltaY = lib.getAprilTagDetection(mainTagId).FtcPose.y;
        double deltaZ = lib.getAprilTagDetection(mainTagId).FtcPose.yaw;
        lib.drive(0, 0, deltaZ, 0.4);               // Orient relative to april tag (to get lined up with sample)
        lib.drive(deltaY - 36, 0, 0, 0.5);
        lib.drive(0, 32 - deltaX, 0, 0.6);
        lib.drive(6.5, 0, 0, 0.3);                  // Drive forward to pick up sample
        lib.closeGrabber();
        lib.setArmHeight(43, 1);                    // Begin raising arm up
        lib.setArmAngle(90, 0.3);                   // Begin rotating arm up
        lib.drive(0, -36, 0, 0.5);                  // Slide left towards basket
        lib.drive(18, 0, 0, 0.5);                   // Move forward to basket
        lib.openGrabber();
        lib.drive(-12, 0, 0, 0.5);                  // Slide back from basket
        lib.setArmHeight(0, 1);                     // Send arm down
        lib.setArmAngle(0, 0.3);                    // rotate arm down
        lib.drive(0, 36, 0, 0.5);                   // slide right 36"
        lib.drive(6, 0, 0, 0.3);
        lib.closeGrabber();
        lib.setArmHeight(43, 1);
        lib.setArmAngle(90, 0.3);
        lib.drive(0, 36, 0, 0.4);
        lib.drive(6, 0, 0, 0.4);
        lib.openGrabber();
        lib.drive(-6, 0, 0, 0.4);
        lib.setArmHeight(0, 1);
        lib.setArmAngle(0, 0.3);
        lib.drive(0, 48, 0, 0.4);
        lib.drive(-12, 0, 0, 0.8);
      }
      
      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
