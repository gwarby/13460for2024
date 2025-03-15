package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "BasketAuto", preselectTeleOp = "finalDrive")
public class BasketAuto extends LinearOpMode 
{
  // keep this constant, even though it is duplicated in AutoCommon  
  final static double DRIVE_POWER = 0.5;
  boolean gyroInit = false;
  

 /**
   * This function is executed when this OpMode is selected & Init is pressed
   */
  @Override
  public void runOpMode() 
  {
    ElapsedTime programTime = new ElapsedTime();

    AutoCommon lib = new AutoCommon(
      hardwareMap);

    lib.initAprilTag();

    //double distanceToWall = lib.getAprilTagDetection(13, "y");
    double deltaX = 0, deltaY = 0, deltaZ = 0;
    while (!opModeIsActive() & !isStopRequested())   
    {
      lib.resetArmEncoders();
      lib.holdSpec();
      //telemetry.addData("current detections", lib.getCurrentAprilTagIds());
      telemetry.addData("tag 13 output", (lib.getAprilTag_BlueRight() == null));
      if (lib.getAprilTag_BlueRight() != null) {
        telemetry.addData("Distance to wall", lib.getAprilTag_BlueRight().ftcPose.y);
        deltaY = lib.getAprilTag_BlueRight().ftcPose.y;
        deltaX = lib.getAprilTag_BlueRight().ftcPose.x;
        deltaZ = lib.getAprilTag_BlueRight().ftcPose.yaw;
        telemetry.addLine("=============");
        if (Math.abs(deltaZ-2) > 0.75) {
          telemetry.addLine("CORRECTING ORIENTATION");
          lib.drive(0, 0, -deltaZ-2, 0.1);
        }
        telemetry.addLine("READY TO START");
        telemetry.addLine("DETECTED BLUE");
        telemetry.addLine("=============");
        if (gyroInit) {
          telemetry.addData("Imu Orientation", lib.getImuYaw());
        } else {
          lib.initGyro();
          gyroInit = true;
        }
        telemetry.addData("deltax", deltaX);
        telemetry.addData("deltaY", deltaY);
        telemetry.addData("deltaZ", deltaZ);
      } else if (lib.getAprilTag_RedRight() != null) {
        telemetry.addData("Distance to wall", lib.getAprilTag_RedRight().ftcPose.y);
        deltaY = lib.getAprilTag_RedRight().ftcPose.y;
        deltaX = lib.getAprilTag_RedRight().ftcPose.x;
        deltaZ = lib.getAprilTag_RedRight().ftcPose.yaw;
        telemetry.addLine("=============");
        if (Math.abs(deltaZ-2) > 0.75) {
          telemetry.addLine("CORRECTING ORIENTATION");
          lib.drive(0, 0, -deltaZ-2, 0.1);
        }
        telemetry.addLine("READY TO START");
        telemetry.addLine("DETECTED RED");
        telemetry.addLine("=============");
        if (gyroInit) {
          telemetry.addData("Imu Orientation", lib.getImuYaw());
        } else if (deltaZ < 0.75) {
          lib.initGyro();
          gyroInit = true;
        }
        telemetry.addData("deltax", deltaX);
        telemetry.addData("deltaY", deltaY);
        telemetry.addData("deltaZ", deltaZ);
      } else {
        telemetry.addLine("=============");
        telemetry.addLine("NO TAGS DETECTED");
        telemetry.addLine("=============");
      }
      telemetry.update();
    }

    // wait for user to press start on Driver Station
    waitForStart();
    lib.closeAprilTag();
    if (opModeIsActive()) {
      /************************************************************************
       * START / RUN OPMODE CODE:
       *   -perform autonomous driving based on identified april tag
       ************************************************************************/
      
      lib.driveRaiseArm(deltaY - 23, deltaX + 0.5, -deltaZ, 0.6);      // drive so we're 30" off the wall (while arm is moving)
      lib.driveWait(5,0,-30, 0.5);
      sleep(100);
      lib.dropSpec();                          // Open grabber
      // First Spec Dropped
      lib.driveWait(0, 0, lib.getImuYaw() - deltaZ, 0.5);
      lib.driveWaitDrive(deltaZ);
      lib.checkArmRes();
      lib.extendArm();
      lib.grabSpec();
      // Second Spec Grabbed
      lib.driveWait(7.5, 0, 0, 0.2);
      lib.holdSpec();
      lib.driveRaiseArm(11, -29, 0, 0.7);
      lib.driveWait(0, 0, -45, 0.4);
      sleep(100);
      lib.dropSpec();
      // Second Spec Dropped
      lib.driveWait(-6, 0, lib.getImuYaw()-deltaZ, 0.4);
      lib.driveWait(0, 0, lib.getImuYaw()-deltaZ, 0.7);
      lib.driveLowerArm(0, 26, 0, 0.6);
      lib.checkArmRes();
      lib.grabSpec();
      // Third Spec Grabbed
      lib.driveWait(7.5, 0, 0, 0.2);
      lib.holdSpec();
      lib.driveRaiseArm(-1, -29, 0, 0.7);
      lib.driveWait(0, 0, -50, 0.6);
      sleep(350);
      lib.dropSpec();
      // Third Spec Dropped
      lib.driveWait(-5, 0, lib.getImuYaw()-deltaZ, 0.6);
      lib.driveLowerArm(-3, 40, 0, 0.7);
      lib.driveWait(-12, 0, 0, 0.6);

      telemetry.addLine("Opmode COMPLETE");
      telemetry.update();
      while (opModeIsActive()) {
        sleep(100); // this opMode doesn't use the boilerplate loop pattern, if this changes, be sure to remove the sleep
      }
    }
  }
}
