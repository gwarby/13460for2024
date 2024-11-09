// Get package/imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Begin code
public class AutoCommon extends LinearOpMode {
  public double dbg1 = 0.0, dbg2 = 0.0, dbg3 = 0.0, dbg4 = 0.0;
  // private members
  HardwareMap hardwareMap;
  // additional AprilTag vision objects
  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;
  
  private IMU imu;

  // Declare motors/servos
  private DcMotor frontleft, rearleft, frontright, rearright, armraise, armrotate;
  private Servo grabber, armlimiter;
  private CRServo armextend;

  // This class isn't an opmode that should be run on the robot
  // but it extends (inherits from) LinearOpMode so it can access
  // hardwareMap & sleep()
  public AutoCommon(HardwareMap hwMap) {
    /************************************************************************
     *   -map hardware specific to our robot
     *   -config hardware settings (like reversed motor directions)
     *   -config camera & OpenCV pipeline used for prop location detection
     ************************************************************************/
    hardwareMap = hwMap;
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearright = hardwareMap.get(DcMotor.class, "rearright");
    armraise = hardwareMap.get(DcMotor.class, "armraise");
    armrotate = hardwareMap.get(DcMotor.class, "armrotate");
    armextend = hardwareMap.get(CRServo.class, "armextend");
    grabber = hardwareMap.get(Servo.class, "grabber");
    
    imu = hardwareMap.get(IMU.class, "imu");  // Retrieve the IMU from the hardware map
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
    
    rearleft.setDirection(DcMotorSimple.Direction.REVERSE);
    rearright.setDirection(DcMotorSimple.Direction.REVERSE);
    
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
  }
  
  //private void sleep(int ms) { LinearOpMode.sleep(ms); }
  
  // This is a library class, not a proper opMode, but we need access to sleep, so we extend linearOpMode, and as such, must implement runOpMode()
  @Override
  public void runOpMode() { }
  
  
  ////////////////
  public void imuResetYaw()
  {
    imu.resetYaw();
  }
  
  public void imuReInit() {
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.UP)));
  }

 

  public double LastDrivePower = 0.0;
  public int WhileLoopId = -1;
  public int Count1 = 0, Count2 = 0, Count3 = 0, Count4 = 0;
  /************************************************************************
   * COMMON DRIVE FUNCTION:
   *   Used to simplify autonomous programming with simple & readable cmds
   ************************************************************************/
  public void drive(double fwd_bck, double right_left, double cw_ccw, double power) {
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Count1 = 0; 
    Count2 = 0; 
    Count3 = 0;
    Count4 = 0;
    // Declare variables to be used later
    double frontLeftDistance, rearLeftDistance, frontRightDistance, rearRightDistance;
    double frontLeftPower, frontRightPower, rearLeftPower, rearRightPower;
    double maxDistance, frontLeftIMU = 0, frontRightIMU = 0, rearLeftIMU = 0, rearRightIMU = 0;
    double driveAngle = 0.0, currentPositionTicks, maxPower = 0.5;
    boolean useNormCalc, holdAngle = false;
    int driveMagnitudeLevel = 0;  // 1: short distance, 2: medium distance, 3: long distance
    if (cw_ccw == 0) {      // if there is no commanded rotation, then IMU could be used to hold heading at start of drive cmd
      adjustEncoderTolerance(9);
      holdAngle = true;     // bool denoting we are trying to hold yaw angle
      driveAngle = getImuYaw();   // snapshot of IMU yaw reading at beginning of drive cmd
      double totalDistance = Math.sqrt(Math.pow(fwd_bck, 2) + Math.pow(right_left, 2));   // dist calc for "diagonal" of fwd_bck & right_left
      if (totalDistance <= 7) {
        maxPower = 0.3;
        driveMagnitudeLevel = 1;  // short
      } else if (totalDistance > 7 && totalDistance < 36) {
        maxPower = 0.55;
        driveMagnitudeLevel = 2;  // medium
      } else {
        maxPower = 0.9;
        driveMagnitudeLevel = 3;  // long
      }
    } else {  // Rotation is part (or likely all) of the drive cmd
      holdAngle = false;
      adjustEncoderTolerance(4);
      if (cw_ccw > 45) {
        maxPower = 0.55;
        driveMagnitudeLevel = 2;  // short (rotation version)
      } else {
        maxPower = 0.35;
        driveMagnitudeLevel = 1;  // medium (rotation version)
      }
    }
    // Reset encoders
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Multiply inputs (inches) by conversion factor (inches to ticks) to get rough target position (ticks)
    fwd_bck = fwd_bck * 54.35714;
    right_left = right_left * 57.04;
    cw_ccw = cw_ccw * 12.85;
    // Calculate motor distances by adding/subtracting different inputs
    frontLeftDistance = -fwd_bck + right_left -cw_ccw;
    frontRightDistance = -fwd_bck + right_left + cw_ccw;
    rearLeftDistance = -fwd_bck -right_left -cw_ccw;
    rearRightDistance = -fwd_bck - right_left + cw_ccw;
    
    useNormCalc = (Math.abs(frontLeftDistance) > 2880);
    
    // Find the max distance a motor is going
    maxDistance = JavaUtil.maxOfList(JavaUtil.createListWith(frontLeftDistance, frontRightDistance, rearLeftDistance, rearRightDistance));
    // Set the power proportionate to the distance the motor has to travel relative to the others
    // This will ensure all wheels arrive at their target position at/near the same time, avoiding draggin
    // Set target position to the previously calculated distance
    frontleft.setTargetPosition((int) frontLeftDistance);
    frontright.setTargetPosition((int) frontRightDistance);
    rearleft.setTargetPosition((int) rearLeftDistance);
    rearright.setTargetPosition((int) rearRightDistance);
    
    frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    
    frontLeftPower = frontLeftDistance / maxDistance;
    frontRightPower = frontRightDistance / maxDistance;
    rearLeftPower = rearLeftDistance / maxDistance;
    rearRightPower = rearRightDistance / maxDistance;
    
    while (frontright.isBusy()) {
      sleep(10);
    }
    
    /*
    // Set the motors to run to position
    // Calculate distance milestones for (possible) power modulation
    double firstQuarter = Math.abs(frontLeftDistance / 4);
    double firstEighth = firstQuarter / 2;
    double lastQuarter = 3 * firstQuarter;
    double lastEighth = firstEighth * 7;
    frontleft.setPower(0.3);
    frontright.setPower(0.3);
    rearleft.setPower(0.3);
    rearright.setPower(0.3);
    
    if (driveMagnitudeLevel == 1 || driveMagnitudeLevel == 2) { //(maxPower == 0.3 || maxPower == 0.7) {   // SHORT, MEDIUM DRIVES & ROTATIONS
      while (frontleft.isBusy()) {
        frontleft.setPower(maxPower * frontLeftPower);
        frontright.setPower(maxPower * frontRightPower);
        rearleft.setPower(maxPower * rearLeftPower);
        rearright.setPower(maxPower * rearRightPower);
        WhileLoopId = 1; }
        
    } else { //if (cw_ccw == 0.0) {  // LONG DRIVES W/O ROTATION
      while (frontleft.isBusy()) {
        currentPositionTicks = Math.abs(frontleft.getCurrentPosition());    // Only call .getCurrentPosition() once per cycle, instead of five

        
        // Ramp smoothly over 1st & last quarter of drive        
        if (currentPositionTicks <= firstQuarter) {  // in 1st 1/4
          power = 0.3 + currentPositionTicks / firstQuarter * (maxPower-0.3);  // ramp from 0.3..maxPower as dist goes 0..0.25*totalDist
        } else if (currentPositionTicks < lastQuarter) {  // between 1/4th & 3/4th
          power = maxPower;
        } else { // lastQuarter
          power = maxPower - (currentPositionTicks - lastQuarter) / firstQuarter * (maxPower-0.3);  // ramp from maxPower..0.3 as dist goes 0.75*totalDist..totalDist
        }
        
        /*
        // Use fixed steps in power at 1/8, 1/4, 3/4 & 7/8 of total Dist
        if (currentPositionTicks <= firstEighth) {  // in 1st 1/8th
          power = 0.3;
        } else if (currentPositionTicks < firstQuarter) {  // between 1/8th & 1/4th
          power = 0.7;
        } else if (currentPositionTicks < lastQuarter) {  // between 1/4th & 3/4th
          power = maxPower;
        } else if (currentPositionTicks < lastEighth) {  // between 3/4th & 7/8th
          power = 0.7;
        } else { // 7/8th to end
          power = 0.3;
        }
        
        
        frontleft.setPower(power * frontLeftPower);
        frontright.setPower(power * frontRightPower);
        rearleft.setPower(power * rearLeftPower);
        rearright.setPower(power * rearRightPower);
        WhileLoopId = 2;
      }
    } 

    /*
    } else { //if (cw_ccw == 0.0) {  // LONG DRIVES W/O ROTATION
      while (frontleft.isBusy()) {
        currentPositionTicks = Math.abs(frontleft.getCurrentPosition());    // Only call .getCurrentPosition() once per cycle, instead of five
        if (currentPositionTicks <= firstEighth || currentPositionTicks > lastEighth) {
          power = 0.3;
        } else if ((currentPositionTicks >= firstEighth && currentPositionTicks < firstQuarter) || currentPositionTicks > lastQuarter) {
          power = 0.7;
        } else if (maxPower == 0.9) {
          power = 0.9;
        }
        frontleft.setPower(power * frontLeftPower);
        frontright.setPower(power * frontRightPower);
        rearleft.setPower(power * rearLeftPower);
        rearright.setPower(power * rearRightPower);
        WhileLoopId = 2;
      }
    } 
    */

    /*
    else {
      while (frontleft.isBusy()) {
        currentPositionTicks = Math.abs(frontleft.getCurrentPosition());    // Only call .getCurrentPosition() once per cycle, instead of five
        if (currentPositionTicks > lastEighth) {
          power = 0.3;
        } else if (maxPower == 0.7) {
          if (currentPositionTicks > lastQuarter) {
            power = 0.4;
          } else if (currentPositionTicks > firstQuarter) {
            power = maxPower;
          }
        } else if (currentPositionTicks > firstEighth) {
          power = 0.4;
        }
        frontleft.setPower(power * frontLeftPower);
        frontright.setPower(power * frontRightPower);
        rearleft.setPower(power * rearLeftPower);
        rearright.setPower(power * rearRightPower);
        WhileLoopId = 3;
      }
    }
    */
    // Set motor power to zero
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
  }
  
  /* setArmHeight()
   *
   * Raise the arm to a set height in inches by converting from ticks
   */
  public void setArmHeight(double height, double power) {
    int raiseTargetTicks = (int) (height * 125.6);
    armraise.setTargetPosition(raiseTargetTicks);
    armraise.setPower(power);
  }
  
  public void setArmHeightWait(double height, double power) {
    int raiseTargetTicks = (int) (height * 125.6);
    armraise.setTargetPosition(raiseTargetTicks);
    armraise.setPower(power);
    while (armraise.isBusy()) {
      sleep(10);
    }
  }

  public void setArmAngle(double angle, double power) {
      int rotateTargetTicks = (int) (angle * 4);
      armrotate.setTargetPosition(rotateTargetTicks);
      armraise.setPower(power);
  }

  public void setArmAngleWait(double angle, double power) {
      int rotateTargetTicks = (int) (angle * 4);
      armrotate.setTargetPosition(rotateTargetTicks);
      armraise.setPower(power);
      while (armraise.isBusy()) {
          sleep(10);
      }
  }
  
  public void openGrabber() {
    grabber.setPosition(0.25);
  }
  
  public void closeGrabber() {
    grabber.setPosition(0.0);
  }
  
  public boolean grabberOpen() {
    return grabber.getPosition() == 0.35;
  }
  
  /* setArmExtend()
   * 
   * Extend the last joint of the arm
   */
   
   public void setArmExtend(double percent) {
     
   }
  
  
  /**
   * Initialize the AprilTag processor.
   */
  public void initAprilTag() {
  
      // Create the AprilTag processor.
      AprilTagProcessor.Builder atpBuilder = new AprilTagProcessor.Builder();
      // lots of options could be overriden here, such as:
      // aprilTag.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
      aprilTag = atpBuilder.build();

      // Adjust Image Decimation to trade-off detection-range for detection-rate.
      // eg: Some typical detection data using a Logitech C920 WebCam
      // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
      // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
      // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
      // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
      // Note: Decimation can be changed on-the-fly to adapt during a match.
      //aprilTag.setDecimation(3);
  
      // Create the vision portal by using a builder.
      VisionPortal.Builder builder = new VisionPortal.Builder();
      builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));

      // Choose a camera resolution. Not all cameras support all resolutions.
      //builder.setCameraResolution(new Size(544, 288));
  
      // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
      builder.enableLiveView(false);  // this will be used in the middle of our opmodes, so preview won't be possible anyway
  
      // Set the stream format; MJPEG uses less bandwidth than default YUY2.
      //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
  
      // Set and enable the processor.
      builder.addProcessor(aprilTag);
  
      // Build the Vision Portal, using the above settings.
      visionPortal = builder.build();
  
      // Disable or re-enable the aprilTag processor at any time.
      //visionPortal.setProcessorEnabled(aprilTag, true);
  
  }   // end method initAprilTag()

  /*********************************************************************
   * getAprilTagDetection(tagID)
   *  - returns null if the tagID is not detected
   *  - returns an AprilTagDetection object if the tagID is detected,
   *  - AprilTagDetection contains:
   *    XYZ (inch): detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z
   *    .ftcPose.y: dist 'forward' from camera to parallel plane of tag
   *    .ftcPose.x: distance 'right' from line straight out from camera to the tag
   *    .ftcPose.z: distance up from line straight out of camera to the tag
   *    (i.e. x,y,z are how far out, right, & up the camera would need to 
   *     move to reach the tag.)
   *    PRY (deg): detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw
   *    RBE (inch, deg, deg): detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation
   * 
   *    NOTE: The y distance was some inches offset from our tape
   *          measure measurements, so it should not be trusted as
   *          an absolute value, but it seemed to be RELATIVELY
   *          accurate from scenario to scenario.  We recorded the
   *          y measurement in a run where we observed our original
   *          auto code dropped the backdrop pixel about 1" away from 
   *          the backdrop.  This .ftcPose.y happened to be ~15.5".
   *          We decided if the AprilTag pose data was reliable, this
   *          meant our desired .ftcPose.y at this point of the program
   *          was actually 14.5" (the 15.5" reported minus 1" closer).
   *          On subsequent runs, we took the difference of the 
   *          .ftcPose.y report and 14.5 to adjust our last drive()
   *          up to the backdrop.
   *          The math in our code looks a bit backward because our
   *          robot backs into the board, and is using a backwards
   *          facing camera to observe the AprilTag.  So our adjustment
   *          calculation subtracts the .ftcPose.y from 14.5 (instead of
   *          the other way around), and then adds this value to the 
   *          negative drive forward command (which causes the robot to
   *          backup into the backdrop).
   *          
   *********************************************************************/
  public AprilTagDetection getAprilTagDetection(int requestId) {
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

    for (AprilTagDetection detection : currentDetections) {  // Step through the list of detections and look for the requested id
      if (detection.id == requestId) {
        return detection;
      }
    }   // end for() loop
    
    return null;  // the requested tag wasn't found
  }
  // CENTERSTAGE & the current FTC SDK are setup with tag IDs on the backdrops as follows
  public AprilTagDetection getAprilTag_BlueLeft() { return getAprilTagDetection(11); }
  public AprilTagDetection getAprilTag_BlueMiddle() { return getAprilTagDetection(12); }
  public AprilTagDetection getAprilTag_BlueRight() { return getAprilTagDetection(13); }
  public AprilTagDetection getAprilTag_RedLeft() { return getAprilTagDetection(14); }
  public AprilTagDetection getAprilTag_RedMiddle() { return getAprilTagDetection(15); }
  public AprilTagDetection getAprilTag_RedRight() { return getAprilTagDetection(16); }
  
  public List<Integer> getCurrentAprilTagIds() {
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    List<Integer> detectedIds = new ArrayList<Integer>();
    for (AprilTagDetection detection: currentDetections) {
      detectedIds.add(detection.id);
    }
    return detectedIds;
  }
    
  double getImuYaw() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }
  public void resetArmEncoders() {
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
  public void resetImuYaw() { imu.resetYaw(); }
  public void adjustEncoderTolerance(int newTolerance) {
    ((DcMotorEx) frontleft).setTargetPositionTolerance(newTolerance);
    ((DcMotorEx) frontright).setTargetPositionTolerance(newTolerance);
    ((DcMotorEx) rearleft).setTargetPositionTolerance(newTolerance);
    ((DcMotorEx) rearright).setTargetPositionTolerance(newTolerance);
  }
}
