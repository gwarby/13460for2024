// Get package/imports
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
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
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;


// Begin code
public class AutoCommon extends LinearOpMode {
  public double dbg1 = 0.0, dbg2 = 0.0, dbg3 = 0.0, dbg4 = 0.0;
  // private members
  HardwareMap hardwareMap;
  // additional AprilTag vision objects
  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;
  
  private IMU imu;
  IntegratingGyroscope gyro;
  NavxMicroNavigationSensor navxMicro;

  // Declare motors/servos
  private DcMotor frontleft, rearleft, frontright, rearright, armrotate;
  private DcMotorEx armraiseEx, armraiseBEx;
  private Servo armlimiter;
  private CRServo armextend, grabber;
  private TouchSensor magSwitch;

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
    armraiseEx = hardwareMap.get(DcMotorEx.class, "armraiseM");
    armraiseBEx = hardwareMap.get(DcMotorEx.class, "armraiseB");
    armrotate = hardwareMap.get(DcMotor.class, "armrotate");
    armextend = hardwareMap.get(CRServo.class, "armextend");
    grabber = hardwareMap.get(CRServo.class, "grabber");
    magSwitch = hardwareMap.get(TouchSensor.class, "magSwitch");
    
    imu = hardwareMap.get(IMU.class, "imu");  // Retrieve the IMU from the hardware map
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
    
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontright.setDirection(DcMotorSimple.Direction.REVERSE);
    
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    armraiseEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armraiseBEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    /*
    PIDFCoefficients pidfOrig = armraiseEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
    PIDFCoefficients pidfNew = new PIDFCoefficients(2.5, 0.1, 0.2, 0.5);
    armraiseEx.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfNew); */
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
  
  public void initGyro() {
    navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "godIMU");
    gyro = (IntegratingGyroscope) navxMicro;
    while (navxMicro.isCalibrating()) {
      sleep(10);
    }
  }
  
  public void imuReInit() {
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.UP)));
  }
  
  public double getImuYaw() {
    AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
    Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    return (double) AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
  }

 public void resetArmEncoders() {
   armraiseEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   armraiseBEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    // Declare variables to be used later
    double frontLeftDistance, rearLeftDistance, frontRightDistance, rearRightDistance;
    double frontLeftPower, frontRightPower, rearLeftPower, rearRightPower;
    double maxDistance;
    
    // Multiply inputs (inches) by conversion factor (inches to ticks) to get rough target position (ticks)
    fwd_bck = fwd_bck * 54.2784576;
    right_left = right_left * 58.04;
    cw_ccw = cw_ccw * 13.217;
    // Calculate motor distances by adding/subtracting different inputs
    frontLeftDistance = -fwd_bck - right_left -cw_ccw;
    frontRightDistance = -fwd_bck + right_left + cw_ccw;
    rearLeftDistance = -fwd_bck + right_left -cw_ccw;
    rearRightDistance = -fwd_bck - right_left + cw_ccw;
    // Find the max distance a motor is going
    maxDistance = JavaUtil.maxOfList(JavaUtil.createListWith(frontLeftDistance, frontRightDistance, rearLeftDistance, rearRightDistance));
    // Set the power proportionate to the distance the motor has to travel relative to the others
    // This will ensure all wheels arrive at their target position at/near the same time, avoiding draggin
    // Set target position to the previously calculated distance
    int frontleftTargetPosition = (int) frontLeftDistance;
    int frontrightTargetPosition = (int) frontRightDistance;
    int rearleftTargetPosition = (int) rearLeftDistance;
    int rearrightTargetPosition = (int) rearRightDistance;
    
    frontleft.setTargetPosition(frontleftTargetPosition);
    frontright.setTargetPosition(frontrightTargetPosition);
    rearleft.setTargetPosition(rearleftTargetPosition);
    rearright.setTargetPosition(rearrightTargetPosition);
    
    frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
    frontLeftPower = power * frontLeftDistance / maxDistance;
    frontRightPower = power * frontRightDistance / maxDistance;
    rearLeftPower = power * rearLeftDistance / maxDistance;
    rearRightPower = power * rearRightDistance / maxDistance;
    
    frontleft.setPower(frontLeftPower);
    frontright.setPower(frontRightPower);
    rearleft.setPower(rearLeftPower);
    rearright.setPower(rearRightPower);
  }
  
  public void driveRaiseArm(double fwd, double lr, double turn, double power) {
    drive(fwd, lr, turn, power);
    raiseArm();
    while (frontright.isBusy() || armraiseEx.isBusy()) {
      sleep(10);
    }
  }
  
  public void driveLowerArm(double fwd, double lr, double turn, double power) {
    drive(fwd, lr, turn, power);
    lowerArmWait();
    while (frontright.isBusy() || armraiseEx.isBusy()) {
      sleep(10);
    }
  }
  
  public void driveWait(double fwd, double lr, double turn, double power) {
    drive(fwd, lr, turn, power);
    while (frontright.isBusy()) {
      sleep(15);
    }
  }
  
  public void raiseArm() {
    armraiseEx.setTargetPosition(4400);
    armraiseBEx.setTargetPosition(-4400);
    armrotate.setTargetPosition(-5500);
    armrotate.setPower(1);
    armraiseEx.setPower(0.95);
    armraiseBEx.setPower(0.95);
    armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (armraiseEx.isBusy()) {
      sleep(10);
    }
  }
  
  public void driveWaitDrive(double deltaZ) {
    lowerArm();
    driveWait(-16, 17, 0, 0.7);
    driveWait(0, 0, getImuYaw() - deltaZ, 0.5);
    drive(-3, 15.5, 0, 0.7);
    while (!(Math.abs(armrotate.getCurrentPosition()) < 200 && Math.abs(armrotate.getCurrentPosition() - 100) < 300)) {
      if (magSwitch.isPressed()) {
        armraiseEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armraiseBEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
    }
  }
  
  public void lowerArmWait() {
    armrotate.setTargetPosition(0);
    armraiseEx.setTargetPosition(0);
    armraiseBEx.setTargetPosition(0);
    armrotate.setPower(0.8);
    armraiseEx.setPower(0.95);
    armraiseBEx.setPower(0.95);
    armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (!(Math.abs(armrotate.getCurrentPosition()) < 200 && Math.abs(armrotate.getCurrentPosition() - 100) < 300)) {
      if (magSwitch.isPressed()) {
        armraiseEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armraiseBEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      }
    }
    
  }
  
  public void lowerArm() {
    armrotate.setTargetPosition(0);
    armraiseEx.setTargetPosition(0);
    armraiseBEx.setTargetPosition(0);
    armrotate.setPower(1);
    armraiseEx.setPower(0.85);
    armraiseBEx.setPower(0.85);
    armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }
  
  public void checkArmRes() {
    if (magSwitch.isPressed()) {
      armraiseEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
  }
  
  public void grabSpec() {
    grabber.setPower(-1);
    sleep(100);
  }
  
  public void holdSpec() {
    grabber.setPower(-0.03);
  }
  
  public void dropSpec() {
    grabber.setPower(0.5);
    sleep(650);
    grabber.setPower(0);
  }
  
  public void extendArm() {
    armextend.setPower(1);
    sleep(500);
    armextend.setPower(0);
  }
  
  public int[] getArmPos() {
    int[] ret = {armraiseEx.getCurrentPosition(), armrotate.getCurrentPosition()};
    return ret;
  }
  
  /*
  public void openGrabber() {
    grabber.setPosition(0.35);
  }
  
  public void closeGrabber() {
    grabber.setPosition(0.0);
  }
  
  public boolean grabberOpen() {
    return grabber.getPosition() == 0.35;
  }*/
  
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
        aprilTag = new AprilTagProcessor.Builder().setTagLibrary(AprilTagGameDatabase
        .getIntoTheDeepTagLibrary())
        .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));

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
      if (detection.id == requestId && detection.metadata != null) {
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
    
  //double getImuYaw() { return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES); }
  public void resetImuYaw() { imu.resetYaw(); }
  public void adjustEncoderTolerance(int newTolerance) {
    ((DcMotorEx) frontleft).setTargetPositionTolerance(newTolerance);
    ((DcMotorEx) frontright).setTargetPositionTolerance(newTolerance);
    ((DcMotorEx) rearleft).setTargetPositionTolerance(newTolerance);
    ((DcMotorEx) rearright).setTargetPositionTolerance(newTolerance);
  }
  public void closeAprilTag() {
    visionPortal.close();
  }
  public void setArmRaiseTolerances(int newTol) {
    armraiseEx.setTargetPositionTolerance(newTol);
    armraiseBEx.setTargetPositionTolerance(newTol);
  }
}
