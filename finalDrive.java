package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "finalDrive")
public class finalDrive extends LinearOpMode {

  IntegratingGyroscope gyro;
  NavxMicroNavigationSensor navxMicro;

  private DcMotor frontright;
  private DcMotor rearleft;
  private DcMotor frontleft;
  private DcMotor rearright;
  private DcMotor armrotate;
  private DcMotorEx armraiseEx;
  private DcMotorEx armraiseBEx;
  private CRServo armextend;
  private Servo grabLeft, grabRight;
  private Servo eyeLeft, eyeRight;
  private TouchSensor magSwitch;
  private IMU imu;
    
  @Override
  public void runOpMode() {
      
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    rearright = hardwareMap.get(DcMotor.class, "rearright");
    armextend = hardwareMap.get(CRServo.class, "armextend");
    armrotate = hardwareMap.get(DcMotor.class, "armrotate");
    armraiseEx = hardwareMap.get(DcMotorEx.class, "armraiseM");
    armraiseBEx = hardwareMap.get(DcMotorEx.class, "armraiseB");
    grabLeft = hardwareMap.get(Servo.class, "grabLeft");
    grabRight = hardwareMap.get(Servo.class, "grabRight");
    eyeLeft = hardwareMap.get(Servo.class, "eyeLeft");
    eyeRight = hardwareMap.get(Servo.class, "eyeRight");
    magSwitch = hardwareMap.get(TouchSensor.class, "magSwitch");
    imu = hardwareMap.get(IMU.class, "imu");  // Retrieve the IMU from the hardware map
    imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
 
   
    
    ElapsedTime programTime = new ElapsedTime();

    // Put initialization blocks here.
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontright.setDirection(DcMotorSimple.Direction.REVERSE);
    armraiseBEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    armraiseEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armraiseBEx.setDirection(DcMotorSimple.Direction.REVERSE);
    
    boolean armUp = false, canRot, specMode = false, motorsFighting, grabSpec = false;
    int timesFought = 0;
    double lastGrabbed = 0, lastRot = 0;
    double cmdFwd, cmdSide, cmdTurn, driveCo = 1;
    YawPitchRollAngles robotOrientation;
    
    final double RIGHT_GRABBED_POS = 1;
    final double LEFT_GRABBED_POS = 0.25;
    final double RIGHT_RELEASE_POS = 0.6;
    final double LEFT_RELEASE_POS = 0.35;
    
    final int RAISE_BASKET_UP = 4800;
    final int ROTATE_BASKET_UP = -4700;
    
    final int RAISE_CLIP_PLACE = 2000;
    final int ROTATE_CLIP_PLACE = -4100;
    
    final int RAISE_CLIP_GRAB = 0;
    final int ROTATE_CLIP_GRAB = -2900;
    
    final int ROTATE_GRAB_SUB = -1500;
    
    armraiseEx.setTargetPositionTolerance(100);
    armraiseBEx.setTargetPositionTolerance(800);
    
    waitForStart();
    if (gamepad1.y || gamepad2.y) {
      armrotate.setTargetPosition(ROTATE_BASKET_UP);
      armraiseEx.setTargetPosition(RAISE_BASKET_UP);
      armraiseBEx.setTargetPosition(RAISE_BASKET_UP);
      armrotate.setPower(0.8);
      armraiseEx.setPower(0.95);
      armraiseBEx.setPower(0.95);
      armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      grabLeft.setPosition(LEFT_GRABBED_POS);
      grabRight.setPosition(RIGHT_GRABBED_POS);
    } else {
      armrotate.setTargetPosition(0);
      armraiseEx.setTargetPosition(0);
      armraiseBEx.setTargetPosition(0);
      armrotate.setPower(0.8);
      armraiseEx.setPower(0.95);
      armraiseBEx.setPower(0.95);
      armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      grabLeft.setPosition(LEFT_GRABBED_POS);
      grabRight.setPosition(RIGHT_GRABBED_POS);
    }
    
    if (opModeIsActive()) {
      // Put run blocks here.
      programTime.reset();
      while (opModeIsActive()) {
        canRot = (programTime.time() > (lastRot + 0.5));
        
        //    Arm Up/Down code
        if (gamepad2.right_bumper && canRot && !specMode) {
          grabSpec = false;
            if (!armUp) {
              armrotate.setTargetPosition(ROTATE_BASKET_UP);
              armraiseEx.setTargetPosition(RAISE_BASKET_UP);
              armraiseBEx.setTargetPosition(RAISE_BASKET_UP);
              armrotate.setPower(1);
              armraiseEx.setPower(1);
              armraiseBEx.setPower(1);
              armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armUp = true;
            } else {
              armrotate.setTargetPosition(ROTATE_GRAB_SUB);
              armraiseEx.setTargetPosition(0);
              armraiseBEx.setTargetPosition(0);
              armrotate.setPower(1);
              armraiseEx.setPower(1);
              armraiseBEx.setPower(1);
              armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armUp = false;
            }
            lastRot = programTime.time();
        }
        
        if (gamepad2.right_bumper && canRot && specMode) {
          grabSpec = false;
            if (!armUp) {
              armrotate.setTargetPosition(ROTATE_CLIP_PLACE);
              armraiseEx.setTargetPosition(RAISE_CLIP_PLACE);
              armraiseBEx.setTargetPosition(RAISE_CLIP_PLACE);
              armrotate.setPower(1);
              armraiseEx.setPower(1);
              armraiseBEx.setPower(1);
              armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armUp = true;
            } else {
              armrotate.setTargetPosition(ROTATE_GRAB_SUB);
              armraiseEx.setTargetPosition(0);
              armraiseBEx.setTargetPosition(0);
              armrotate.setPower(1);
              armraiseEx.setPower(1);
              armraiseBEx.setPower(1);
              armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
              armUp = false;
            }
            lastRot = programTime.time();
        }
        
        if (gamepad2.dpad_left && specMode) {
            armrotate.setTargetPosition(ROTATE_CLIP_GRAB);
            armraiseEx.setTargetPosition(RAISE_CLIP_GRAB);
            armraiseBEx.setTargetPosition(RAISE_CLIP_GRAB);
            armrotate.setPower(1);
            armraiseEx.setPower(1);
            armraiseBEx.setPower(1);
            armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armUp = false;
            grabSpec = true;
        }
        
        if (gamepad2.dpad_down) {
          armrotate.setTargetPosition(0);
          armraiseEx.setTargetPosition(0);
          armraiseBEx.setTargetPosition(0);
          armrotate.setPower(1);
          armraiseEx.setPower(1);
          armraiseBEx.setPower(1);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        if (gamepad2.dpad_up) {
          armrotate.setTargetPosition((int) (ROTATE_BASKET_UP / 1.5));
          armraiseEx.setTargetPosition(1000);
          armraiseBEx.setTargetPosition(1000);
          armrotate.setPower(1);
          armraiseEx.setPower(1);
          armraiseBEx.setPower(1);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        //    Map arm controls to joysticks and grabbers to triggers
        
        armextend.setPower(-gamepad2.right_stick_y);
        
        if (gamepad2.left_stick_y != 0) {
            armrotate.setTargetPosition((int) (armrotate.getCurrentPosition() + gamepad2.left_stick_y * 100));
            armrotate.setPower(1);
            armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        if (gamepad2.left_trigger > 0.4) {
            grabLeft.setPosition(LEFT_RELEASE_POS);
        } else {
            grabLeft.setPosition(LEFT_GRABBED_POS);
        }
        
        if (gamepad2.right_trigger > 0.4) {
            grabRight.setPosition(RIGHT_RELEASE_POS);
        } else {
            grabRight.setPosition(RIGHT_GRABBED_POS);
        }
          
        // change robot modes
        if (gamepad2.y) {
            specMode = !specMode;
        }
        
        // Drive base commands
        robotOrientation = imu.getRobotYawPitchRollAngles();
        double Yaw   = robotOrientation.getYaw(AngleUnit.DEGREES);
        double Pitch = robotOrientation.getPitch(AngleUnit.DEGREES);
        double Roll  = robotOrientation.getRoll(AngleUnit.DEGREES);
        
        cmdFwd = gamepad1.right_stick_y;
        cmdSide = -gamepad1.right_stick_x;
        cmdTurn = -gamepad1.left_stick_x;
        
        if (grabSpec) {
          driveCo = 0.5;
        } else if (armUp) {
          driveCo = 0.6;
        }
        
        if (armraiseEx.getCurrentPosition() >  2000) {
          cmdTurn *= 0.7;
        }
        
        frontright.setPower((cmdFwd - cmdSide - cmdTurn) * driveCo);
        frontleft.setPower((cmdFwd + cmdSide + cmdTurn) * driveCo);
        rearright.setPower((cmdFwd + cmdSide - cmdTurn) * driveCo);
        rearleft.setPower((cmdFwd - cmdSide + cmdTurn) * driveCo);
        
        // set eye positions
        if (cmdSide > 0 || cmdTurn > 0) {
          eyeLeft.setPosition(1);
          eyeRight.setPosition(1);
        } else if (cmdSide < 0 || cmdTurn < 0) {
          eyeLeft.setPosition(0.3);
          eyeRight.setPosition(0.3);
        } else if (gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
          eyeLeft.setPosition(1);
          eyeRight.setPosition(0.3);
        } else if (armraiseEx.isBusy()) {
          eyeLeft.setPosition(0.6);
          eyeRight.setPosition(0.6);
        } else if (!specMode) {
          eyeLeft.setPosition(0.3);
          eyeRight.setPosition(1);
        } else {
          eyeLeft.setPosition(1);
          eyeRight.setPosition(0.3);
        }
        
        // check if lift motors are fighting
        if (Math.abs(armraiseEx.getCurrentPosition() - armraiseBEx.getCurrentPosition()) > 500) {
          motorsFighting = true;
          timesFought ++;
        } else {
          motorsFighting = false;
        }
        
        // reset arm at bottom and turn down/off motors accordingly
        
        //if (magSwitch.isPressed() && armraiseEx.isBusy()) {
        //  armraiseEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  armraiseBEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //}
        if (!armraiseEx.isBusy()) {
          if (armraiseEx.getCurrentPosition() < 1000) {
            armraiseEx.setPower(0);
            armraiseBEx.setPower(0);
          } else {
            armraiseEx.setPower(0);
            armraiseBEx.setPower(0.3);
          }
        }
        
        telemetry.addData("Spec Mode:", specMode);
        telemetry.addLine("-----------------");
        telemetry.addData("motorsFighting", motorsFighting);
        telemetry.addData("times fought", timesFought);
        telemetry.addData("Pitch", Pitch);
        telemetry.addData("Roll", Roll);
        telemetry.addData("Arm busy", armraiseEx.isBusy());
        telemetry.addData("arm raise A pow", armraiseEx.getPower());
        telemetry.addData("arm raise B pow", armraiseBEx.getPower());
        telemetry.addData("arm raise A pos", armraiseEx.getCurrentPosition());
        telemetry.addData("arm raise B pos", armraiseBEx.getCurrentPosition());
        telemetry.addData("time", programTime.time());
        telemetry.update();
      }
    }
  }
}
