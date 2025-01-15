package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
  private CRServo armextend, grabber;
  private TouchSensor magSwitch;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
   

    
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
    grabber = hardwareMap.get(CRServo.class, "grabber");
    magSwitch = hardwareMap.get(TouchSensor.class, "magSwitch");
 
   
    
    ElapsedTime programTime = new ElapsedTime();

    // Put initialization blocks here.
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontright.setDirection(DcMotorSimple.Direction.REVERSE);
    armraiseBEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    armraiseEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    int extendLimitTicks;
    int rotateLimitTicks;
    boolean armUp = false, useHold = false;
    int rotatePosTicks = 0;
    int raisePosTicks = 0;
    double lastGrabbed = 0, lastRot = 0;
    
    //grabber.setPosition(0.25);
    
    /*@Override public void runOpMode() throws InterruptedException {
        // Get a reference to a Modern Robotics GyroSensor object. We use several interfaces
        // on this object to illustrate which interfaces support which functionality.
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "godIMU");
        gyro = (IntegratingGyroscope)navxMicro;
        // If you're only interested int the IntegratingGyroscope interface, the following will suffice.
        // gyro = hardwareMap.get(IntegratingGyroscope.class, "navx");

        // The gyro automatically starts calibrating. This takes a few seconds.
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        // Wait until the gyro calibration is complete
        timer.reset();
        while (navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();
        telemetry.log().clear();
    */
    
    waitForStart();
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
        frontleft.setPower(Math.pow(-gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x, 2/3));
        frontright.setPower(Math.pow(gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x, 2/3));
        rearleft.setPower(Math.pow(gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x, 2/3));
        rearright.setPower(Math.pow(-gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x, 2/3));
      }
    }
    if (opModeIsActive()) {
      // Put run blocks here.
      programTime.reset();
      while (opModeIsActive()) {
        boolean canRotate = (programTime.time() > (lastRot + 0.25));
        boolean canGrab = (programTime.time() > (lastGrabbed + 0.25));
        // Put loop blocks here.
        frontleft.setPower(Math.pow(-gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x, 1));
        frontright.setPower(Math.pow(gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x, 1));
        rearleft.setPower(Math.pow(gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x, 1));
        rearright.setPower(Math.pow(-gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x, 1));
        armextend.setPower(-gamepad2.right_stick_y);
        
        if (gamepad2.right_bumper && canRotate) {
          useHold = true;
          armrotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          armraiseEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
          if (armUp) {
            armrotate.setTargetPosition(0);
            armraiseEx.setTargetPosition(0);
            armraiseBEx.setTargetPosition(0);
            armrotate.setPower(1);
            armraiseEx.setPower(1);
            armraiseBEx.setPower(1);
            armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armUp = false;
            
          } else {
            armrotate.setTargetPosition(-5500);
            armraiseEx.setTargetPosition(4400);
            armraiseBEx.setTargetPosition(-4400);
            armrotate.setPower(1);
            armraiseEx.setPower(1);
            armraiseBEx.setPower(1);
            armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armraiseEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armraiseBEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armUp = true;
          }
          lastRot = programTime.time();
        }
        
        if (gamepad2.right_trigger != 0) {
          grabber.setPower(-gamepad2.right_trigger);
          useHold = true;
        } else if (gamepad2.left_trigger != 0) {
          grabber.setPower(gamepad2.left_trigger);
          useHold = false;
        } else if (useHold) {
          grabber.setPower(-0.08);
        } else {
          grabber.setPower(0);
        }
        
        if (gamepad2.dpad_left) {
          armrotate.setTargetPosition(-2000);
          armrotate.setPower(1);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.dpad_right) {
          armrotate.setTargetPosition(-5500);
          armrotate.setPower(1);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.dpad_down) {
          armrotate.setTargetPosition(armrotate.getCurrentPosition() + 50);
          armrotate.setPower(1);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.dpad_up) {
          armrotate.setTargetPosition(armrotate.getCurrentPosition() - 50);
          armrotate.setPower(1);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        
        rotateLimitTicks = armrotate.getCurrentPosition();
        telemetry.addData("armraiseEx ticks", armraiseEx.getCurrentPosition());
        telemetry.addData("armrotate Ticks", rotateLimitTicks);
        telemetry.addData("time", programTime.time());
        telemetry.update();
      }
    }
  }
}
