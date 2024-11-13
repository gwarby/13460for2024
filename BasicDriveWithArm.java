package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "BasicDriveWithArm")
public class BasicDriveWithArm extends LinearOpMode {

  IntegratingGyroscope gyro;
  NavxMicroNavigationSensor navxMicro;

  private DcMotor frontright;
  private DcMotor rearleft;
  private DcMotor frontleft;
  private DcMotor rearright;
  private DcMotor armrotate;
  private DcMotor armraise;
  private Servo grabber;
  private CRServo armextend;

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
    armraise = hardwareMap.get(DcMotor.class, "armraise");
    grabber = hardwareMap.get(Servo.class, "grabber");
    
    ElapsedTime programTime = new ElapsedTime();

    // Put initialization blocks here.
    rearleft.setDirection(DcMotorSimple.Direction.REVERSE);
    rearright.setDirection(DcMotorSimple.Direction.REVERSE);
    armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armraise.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armrotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armraise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    int extendLimitTicks;
    int rotateLimitTicks;
    boolean grabberOpen = true, armUp = false, cmdRaise;
    int rotatePosTicks = 0;
    int raisePosTicks = 0;
    double lastGrabbed = 0, lastRot = 0;
    
    grabber.setPosition(0.25);
    
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
    
    // top raise softlimit - around 5000 ticks
    // top rotate softlimit - around 450 ticks
    // bottom both soft limit - 0 ticks
    
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      programTime.reset();
      while (opModeIsActive()) {
        cmdRaise = gamepad2.right_stick_y != 0;
        boolean canGrab = (programTime.time() > (lastGrabbed + 0.4));
        boolean canRot = (programTime.time() > (lastRot + 0.3));
        telemetry.addData("can rotate", canRot);
        telemetry.addData("gamepad2 b", gamepad2.b);
        // Put loop blocks here.
        frontleft.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        frontright.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        rearleft.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        rearright.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        armextend.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        armraise.setPower(-gamepad2.right_stick_y);
        
        
        // if the dpad_down is held, slow the rotation
        if (gamepad2.dpad_down){
          if (gamepad2.left_stick_y < 0) {
            armrotate.setPower(-gamepad2.left_stick_y * 0.02); // set the power at an extremely low positive value to allow gravity to let it fall
          } else {
            armrotate.setPower(-gamepad2.left_stick_y * 0.1);
          }
          
        } else {
          if (gamepad2.left_stick_y < 0) {
            armrotate.setPower(-gamepad2.left_stick_y * 0.005);
          } else {
            armrotate.setPower(-gamepad2.left_stick_y * 0.9);
          }
        }
        
        // gamepad b puts the arm at either a slightly higher than middle posistion, or the bottom position
        if (gamepad2.b && canRot) {
          telemetry.addData("gamepad2.b Pressed", "True");
          armrotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
        if (armUp) {
            armrotate.setTargetPosition(0);
            armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armrotate.setPower(0.4);
            armUp = false;
        } else {
            armrotate.setTargetPosition(470);
            armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armrotate.setPower(0.6);
            armUp = true;
        }
    
    lastRot = programTime.time();
    telemetry.addData("armUp Toggled", armUp);
    telemetry.update();
}

        telemetry.addData("arm rot target", armrotate.getTargetPosition());
        
        //armrotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armrotate.setPower(gamepad2.left_stick_y);
        
        if (cmdRaise) {
          armraise.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          armraise.setPower(-gamepad2.right_stick_y);
          raisePosTicks = armraise.getCurrentPosition();
          telemetry.addData("Inside arm raise cmd loop, raise power", -gamepad2.right_stick_y);
        } else {
          armraise.setTargetPosition(raisePosTicks);
          armraise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armraise.setPower(0.6);
        }
        
        // gamepad a opens and closes the grabber
        if (gamepad2.a && canGrab) {
          if (grabberOpen) {
            grabber.setPosition(0.0);
            grabberOpen = false;
          } else {
            grabber.setPosition(0.35);
            grabberOpen = true;
          }
          lastGrabbed = programTime.time();
        }
        
        rotateLimitTicks = armrotate.getCurrentPosition();
        telemetry.addData("armraise ticks", armraise.getCurrentPosition());
        telemetry.addData("armrotate Ticks", rotateLimitTicks);
        telemetry.addData("time", programTime.time());
        telemetry.update();
      }
    }
  }
}
