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
    armrotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armraise.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armrotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    armraise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    int extendLimitTicks;
    int rotateLimitTicks;
    boolean grabberOpen = true, cmdRotate, cmdRaise;
    int rotatePosTicks = 0;
    int raisePosTicks = 0;
    double lastGrabbed = 0;
    
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
    
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      programTime.reset();
      while (opModeIsActive()) {
        cmdRotate = gamepad2.left_stick_y != 0;
        cmdRaise = gamepad2.right_stick_y != 0;
        boolean canGrab = (programTime.time() > (lastGrabbed + 0.1));
        // Put loop blocks here.
        frontleft.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        frontright.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        rearleft.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        rearright.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        armextend.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        armraise.setPower(-gamepad2.right_stick_y);
        if (cmdRotate) {
          armrotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          if (gamepad2.left_stick_y < 0) {
            armrotate.setPower(gamepad2.left_stick_y * 0.07);
          } else {
            armrotate.setPower(gamepad2.left_stick_y * 0.55);
          }
          rotatePosTicks = armrotate.getCurrentPosition();
        } else {
          armrotate.setTargetPosition(rotatePosTicks);
          armrotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          armrotate.setPower(0.6);
        }
        
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
