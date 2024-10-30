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

@TeleOp(name = "BasicDriveWithArm (Blocks to Java)")
public class BasicDriveWithArm extends LinearOpMode {

  IntegratingGyroscope gyro;
  NavxMicroNavigationSensor navxMicro;

  private DcMotor frontright;
  private DcMotor rearleft;
  private DcMotor frontleft;
  private DcMotor rearright;
  private DcMotor armraise;
  private Servo armrotate;
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
    armraise = hardwareMap.get(DcMotor.class, "armraise");
    armrotate = hardwareMap.get(Servo.class, "armrotate");
    armextend = hardwareMap.get(CRServo.class, "armextend");

    // Put initialization blocks here.
    rearright.setDirection(DcMotorSimple.Direction.REVERSE);
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    armraise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armraise.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armraise.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    int raiseLimitTicks;
    int extendLimitTicks;
    int rotateLimitPosition;
    double rotateLimitFraction;
    double extendLimitFraction;
    
    
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
      while (opModeIsActive()) {
        
        // Put loop blocks here.
        frontleft.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        frontright.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        rearleft.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        rearright.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        armraise.setPower(-gamepad2.right_stick_y);
        armrotate.setPosition(armrotate.getPosition() + gamepad2.left_stick_y/100);
        armextend.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        
        raiseLimitTicks = armraise.getCurrentPosition();
        // TODO: refactor code:
        //  1st: match config of robot motors & servos - build team was in flux
        //  2nd: ticks are an int, but servo position is a double rep'ing fractional position
        extendLimitFraction = armextend.getController().getServoPosition(armextend.getPortNumber());
        rotateLimitFraction = armrotate.getPosition();
        
        telemetry.addData("armraise Ticks", raiseLimitTicks);
        telemetry.addData("armrotate Position", rotateLimitFraction);
        telemetry.addData("armextend Ticks", extendLimitFraction);
        telemetry.update();
      }
    }
  }
}
