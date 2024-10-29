package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name = "AutoNewIMUTesting.java")

public class AutoNewIMUTesting extends LinearOpMode {
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    
    private DcMotor frontright;
    private DcMotor rearleft;
    private DcMotor frontleft;
    private DcMotor rearright;
    private DcMotor armextend;
    private DcMotor armrotate;
    private Blinker control_Hub;
    private HardwareDevice webcam;
      /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
      
    gyro = hardwareMap.get(IntegratingGyroscope.class, "godIMU");

    // todo: write your code here
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    rearright = hardwareMap.get(DcMotor.class, "rearright");
    //armextend = hardwareMap.get(DcMotor.class, "armextend");
    //armrotate = hardwareMap.get(DcMotor.class, "armrotate");

    // Put initialization blocks here.
    rearleft.setDirection(DcMotor.Direction.REVERSE);
    frontleft.setDirection(DcMotor.Direction.REVERSE);
    //rearright.setDirection(DcMotor.Direction.REVERSE);
    //frontright.setDirection(DcMotor.Direction.REVERSE);

    //armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //armextend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //armrotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //armrotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    int extendLimitTicks;
    int rotateLimitTicks;
    
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
        
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        //double botHeading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        double botHeading = angles.firstAngle;
        double driverCmd_Right = gamepad1.right_stick_x;
        double driverCmd_Fwd = -gamepad1.right_stick_y;
        double driverCmd_Rotate = gamepad1.left_stick_x;
        
      //telemetry.addLine().addData("cmd_fwd", String.format("%.1f", driverCmd_Fwd));
      //telemetry.addLine().addData("cmd_rgt", String.format("%.1f", driverCmd_Right));
      //telemetry.addLine().addData("cmd_rot", String.format("%.1f", driverCmd_Rotate));


        double robotCmd_Right = driverCmd_Right * Math.cos(-botHeading) - driverCmd_Fwd * Math.sin(-botHeading);
        double robotCmd_Fwd = driverCmd_Right * Math.sin(-botHeading) + driverCmd_Fwd * Math.cos(-botHeading);
        double robotCmd_Rotate = driverCmd_Rotate;

      telemetry.addLine().addData("robo_fwd", String.format("%.1f", robotCmd_Fwd));
      telemetry.addLine().addData("robo_rgt", String.format("%.1f", robotCmd_Right));
      telemetry.addLine().addData("robo_rot", String.format("%.1f", robotCmd_Rotate));
      telemetry.addLine().addData("botHeading", String.format("%.1f", botHeading));

        double MAX_DRIVE_MOTOR_POWER = 0.8;
        
        // Denominator is the largest motor power (absolute value) or MAX_DRIVE_MOTOR_POWER
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-MAX_DRIVE_MOTOR_POWER, MAX_DRIVE_MOTOR_POWER]
        double denominator = Math.max(Math.abs(robotCmd_Fwd) + Math.abs(robotCmd_Right) + Math.abs(robotCmd_Rotate), 1.0);

        double frontLeftPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd + robotCmd_Right + robotCmd_Rotate) / denominator;
        double backLeftPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd - robotCmd_Right + robotCmd_Rotate) / denominator;
        double frontRightPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd - robotCmd_Right - robotCmd_Rotate) / denominator;
        double backRightPower = MAX_DRIVE_MOTOR_POWER * (robotCmd_Fwd + robotCmd_Right - robotCmd_Rotate) / denominator;
        
        frontleft.setPower(frontLeftPower);
        frontright.setPower(frontRightPower);
        rearleft.setPower(backLeftPower);
        rearright.setPower(backRightPower);
        
        //frontleft.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        //frontright.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        //rearleft.setPower(gamepad1.right_stick_x + gamepad1.right_stick_y + -gamepad1.left_stick_x);
        //rearright.setPower(-gamepad1.right_stick_x + gamepad1.right_stick_y + gamepad1.left_stick_x);
        //armextend.setPower(-gamepad2.right_stick_y);
        //armrotate.setPower(-gamepad2.left_stick_y);
        //extendLimitTicks = armextend.getCurrentPosition();
        //rotateLimitTicks = armrotate.getCurrentPosition();
        //telemetry.addData("armextend Ticks", extendLimitTicks);
        //telemetry.addData("armrotate Ticks", rotateLimitTicks);
        
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        keepAnglePositive(angles);
        telemetry.update();
      }
    }
  }
    private void keepAnglePositive(Orientation angles) {
    if (-AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) < 0) {
      telemetry.addLine()
        .addData("heading", String.format("%.1f", -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)) + 360));
    } else {
      telemetry.addLine()
        .addData("heading", String.format("%.1f", -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle))));
    }
  }
}