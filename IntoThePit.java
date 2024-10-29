package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
///import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
///import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "IntoThePit")
public class IntoThePit extends LinearOpMode {

  IntegratingGyroscope gyro;
  NavxMicroNavigationSensor navxMicro;
  private DcMotor frontleft;
  private DcMotor rearleft;
  private DcMotor frontright;
  private DcMotor rearright;



  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {

    gyro = hardwareMap.get(IntegratingGyroscope.class, "godIMU");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    rearleft = hardwareMap.get(DcMotor.class, "rearleft");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    rearright = hardwareMap.get(DcMotor.class, "rearright");

    // Put initialization blocks here.
    

    Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
    //double botHeading = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    double botHeading = angles.firstAngle;
    commentTelemetry("Bot heading", String.format("%.1f", botHeading));

    // Set the number of pixels to obscure on the left, top,
    // right, and bottom edges of each image passed to the
    // TensorFlow object detector. The size of the images are not
    // changed, but the pixels in the margins are colored black.
    frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
    rearleft.setDirection(DcMotorSimple.Direction.REVERSE);
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    waitForStart();

    drive(10, 10, 0, 0);
    drive(-10, 10, 0, 0);
    drive(-10, -10, 0, 0);
    drive(10, -10, 0, 0);
    sleep(3000);
    drive(0, 0, 45, 0);
    drive(20, 0, 0, 0);
    drive(0, 0, -90, 0);
    drive(20, 0, 0, 0);
    drive(0, 0, -90, 0);
    drive(20, 0, 0, 0);
    drive(0, 0, -90, 0);
    drive(20, 0, 0, 0);
    drive(0, 0, -90, 0);

    commentTelemetry("Program status", "Autonomous program completed");
  }


  /**
   * Describe this function...
   */
  private void drive(double fwd_bck, double right_left, double cw_ccw, double power) {
    double frontLeftDistance;
    double rearLeftDistance;
    double frontRightDistance;
    double rearRightDistance;
    double maxDistance;
    double frontLeftPower;
    double frontRightPower;
    double rearLeftPower;
    double rearRightPower;

    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    if (fwd_bck == 0) {
      fwd_bck = 0;
    }
    if (right_left == 0) {
      right_left = 0;
    }
    if (cw_ccw == 0) {
      cw_ccw = 0;
    }
    if (power == 0) {
      power = 0.7;
    }
    fwd_bck = fwd_bck * 57.075;
    right_left = right_left * 59.88;
    cw_ccw = cw_ccw * 10.042;
    frontLeftDistance = fwd_bck + right_left + cw_ccw;
    rearLeftDistance = fwd_bck + -right_left + cw_ccw;
    frontRightDistance = fwd_bck + -right_left + -cw_ccw;
    rearRightDistance = fwd_bck + right_left + -cw_ccw;
    maxDistance = JavaUtil.maxOfList(JavaUtil.createListWith(frontLeftDistance, frontRightDistance, rearLeftDistance, rearRightDistance));
    frontLeftPower = frontLeftDistance / maxDistance;
    frontRightPower = frontRightDistance / maxDistance;
    rearLeftPower = rearLeftDistance / maxDistance;
    rearRightPower = rearRightDistance / maxDistance;
    frontleft.setTargetPosition((int) frontLeftDistance);
    frontright.setTargetPosition((int) frontRightDistance);
    rearleft.setTargetPosition((int) rearLeftDistance);
    rearright.setTargetPosition((int) rearRightDistance);
    frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontleft.setPower(Math.abs(frontLeftPower * power));
    frontright.setPower(Math.abs(frontRightPower * power));
    rearleft.setPower(Math.abs(rearLeftPower * power));
    rearright.setPower(Math.abs(rearRightPower * power));
    while (frontleft.isBusy() || frontright.isBusy() || rearleft.isBusy() || rearright.isBusy()) {
      // Disable telemetry for competition as it slows the loop down
      if (true) {
        sleep(10);
      }
    }
    frontleft.setPower(0);
    frontright.setPower(0);
    rearleft.setPower(0);
    rearright.setPower(0);
    printCurrentHeading();
  }

  /**
   * Describe this function...
   */
  private void printCurrentHeading() {
    Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double botHeading = angles.firstAngle;
    commentTelemetry("Bot heading", String.format("%.1f", botHeading));
  }

  private void commentTelemetry (String description, String comment) {
    telemetry.addData(description, comment);
    telemetry.update();
    //sleep(0);
  }
}
