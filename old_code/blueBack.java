// package org.firstinspires.ftc.teamcode.obsolete_past_vision;
// 
// import android.util.Size;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import java.util.List;
// import org.firstinspires.ftc.robotcore.external.JavaUtil;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// ///import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
// import org.firstinspires.ftc.vision.VisionPortal;
// ///import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// 
// @Autonomous(name = "blueBack")
// public class blueBack extends LinearOpMode {
// 
//   private DcMotor frontleft;
//   private DcMotor rearleft;
//   private DcMotor frontright;
//   private DcMotor rearright;
// 
//   boolean USE_WEBCAM;
//   TfodProcessor myTfodProcessor;
// 
//   /**
//    * This function is executed when this OpMode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     int blueVsRed;
// 
//     frontleft = hardwareMap.get(DcMotor.class, "frontleft");
//     rearleft = hardwareMap.get(DcMotor.class, "rearleft");
//     frontright = hardwareMap.get(DcMotor.class, "frontright");
//     rearright = hardwareMap.get(DcMotor.class, "rearright");
// 
//     // Put initialization blocks here.
//     USE_WEBCAM = true;
//     blueVsRed = 1;
//     // Blue = 1, Red = -1
//     if (blueVsRed == 1) {
//         commentTelemetry("Program status", "Currently in BLUE program");
//     } else if (blueVsRed == -1) {
//         commentTelemetry("Program status", "Currently in RED program");
//     }
//     initTfod();
//     // Set the number of pixels to obscure on the left, top,
//     // right, and bottom edges of each image passed to the
//     // TensorFlow object detector. The size of the images are not
//     // changed, but the pixels in the margins are colored black.
//     myTfodProcessor.setClippingMargins(80, 90, 80, 100);
//     frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
//     rearleft.setDirection(DcMotorSimple.Direction.REVERSE);
//     frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     waitForStart();
//     // Put run blocks here.
//     commentTelemetry("Program status", "Beginning program");
//     sleep(500);
//     commentTelemetry("Program status", "Driving forward");
//     drive(20, 0, 0, 0.3);
//     commentTelemetry("Program status", "Checking for pixel");
//     if (checkForPixel()) {
//       // place pixel
//       commentTelemetry("Pixel position", "position 2");
//       drive(0, 0, blueVsRed * -90, 0.4);
//       drive(34, 0, 0, 0.4);
//       if (blueVsRed == 1) {
//           commentTelemetry("Program status", "Placing pixel");
//           // drop off pixel at backdrop
//           drive(0, -14, 0, 0.4);
//         } else if (blueVsRed == -1) {
//           drive(0, -12, 0, 0.4);
//           commentTelemetry("Program status", "Placing pixel");
//           // drop off pixel at backdrop
//           drive(0, 26, 0, 0.4);
//         }
//       drive(5, 0, 0, 0.4); 
//       commentTelemetry("Program status", "Autonomous completed");
//     } else {
//       commentTelemetry("Pixel position", "Pixel not in position 2");
//       drive(0, 0, -35, 0.4);
//       if (checkForPixel()) {
//         commentTelemetry("Pixel position", "Pixel in position 1");
//         // drop off purple pixel on middle spike
//         drive(0, 0, 35, 0.4);
//         drive(0, 0, blueVsRed * -90, 0.4);
//         drive(34, 0, 0, 0.4);
//         if (blueVsRed == 1) {
//           // drop off pixel at backdrop
//           commentTelemetry("Program status", "Placing pixel");
//           drive(0, -14, 0, 0.4);
//         } else if (blueVsRed == -1) {
//           drive(0, 12, 0, 0.4);
//           // drop off pixel at backdrop
//           commentTelemetry("Program status", "Placing pixel");
//           drive(0, 26, 0, 0.4);
//         }
//         drive(5, 0, 0, 0.4);
//       } else {
//         commentTelemetry("Pixel position", "Pixel not in position 1 or 2, assumed position 3");
//         if (blueVsRed == 1) {
//           commentTelemetry("Program status", "Turning towards backdrop");
//           drive(0, 0, -90 + 35, 0.4);
//         } else if (blueVsRed == -1) {
//           commentTelemetry("Program status", "Turning towards backdrop");
//           drive(0, 0, 90 + 35, 0.4);
//         }
//         drive(34, 0, 0, 0.4);
//         if (blueVsRed == 1) {
//           commentTelemetry("Program status", "Moving to position");
//           drive(0, 6*2, 0, 0.4);
//           commentTelemetry("Program status", "Moving to wall");
//           drive(0, -6*2 - 14, 0, 0.4);
//         } else if (blueVsRed == -1) {
//           commentTelemetry("Program status", "Placing pixel");
//           drive(0, 14, 0, 0.4);
//         }
//         drive(5, 0, 0, 0.4);
//       }
//     }
//     commentTelemetry("Program status", "Autonomous program completed");
//   }
// 
//   /**
//    * Initialize TensorFlow Object Detection.
//    */
//   private void initTfod() {
//     TfodProcessor.Builder myTfodProcessorBuilder;
//     VisionPortal.Builder myVisionPortalBuilder;
//     VisionPortal myVisionPortal;
// 
//     // First, create a TfodProcessor.Builder.
//     myTfodProcessorBuilder = new TfodProcessor.Builder();
//     // Create a TfodProcessor by calling build.
//     myTfodProcessor = myTfodProcessorBuilder.build();
//     // Next, create a VisionPortal.Builder and set attributes related to the camera.
//     myVisionPortalBuilder = new VisionPortal.Builder();
//     if (USE_WEBCAM) {
//       // Use a webcam.
//       myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
//     } else {
//       // Use the device's back camera.
//       myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
//     }
//     // Add myTfodProcessor to the VisionPortal.Builder.
//     myVisionPortalBuilder.addProcessor(myTfodProcessor);
//     // Create a VisionPortal by calling build.
//     myVisionPortal = myVisionPortalBuilder.build();
//     // Set the camera resolution.
//     myVisionPortalBuilder.setCameraResolution(new Size(960, 720));
//   }
// 
//   /**
//    * Describe this function...
//    */
//   private void drive(double fwd_bck, double right_left, double cw_ccw, double power) {
//     double frontLeftDistance;
//     double rearLeftDistance;
//     double frontRightDistance;
//     double rearRightDistance;
//     double maxDistance;
//     double frontLeftPower;
//     double frontRightPower;
//     double rearLeftPower;
//     double rearRightPower;
// 
//     frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     rearleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     rearright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//     if (fwd_bck == 0) {
//       fwd_bck = 0;
//     }
//     if (right_left == 0) {
//       right_left = 0;
//     }
//     if (cw_ccw == 0) {
//       cw_ccw = 0;
//     }
//     if (power == 0) {
//       power = 0.7;
//     }
//     fwd_bck = fwd_bck * 57.075;
//     right_left = right_left * 59.88;
//     cw_ccw = cw_ccw * 10.042;
//     frontLeftDistance = fwd_bck + right_left + cw_ccw;
//     rearLeftDistance = fwd_bck + -right_left + cw_ccw;
//     frontRightDistance = fwd_bck + -right_left + -cw_ccw;
//     rearRightDistance = fwd_bck + right_left + -cw_ccw;
//     maxDistance = JavaUtil.maxOfList(JavaUtil.createListWith(frontLeftDistance, frontRightDistance, rearLeftDistance, rearRightDistance));
//     frontLeftPower = frontLeftDistance / maxDistance;
//     frontRightPower = frontRightDistance / maxDistance;
//     rearLeftPower = rearLeftDistance / maxDistance;
//     rearRightPower = rearRightDistance / maxDistance;
//     frontleft.setTargetPosition((int) frontLeftDistance);
//     frontright.setTargetPosition((int) frontRightDistance);
//     rearleft.setTargetPosition((int) rearLeftDistance);
//     rearright.setTargetPosition((int) rearRightDistance);
//     frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     rearleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     rearright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//     frontleft.setPower(Math.abs(frontLeftPower * power));
//     frontright.setPower(Math.abs(frontRightPower * power));
//     rearleft.setPower(Math.abs(rearLeftPower * power));
//     rearright.setPower(Math.abs(rearRightPower * power));
//     while (frontleft.isBusy() || frontright.isBusy() || rearleft.isBusy() || rearright.isBusy()) {
//       // Disable telemetry for competition as it slows the loop down
//       if (true) {
//         sleep(10);
//       }
//     }
//     frontleft.setPower(0);
//     frontright.setPower(0);
//     rearleft.setPower(0);
//     rearright.setPower(0);
//   }
// 
//   /**
//    * Describe this function...
//    */
//   private boolean checkForPixel() {
//     boolean pixelDetected;
//     List<Recognition> myTfodRecognitions;
//     int count = 0;
//     do {
//       // Get a list of recognitions from TFOD.
//       myTfodRecognitions = myTfodProcessor.getRecognitions();
//       sleep(10);
//       count += 1;
//     } while (JavaUtil.listLength(myTfodRecognitions) == 0 && count < 70);
//     
//     if (JavaUtil.listLength(myTfodRecognitions) > 0) {
//       pixelDetected = true;
//     } else {
//       pixelDetected = false;
//     }
//     return pixelDetected;
//   }
//   private void commentTelemetry (String description, String comment) {
//     telemetry.addData(description, comment);
//     telemetry.update();
//     //sleep(0);
//   }
// }
// 
// 
