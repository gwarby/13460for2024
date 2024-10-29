package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;


@TeleOp(name = "vis_eocv_test")
public class vis_eocv_test extends LinearOpMode 
{
  OpenCvWebcam webcam;
 /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() 
  {
    double leftScore, midScore, rightScore;
    int camMonViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), camMonViewId); 

    // specify image processing pipeline to invoke
    // upon receipt of a frame from the camera
    FindPropPipeline findPropPL = new FindPropPipeline();
    webcam.setPipeline(findPropPL);

    // open connection to webcam asynchronously
    webcam.setMillisecondsPermissionTimeout(5000);

    // optional: use GPU acceleration
    webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        // tell cam to start streaming
        // note: must use resolution supported by cam
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode)
      {
        // called if camera can't be opened
      }
    });

    telemetry.addLine("Waiting for start");
    telemetry.update();

    // wait for user to press start on Driver Station
    waitForStart();

    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.

        // do some telemetry
        telemetry.addData("frames", webcam.getFrameCount());
        telemetry.addData("fps", String.format("%.2f",webcam.getFps()));
        telemetry.addData("theoretical max fps", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("","");
        telemetry.addData("PROP LOCATION: ", findPropPL.propLocation);
        telemetry.addData("","");
        telemetry.addData("left Chroma", findPropPL.leftChroma);
        telemetry.addData("mid Chroma", findPropPL.midChroma);
        telemetry.addData("right Chroma", findPropPL.rightChroma);
        telemetry.update();

        // don't do this in a real op mode, but for this cam test, chill on the CPU cycles
        sleep(280);
      }
    }
  }
  
  class FindPropPipeline extends OpenCvPipeline
  {
    // Attributes accessed by caller
    double leftChroma = 0.0, midChroma = 0.0, rightChroma = 0.0;
    String propLocation;
    
    private Mat YCrCb = new Mat();   // Mat for converting color system
    private Mat ChromaMat = new Mat();   // Mat for extracting desired Chroma channel
    private Mat leftSubmat, midSubmat, rightSubmat;  // Mats for left, mid, right sub windows
    
    // Blue Front region subframe coords & geometry:
    // 0,0 is the upper left point, +y goes down, +x goes right
    // (240 vert, 320 horiz)
    final Point LeftRegion_TopLeft_Point = new Point(0,46);
    final Point LeftRegion_BottomRight_Point = new Point(85,140);
    final Rect LeftRegion_Rect = new Rect(LeftRegion_TopLeft_Point, LeftRegion_BottomRight_Point);

    final Point MiddleRegion_TopLeft_Point = new Point(106,28);
    final Point MiddleRegion_BottomRight_Point = new Point(210,105);
    final Rect MiddleRegion_Rect = new Rect(MiddleRegion_TopLeft_Point, MiddleRegion_BottomRight_Point);

    final Point RightRegion_TopLeft_Point = new Point(245,46);
    final Point RightRegion_BottomRight_Point = new Point(319,140);
    final Rect RightRegion_Rect = new Rect(RightRegion_TopLeft_Point, RightRegion_BottomRight_Point);
    
    @Override
    public void init(Mat firstFrame)
    {
      // see the SkystoneDeterminationExample.java within
      //    github.com/OpenFTC/EasyOpenCV
    }

    @Override
    public Mat processFrame(Mat input)
    {
      // extracting chroma channel
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
      Core.extractChannel(YCrCb, ChromaMat, 2);  // channel 1: red,  channel 2: blue
      
      // original approach in RGB
      leftSubmat = ChromaMat.submat(LeftRegion_Rect);
      midSubmat = ChromaMat.submat(MiddleRegion_Rect);
      rightSubmat = ChromaMat.submat(RightRegion_Rect);

      // get the average Chroma value for each submat
      leftChroma = Core.mean(leftSubmat).val[0]; // works because we are operating on just Cb or Cr subchan 
      midChroma = Core.mean(midSubmat).val[0];
      rightChroma = Core.mean(rightSubmat).val[0];

      // compare the scores in order to report the position
      propLocation = "MIDDLE";
      if (rightChroma > midChroma)
      {
        propLocation = "RIGHT";
      }
      if (leftChroma > midChroma && leftChroma > rightChroma)
      {
        propLocation = "LEFT";
      }

      // draw rectangles for debug / alignment purposes
      Imgproc.rectangle(ChromaMat, // Mat on which to draw
        LeftRegion_TopLeft_Point, LeftRegion_BottomRight_Point, // topleft & botright pts
        new Scalar(0,0,0), 2); // color & width in pixels
      
      Imgproc.rectangle(ChromaMat,
        MiddleRegion_TopLeft_Point, MiddleRegion_BottomRight_Point,
        new Scalar(0,0,0), 2);
      
      Imgproc.rectangle(ChromaMat,
        RightRegion_TopLeft_Point, RightRegion_BottomRight_Point,
        new Scalar(0,0,0), 2);
      
      return ChromaMat;  // return the chroma blue channel w/ rectangles overlaid
    }
  }
}
