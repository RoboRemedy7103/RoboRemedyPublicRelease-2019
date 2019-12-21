/* TrackTape.java - a NetworkTable Server to deliver
                    x and y coordinate result from vision processing */

import edu.wpi.first.networktables.*;
import edu.wpi.cscore.*;
import org.opencv.core.*;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.cameraserver.CameraServer;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.io.IOException;

public class TrackTape {
  public static void main(String[] args) {
    int iCount = 0;
    NetworkTableInstance tableInstance;
    NetworkTable table;
    String json;
    tableInstance = NetworkTableInstance.getDefault();
    table = tableInstance.getTable("vision");
    tableInstance.setUpdateRate(1);
    tableInstance.startClientTeam(7103);

    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture("Pi Camera", 0);
    System.out.println("Got the camera");

    try {
      json = new String(Files.readAllBytes(Paths.get("camera.json")));
    } catch (IOException e) {
      e.printStackTrace();
      return;
    }

    camera.setConfigJson(json);
    VisionThread visionThread = new VisionThread(camera,
            new MyPipeline(), pipeline -> {
        // do something with pipeline results
        table.getEntry("Distance").setDouble(pipeline.distance);
        table.getEntry("Angle").setDouble(pipeline.angle);
        table.getEntry("Val").setNumber(pipeline.val);
        tableInstance.flush();
        pipeline.visionMode = table.getEntry("Mode").getString("none");
      });
    visionThread.start();

    System.out.println("Starting the loop");
    long start = System.nanoTime() / 1000000;
    long last = start;
    while(true) {
      try {
        Thread.sleep(1000);
      } catch(Exception e) {
      }
    }
  }

  public static class MyPipeline implements VisionPipeline {
    public int val = 0;
    public long last = 0;
    TapeGrip vision = new TapeGrip();
    public double distance = -2;
    public double  angle = 0;
    public String visionMode = "init";
    private int badCount = 0;
  
    public void MyPipeline() {
      last = System.nanoTime() / 1000000;
    }
  
    @Override
    public void process(Mat mat) {
      double before = System.nanoTime() / 1000000;
      vision.process(mat);
      double after = System.nanoTime() / 1000000;

      val += 1;
  
      int contourCount = vision.filterContoursOutput().size();
      if (contourCount == 2) {
        MatOfPoint point1 = vision.filterContoursOutput().get(0);
        MatOfPoint point2 = vision.filterContoursOutput().get(1);
        Rect r1 = Imgproc.boundingRect(point1);
        Rect r2 = Imgproc.boundingRect(point2);
        int centerx1 = r1.x + (r1.width/2);
        int centerx2 = r2.x + (r2.width/2);
        double tapeCenter = ((centerx1 + centerx2)/2);
        double tapeOffset = (tapeCenter - 240);
        distance = 4349.75 / Math.abs(centerx1 - centerx2);
        angle = (tapeOffset/6.0);
      } else {
        distance = -2;
        angle = 180;
        badCount++;
        if (badCount % 10 == 0 ||
            visionMode.equals("tele") ||
             visionMode.equals("auto")) {
          Imgcodecs.imwrite("Img/bad" + val + "_" +
                            visionMode + "_" + contourCount + ".jpg", mat);
          Imgcodecs.imwrite("Img/hsv" + val + ".jpg", vision.hsvThresholdOutput());
        }
      }
      // Send X and Y to Network Table
  
      long current = System.nanoTime() / 1000000;
      last = current;
    }
  }
}
