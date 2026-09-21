package frc.robot;

import java.util.concurrent.atomic.AtomicReference;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Timer;

public class VisionThread extends Thread
{
    private static UsbCamera camera;
    private static AtomicReference<Double> lastFrameTimeStamp;
    private static AprilTagFieldLayout rebuiltTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    private static AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(
    new AprilTagPoseEstimator.Config(
        Constants.APRILTAG_SIZE,
        1, 1,
        1, 1
    )
);

    public VisionThread(UsbCamera webcam, AtomicReference<Double> lastFrameTimeStamp)
    {
        super("RioVisionThread");
        setDaemon(true);

        this.camera = webcam;
        this.lastFrameTimeStamp = lastFrameTimeStamp;
    }

    @Override
    public void run()
    {
        try{
        OpenCvLoader.forceLoad();
        }
        catch(Exception e)
        {
            System.out.println("Load failed");
        }

        CvSink cvSink = CameraServer.getVideo(camera);;
        AprilTagDetector detector = new AprilTagDetector();
        Mat mat = new Mat();
        Mat grayMat = new Mat();
        MjpegServer overlayServer = new MjpegServer("RioVisionOverlayServer", Constants.CAMERA_OVERLAY_STREAM_PORT);

        // CvSource overlayOutput = CameraServer.putVideo(
        //     "RioVisionOverlay",
        //     Constants.CAMERA_WIDTH,
        //     Constants.CAMERA_HEIGHT
        // );

        CvSource overlayOutput = new CvSource(
            "RioVisionOverlay",
            PixelFormat.kBGR,
            Constants.CAMERA_WIDTH,
            Constants.CAMERA_HEIGHT,
            Constants.CAMERA_FPS
        );
        
        configureAprilDetector(detector);
        cvSink.setEnabled(true);
        overlayServer.setSource(overlayOutput);

        while(!Thread.interrupted())
        {
            long frameTime = cvSink.grabFrame(mat, 0.5);
            if(frameTime == 0) //error
            {
                continue;
            }
            lastFrameTimeStamp.set(Timer.getFPGATimestamp()); //robot run time

            Mat detectorFrame = prepareFrame(mat, grayMat);
            AprilTagDetection[] detections = detector.detect(detectorFrame);
            annotateFrame(mat, detections);
            overlayOutput.putFrame(mat);
 
        }
        detector.close();
            if (overlayServer != null) {
                overlayServer.close();
            }
            if (overlayOutput != null) {
                overlayOutput.close();
            }
            grayMat.release();
            mat.release();
    }

    /**
     * An image is stored in Mat. It can be either 1 channel (grayscale), 3 channel (BGR), or 4 Channel (BGR + Alpha.
     * Aprildetector needs a grayscale image, so anything not grayscale will be converted to grayscale.
     * @param frame An image (Mat) received by the camera
     * @param grayMat Another Mat that will be a grayscale version of frame if it is not already grayscale
     * @return A grayscale image from the camera
     */
    public static Mat prepareFrame(Mat frame, Mat grayMat)
    {
        switch(frame.channels())
        {
            case 1:
                return frame;
            case 3:
                Imgproc.cvtColor(frame, grayMat, Imgproc.COLOR_BGR2GRAY);
                return grayMat;
            case 4:
                Imgproc.cvtColor(frame, grayMat, Imgproc.COLOR_BGRA2GRAY);
                return grayMat;
            default:
                return frame;
        }
    }

    public static void configureAprilDetector(AprilTagDetector detector)
    {
        AprilTagDetector.Config detectorConfig = detector.getConfig();
        detectorConfig.numThreads = Constants.APRILTAG_NUM_THREADS;
        detectorConfig.quadDecimate = Constants.APRILTAG_QUAD_DECIMATE;
        detectorConfig.quadSigma = Constants.APRILTAG_QUAD_SIGMA;
        detectorConfig.refineEdges = true;
        detectorConfig.decodeSharpening = Constants.APRILTAG_DECODE_SHARPENING;
        detectorConfig.debug = false;
        detector.setConfig(detectorConfig);

        AprilTagDetector.QuadThresholdParameters quadThresholds = detector.getQuadThresholdParameters();
        quadThresholds.minClusterPixels = Constants.APRILTAG_MIN_CLUSTER_PIXELS;
        quadThresholds.maxNumMaxima = Constants.APRILTAG_MAX_NUM_MAXIMA;
        quadThresholds.criticalAngle = Constants.APRILTAG_CRITICAL_ANGLE_RAD;
        quadThresholds.maxLineFitMSE = Constants.APRILTAG_MAX_LINE_FIT_MSE;
        quadThresholds.minWhiteBlackDiff = Constants.APRILTAG_MIN_WHITE_BLACK_DIFF;
        quadThresholds.deglitch = Constants.APRILTAG_DEGLITCH;
        detector.setQuadThresholdParameters(quadThresholds);
        detector.addFamily("tag36h11");
    }




    private static void annotateFrame(Mat frame, AprilTagDetection[] detections) {
        Scalar centerColor = new Scalar(255, 255, 255);
        Scalar hubColor = new Scalar(60, 210, 80);
        Scalar otherColor = new Scalar(255, 180, 60);
        Scalar textColor = new Scalar(255, 255, 255);

        int imageCenterX = Constants.CAMERA_WIDTH / 2;
        int imageCenterY = Constants.CAMERA_HEIGHT / 2;
        Imgproc.drawMarker(frame, new Point(imageCenterX, imageCenterY), centerColor, Imgproc.MARKER_CROSS, 18, 1);
        Imgproc.putText(frame,"RIO TAG OVERLAY", new Point(8, 18), Imgproc.FONT_HERSHEY_SIMPLEX, 0.45, textColor,1);

        for (AprilTagDetection detection : detections) {
            boolean hubTarget = isHubTag(detection.getId());
            Scalar color = hubTarget ? hubColor : otherColor;
            Point[] corners = new Point[] {
                new Point(detection.getCornerX(0), detection.getCornerY(0)),
                new Point(detection.getCornerX(1), detection.getCornerY(1)),
                new Point(detection.getCornerX(2), detection.getCornerY(2)),
                new Point(detection.getCornerX(3), detection.getCornerY(3))
            };
            for (int i = 0; i < corners.length; i++) {
                Imgproc.line(frame, corners[i], corners[(i + 1) % corners.length], color, 2);
            }
            Point labelPoint = new Point(detection.getCornerX(0), Math.max(12.0, detection.getCornerY(0) - 6.0));
            Imgproc.putText(
                    frame,
                    (hubTarget ? "HUB " : "TAG ") + detection.getId(),
                    labelPoint,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1);
        }

        String summary = "HUB CTR --";
        String coverage = "tags 0 faces 0";

        Imgproc.putText(frame, summary, new Point(8, 36), Imgproc.FONT_HERSHEY_SIMPLEX, 0.42, textColor, 1);
        Imgproc.putText(frame, coverage, new Point(8, 54), Imgproc.FONT_HERSHEY_SIMPLEX, 0.42, textColor, 1);
    }

    public static boolean isHubTag(int detectedID)
    {
        for(int id : Constants.HUB_TAG_IDS)
        {
            if(id == detectedID) return true;
        }
        return false;
    }

    public static Pose2d getVisionOdometry(AprilTagDetection detection)
    {
        Transform3d cameraToTag = estimator.estimate(detection);
        Pose3d tagPose = rebuiltTagFieldLayout.getTagPose(detection.getId()).get();
        Pose3d cameraToField = tagPose.transformBy(cameraToTag.inverse());
        Pose3d robotToField = cameraToField.transformBy(Constants.CAMERATOROBOTLOCATION);

        return robotToField.toPose2d();
    }
}
