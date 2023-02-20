package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class AprilTagPipeline extends OpenCvPipeline {
    private long nativeAprilTagPtr;
    private Mat gray = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private int[] counts = {0, 0, 0, 0}; // Note that the fourth position is arbitrary

    private boolean beginCounting = false;
    private final Object beginCountingSync = new Object();

    // UNITS ARE METERS
    double tagsize;

    /** Note that tagsize should be in meters */
    public AprilTagPipeline(double tagsize) {
        this.tagsize = tagsize;

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 2, 3);
    }

    @Override
    protected void finalize() {
        // Might be null if createAprilTagDetector() threw an exception
        if(nativeAprilTagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
            nativeAprilTagPtr = 0;
        } else {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeAprilTagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGBA2GRAY);

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, gray, tagsize, 578.272, 578.272, 402.145, 221.506);

        // I don't think this requires a sync block, but I'm going to put one anyway
        synchronized (beginCountingSync) {
            if (beginCounting) {
                for (AprilTagDetection detection : detections)
                    counts[(detection.id < 3 && detection.id >= 0) ? (detection.id) : 3]++;
            }
        }

        Imgproc.putText(input, (detections.size() > 0) ? ("" + detections.get(0).id) : ("None"), new Point(25, 25), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 255, 255), 4);


        return input;
    }

    /** Returns the id of whichever tag is most likely to be present */
    public int getMaxFinds() {
        if (counts[0] >= counts[1] && counts[0] >= counts[2]) return 0;
        else if (counts[1] >= counts[0] && counts[1] >= counts[2]) return 1;
        else return 2;
    }

    public void beginCounting() {
        synchronized (beginCountingSync) {
            beginCounting = true;
        }
    }


}
