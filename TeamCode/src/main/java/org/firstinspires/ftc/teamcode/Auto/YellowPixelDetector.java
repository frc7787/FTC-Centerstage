package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Properties.LEFT_X;
import static org.firstinspires.ftc.teamcode.Properties.RIGHT_X;

import static org.firstinspires.ftc.teamcode.Auto.YellowPixelLocation.*;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * OpenCV pipeline to detect yellow pixel location on the backdrop in FTC 2023 - 2024 Centerstage
 */
@Config
public class YellowPixelDetector extends OpenCvPipeline {
    public static int VIEW_DISPLAYED = 1;
    public static int ERODE_PASSES   = 3;

    public static Scalar lowYellowHSVRange  = new Scalar(0, 0, 122);
    public static Scalar highYellowHSVRange = new Scalar(0, 0, 230);

    public static volatile Scalar BOUNDING_RECTANGLE_COLOR = new Scalar(255, 255, 255);

    private static final Point CV_ANCHOR        = new Point(-1, -1);
    private static final Scalar CV_BORDER_VALUE = new Scalar(-1);
    private static final int CV_BORDER_TYPE     = Core.BORDER_CONSTANT;

    YellowPixelLocation yellowPixelLocation = NONE;

    private final Mat hsvMat          = new Mat(),
                      threshold       = new Mat(),
                      hierarchy       = new Mat(),
                      cvErodeKernel   = new Mat(),
                      thresholdOutput = new Mat(),
                      erodeOutput     = new Mat();

    @Override public Mat processFrame(Mat input) {
        // Convert color to HSV
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Get everything that is bright
        Core.inRange(hsvMat, lowYellowHSVRange, highYellowHSVRange, threshold);

        // Erode to remove noise
        Imgproc.erode(thresholdOutput,
                      erodeOutput,
                      cvErodeKernel,
                      CV_ANCHOR,
                      ERODE_PASSES,
                      CV_BORDER_TYPE,
                      CV_BORDER_VALUE);

        // Finds the contours of the image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(erodeOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Creates bounding rectangles along all of the detected contours
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        Rect biggestBoundingBox = new Rect(0, 0, 0, 0);

        // Gets the biggest bounding box
        for (Rect rect : boundRect) {
            if (rect.area() > biggestBoundingBox.area()) {
                biggestBoundingBox = rect;
            }
        }

        if (biggestBoundingBox.area() != 0) { // If there is a yellow pixel
            if (biggestBoundingBox.x < LEFT_X) { // Check to see if the bounding box is on the left 25% of the screen
                yellowPixelLocation = LEFT;
            } else if (biggestBoundingBox.x > RIGHT_X) { // Check to see if the bounding box is on the right 25% of the screen
                yellowPixelLocation = RIGHT;
            }
        } else { // If there isn't a yellow pixel
            yellowPixelLocation = NONE;
        }

        // Draw a rectangle over the biggest bounding box
        Imgproc.rectangle(hsvMat, biggestBoundingBox, BOUNDING_RECTANGLE_COLOR);

        if (VIEW_DISPLAYED == 1) {
            Imgproc.rectangle(input, biggestBoundingBox, BOUNDING_RECTANGLE_COLOR);
            return input;
        } else if (VIEW_DISPLAYED == 2) {
            return threshold;
        }  else if (VIEW_DISPLAYED == 3) {
            return thresholdOutput;
        } else if (VIEW_DISPLAYED == 4) {
            return erodeOutput;
        } else {
            return hsvMat;
        }
    }

    public YellowPixelLocation getYellowPixelLocation() { return this.yellowPixelLocation; }
}