package org.firstinspires.ftc.teamcode.Auto.Utility;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PropDetectorRed extends OpenCvPipeline {
    public enum SkystoneLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private int width = 340; // width of the image
    SkystoneLocation location;


    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat mat0 = new Mat();
        Imgproc.cvtColor(input, mat0, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat0.empty()) {
            location = SkystoneLocation.NONE;
            return input;
        }

        Mat mat1 = new Mat();
        Imgproc.cvtColor(input, mat1, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat1.empty()) {
            location = SkystoneLocation.NONE;
            return input;
        }

        // We create a HSV range for blue to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value

        // First scalar
        Scalar lowHSV0 = new Scalar(160, 180, 0); // lower bound HSV for red
        Scalar highHSV0 = new Scalar(180, 255, 255); // higher bound HSV for red
        Mat thresh0 = new Mat();

        // Second scalar
        Scalar lowHSV1 = new Scalar(0, 180, 0); // lower bound HSV for red
        Scalar highHSV1 = new Scalar(10, 255, 255); // higher bound HSV for red
        Mat thresh1 = new Mat();


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat0, lowHSV0, highHSV0, thresh0);
        Core.inRange(mat1, lowHSV1, highHSV1, thresh1);
        
        Mat thresh = new Mat();
        
        Core.add(thresh0, thresh1, thresh);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((4*2) + 1, (4*2)+1));

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                left = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                right = true;

            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well

            // BEWARE!!!!
            Imgproc.rectangle(input, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        // IDK MAN, IT'S REVERSED!
        if (!right) location = SkystoneLocation.LEFT;
        else if (!left) location = SkystoneLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = SkystoneLocation.NONE;

        if (boundRect.length < 2) {
            location = SkystoneLocation.NONE;
        }

        return input; // return the mat with rectangles drawn
    }

    public SkystoneLocation getLocation() {
        return this.location;
    }
}