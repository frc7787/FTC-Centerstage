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

public class PropDetectorBlue extends OpenCvPipeline {
    public enum PropLocation {
        LEFT,
        RIGHT,
        NONE
    }

    private int width = 340; // width of the image
    PropLocation location;

    Mat mat = new Mat();
    Mat dst = new Mat();
    Mat edges = new Mat();
    Mat hierarchy = new Mat();


    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(new Rect(0, 80, 320, 80));

        // Make a working copy of the input matrix in HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            location = PropLocation.NONE;
            return input;
        }

        // We create a HSV range for blue to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(97, 100, 100); // lower bound HSV for blue
        Scalar highHSV = new Scalar(115, 255, 255); // higher bound HSV for blue


        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, dst);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Imgproc.Canny(dst, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
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
            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        // if there is no yellow regions on a side
        // that side should be a Skystone
        // IDK MAN, IT'S REVERSED!
        if (!right) location = PropLocation.LEFT;
        else if (!left) location = PropLocation.RIGHT;
            // if both are true, then there's no Skystone in front.
            // since our team's camera can only detect two at a time
            // we will need to scan the next 2 stones
        else location = PropLocation.NONE;

        if (boundRect.length == 0) {
            location = PropLocation.NONE;
        }


        Imgproc.resize(mat, mat, new Size(320, 240));


        return mat; // return the mat with rectangles drawn
    }

    public PropLocation getLocation() {
        return this.location;
    }
}