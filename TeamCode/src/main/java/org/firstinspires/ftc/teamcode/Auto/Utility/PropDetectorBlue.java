package org.firstinspires.ftc.teamcode.Auto.Utility;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PropDetectorBlue extends OpenCvPipeline {

    private int width = 340; // width of the image
    PropLocation location;

    Mat mat            = new Mat();
    Mat output         = new Mat();
    Mat edges          = new Mat();
    Mat hierarchy      = new Mat();
    Mat cvErodeKernel  = new Mat();
    Mat cvDilateKernel = new Mat();

    public final int ERODE_ITERATIONS   = 7;
    public final int DIALATE_ITERATIONS = 11;

    @Override
    public Mat processFrame(Mat input) {
        input = input.submat(new Rect(0, 80, 320, 80));

        // Make a working copy of the input matrix in HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // If something goes wrong, we assume there is no prop and return the input
        if (mat.empty()) {
            location = PropLocation.NONE;
            return input;
        }

        // HSV values are half of what they actually are
        Scalar lowHSV  = new Scalar(97, 100, 100);   // lower bound HSV for blue
        Scalar highHSV = new Scalar(115, 255, 255); // higher bound HSV for blue

        Core.inRange(mat, lowHSV, highHSV, output);

        // Run erode to remove noise
        Point cvErodeAnchor       = new Point(-1, -1);
        int cvErodeBordertype     = Core.BORDER_CONSTANT;
        Scalar cvErodeBordervalue = new Scalar(-1);
        Imgproc.erode(output, output, cvErodeKernel, cvErodeAnchor, ERODE_ITERATIONS, cvErodeBordertype, cvErodeBordervalue);


        // Run dilation to increase the stuff we want
        Point cvDilateAnchor       = new Point(-1, -1);
        int cvDilateBordertype     = Core.BORDER_CONSTANT;
        Scalar cvDilateBordervalue = new Scalar(-1);
        Imgproc.dilate(output, output, cvDilateKernel, cvDilateAnchor, DIALATE_ITERATIONS, cvDilateBordertype, cvDilateBordervalue);

        // Use Canny Edge Detection to find edges
        Imgproc.Canny(output, edges, 100, 300);

        // Find the contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Draw bounding boxes around the contours
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        double left_x  = 0.25 * width;
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

        // IDK MAN, IT'S REVERSED!
        if (!right)     location = PropLocation.LEFT;
        else if (!left) location = PropLocation.RIGHT;
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