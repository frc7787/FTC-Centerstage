package org.firstinspires.ftc.teamcode.Auto.Utility;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class BluePropDetector extends OpenCvPipeline {

    PropLocation location = PropLocation.CENTER;

    @Override
    public Mat processFrame(Mat input) {

        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // If something goes wrong, we assume there's no prop
        if (mat.empty()) { return input; }

        Scalar lowHSV = new Scalar(97, 100, 100);   // lower bound HSV for blue
        Scalar highHSV = new Scalar(115, 255, 255); // higher bound HSV for blue
        Mat thresh = new Mat();

        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 0, 100);

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
        double left_x  = 0.25  * IMAGE_WIDTH;
        double right_x = 0.75  * IMAGE_WIDTH;
        boolean onLeftSide  = false;
        boolean onRightSide = false;
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].x < left_x)
                onLeftSide = true;
            if (boundRect[i].x + boundRect[i].width > right_x)
                onRightSide = true;

            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
        }

        if (!onLeftSide)  { location = PropLocation.LEFT;  }
        if (!onRightSide) { location = PropLocation.RIGHT; }

        return mat;
    }

    public PropLocation getLocation() {
        return this.location;
    }
}