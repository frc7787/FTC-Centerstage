package org.firstinspires.ftc.teamcode.Auto.Utility;

import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.LEFT;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.NONE;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.Properties.BOUNDING_RECTANGLE_COLOR;
import static org.firstinspires.ftc.teamcode.Properties.CROP_RECT;
import static org.firstinspires.ftc.teamcode.Properties.CV_ANCHOR;
import static org.firstinspires.ftc.teamcode.Properties.CV_BORDER_TYPE;
import static org.firstinspires.ftc.teamcode.Properties.CV_BORDER_VALUE;
import static org.firstinspires.ftc.teamcode.Properties.DIALATE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.Properties.ERODE_ITERATIONS;
import static org.firstinspires.ftc.teamcode.Properties.HIGH_HSV_RANGE_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.HIGH_HSV_RANGE_RED_ONE;
import static org.firstinspires.ftc.teamcode.Properties.HIGH_HSV_RANGE_RED_TWO;
import static org.firstinspires.ftc.teamcode.Properties.LEFT_X;
import static org.firstinspires.ftc.teamcode.Properties.LOW_HSV_RANGE_BLUE;
import static org.firstinspires.ftc.teamcode.Properties.LOW_HSV_RANGE_RED_ONE;
import static org.firstinspires.ftc.teamcode.Properties.LOW_HSV_RANGE_RED_TWO;
import static org.firstinspires.ftc.teamcode.Properties.RIGHT_X;

import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PropDetector extends OpenCvPipeline {

    public PropColor propColor;
    Rect cropRectangle;

    PropLocation location = NONE;

    private Mat mat            = new Mat(),
                mat1           = new Mat(),
                thresh0        = new Mat(),
                thresh1        = new Mat(),
                edges          = new Mat(),
                hierarchy      = new Mat(),
                cvDilateKernel = new Mat(),
                cvErodeKernel  = new Mat(),
                output         = new Mat();


    public PropDetector(@NonNull PropColor color, @NonNull Rect cropRectangle) {
        this.cropRectangle = cropRectangle;
        propColor = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Crop
        input = input.submat(cropRectangle);

        // Convert color to HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Imgproc.cvtColor(input, mat1, Imgproc.COLOR_RGB2HSV);

        switch (propColor) {
            case RED:
                // Check if the image is in range, then adds the ranges together
                Core.inRange(mat, LOW_HSV_RANGE_RED_ONE, HIGH_HSV_RANGE_RED_ONE, thresh0);
                Core.inRange(mat1, LOW_HSV_RANGE_RED_TWO, HIGH_HSV_RANGE_RED_TWO, thresh1);
                Core.add(thresh0, thresh1, output);

                break;
            case BLUE:
                // Checks if the image is in range
                Core.inRange(mat, LOW_HSV_RANGE_BLUE, HIGH_HSV_RANGE_BLUE, output);

                break;
        }

        // Erode to remove noise
        Imgproc.erode(
                output,
                output,
                cvErodeKernel,
                CV_ANCHOR,
                ERODE_ITERATIONS,
                CV_BORDER_TYPE,
                CV_BORDER_VALUE);

        // Dilate the image to get more of what is left
        Imgproc.dilate(
                output,
                output,
                cvDilateKernel,
                CV_ANCHOR,
                DIALATE_ITERATIONS,
                CV_BORDER_TYPE,
                CV_BORDER_VALUE);

        // Use Canny Edge Detection to find edges
        Imgproc.Canny(output, edges, 0, 100);

        // Oftentimes the edges are disconnected. findContours connects these edges.
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // -------------------------------------------------------------------
        // New Logic Start
        // -------------------------------------------------------------------

        Rect biggestBoundingBox = new Rect(0,0,0,0);

        for (Rect rect : boundRect) {
            if (rect.area() > biggestBoundingBox.area()) {
                biggestBoundingBox = rect;
            }
        }

        if (biggestBoundingBox.x < LEFT_X && biggestBoundingBox.area() != 0) {
            location = LEFT;
        } else if (biggestBoundingBox.x + biggestBoundingBox.width > RIGHT_X / 2.0) {
            location = RIGHT;
        }



        Imgproc.rectangle(mat, biggestBoundingBox, BOUNDING_RECTANGLE_COLOR);

        // ------------------------------------------------------------------
        // New Logic End
        // ------------------------------------------------------------------

        // Resizes the code so it can be viewed on the driver station
        // Comment this code out for competition
        Imgproc.resize(mat, mat, new Size(320, 240));

        return mat; // return the mat with rectangles drawn
    }

    public PropLocation getLocation() { return this.location; }
}