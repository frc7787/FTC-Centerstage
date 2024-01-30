package org.firstinspires.ftc.teamcode.Auto.Utility;

import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.CENTER;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.LEFT;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.NONE;
import static org.firstinspires.ftc.teamcode.Auto.Utility.PropLocation.RIGHT;
import static org.firstinspires.ftc.teamcode.Properties.BOUNDING_RECTANGLE_COLOR;
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

/**
 * OpenCV pipeline to detect the prop in FTC 2023 - 2024 Centerstage
 */
public class PropDetector extends OpenCvPipeline {
    PropColor propColor;
    Rect cropRectangle;

    PropLocation propLocation = NONE;

    private Mat mat            = new Mat(),
                mat1           = new Mat(),
                thresh0        = new Mat(),
                thresh1        = new Mat(),
                edges          = new Mat(),
                hierarchy      = new Mat(),
                cvDilateKernel = new Mat(),
                cvErodeKernel  = new Mat(),
                output         = new Mat();


    public PropDetector(@NonNull PropColor color, Rect cropRectangle) {
        this.cropRectangle = cropRectangle;
        propColor = color;
    }

    @Override
    public Mat processFrame(Mat input) {

        if (cropRectangle != null) { // Check to see if we should crop
            // Crop the image
            input = input.submat(cropRectangle);
        }

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

        // Dilate the image to increase the size of what is left
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

        // Creates bounding rectangles along all of the detected contours
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        Rect biggestBoundingBox = new Rect(0,0,0,0);

        // Gets the biggest bounding box
        for (Rect rect : boundRect) {
            if (rect.area() > biggestBoundingBox.area()) {
                biggestBoundingBox = rect;
            }
        }

        // TODO: Maybe we should adjust the range just slightly from 25% to around 23% to increase accuracy?
        if (biggestBoundingBox.area() != 0) { // If we detect the prop
            if (biggestBoundingBox.x < LEFT_X) { // Check to see if the bounding box is on the left 25% of the screen
                propLocation = LEFT;
            } else if (biggestBoundingBox.x > RIGHT_X) { // Check to see if the bounding box is on the right 25% of the screen
                propLocation = RIGHT;
            } else { // If it isn't either, and the prop is detected it must be in the center
                propLocation = CENTER;
            }
        } else { // If we don't detect anything we assume there is no prop
            propLocation = NONE;
        }

        // All code below this line (Except for the return statement) should be commented out for competition to save processing

        // Draw a rectangle over the biggest bounding box
        Imgproc.rectangle(mat, biggestBoundingBox, BOUNDING_RECTANGLE_COLOR);

        // Resizes the code so it can be viewed on the driver station
        Imgproc.resize(mat, mat, new Size(320, 240));

        return mat; // return the mat
    }

    public PropLocation getPropLocation() { return this.propLocation; }
}