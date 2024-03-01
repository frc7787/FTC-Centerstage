package org.firstinspires.ftc.teamcode.Auto.Expiremental;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

// Goes with FirstVisionOpmodeNov26 to attempt integration with FTC Dashboard
@Config
public class FirstVisionProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public static int VIEW_DISPLAYED = 1;
    public static int ERODE_PASSES = 1;
    public static int DILATE_PASSES = 1;

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    public static Alliance alliance = Alliance.BLUE_ALLIANCE;


    double cX = 0;
    double cY = 0;
    double width = 0;

    //Scalar blueLowHSV = new Scalar(63, 113, 61); // the x values are the low and high hue range for blue
    //Scalar blueHighHSV = new Scalar(130, 206, 255); // picking the correct range for hue is important!
    Scalar blueLowHSV = new Scalar(101, 185, 37); // the x values are the low and high hue range for blue
    Scalar blueHighHSV = new Scalar(127, 255, 110); // picking the correct range for hue is important!
    Scalar redLowHSVrange1 = new Scalar(160, 180, 0); // the x values are the low and high hue range for blue
    Scalar redHighHSVrange1 = new Scalar(180, 255, 255); // picking the correct range for hue is important!
    Scalar redLowHSVrange2 = new Scalar(0, 180, 0); // the x values are the low and high hue range for blue
    Scalar redHighHSVrange2 = new Scalar(10, 255, 255); // picking the correct range for hue is important!
    //Scalar redLowHSVrange2 = new Scalar(133, 83, 112); // the x values are the low and high hue range for blue
    //Scalar redHighHSVrange2 = new Scalar(180, 254, 194); // picking the correct range for hue is important!


    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.CENTER;

    Mat submat = new Mat();

    Mat hierarchy = new Mat();
    Mat blurmat = new Mat();
    Mat hsvmat = new Mat();
    Mat coremat = new Mat();
    Mat thresh1mat = new Mat();
    Mat thresh2mat = new Mat();
    Mat erodedmat = new Mat();
    Mat contouredmat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    } // end init

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        coremat = preProcessFrame(frame);

        // CHAN_APPROX_SIMPLE: compresses horizontal, vertical, and diagonal segments and leaves only their end points.
        // https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga4303f45752694956374734a03c54d5ff
        // contours: Detected contours. Each contour is stored as a vector of points (e.g. std::vector<std::vector<cv::Point> >).
        // https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(erodedmat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) { // let's loop through list of contours

            MatOfPoint contour = contours.get(i);
            double countourArea = Imgproc.contourArea(contour, true);
            //double perimeter = Imgproc.arcLength(contour, true);
            Rect rectangle = Imgproc.boundingRect(contour);
            // Imgproc.rectangle(image, rectangle.tl(), rectangle.br(), new Scalar(255,0,0),1);
            Imgproc.rectangle(frame, rectangle.tl(), rectangle.br(), new Scalar(0,0,255),1);
        } // end of for i in contours

        MatOfPoint largestContour = findLargestContour(contours);

        if (largestContour != null) {
            Rect rectangle = Imgproc.boundingRect(largestContour);
            width = calculateWidth(largestContour);
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            //Imgproc.drawContours(newthresh, contours, contours.indexOf(largestContour), new Scalar(255,0,0),2);
            Imgproc.rectangle(erodedmat, rectangle.tl(), rectangle.br(), new Scalar(255),4);
            Imgproc.rectangle(frame, rectangle.tl(), rectangle.br(), new Scalar(255,0,0),4);
            String widthLabel = "Width: " + width + "pixels";
            String label = "(" + Math.round(cX) + ", " + Math.round(cY) + ")";
            Imgproc.putText(frame, widthLabel, new Point(cX+10,cY+20),Imgproc.FONT_HERSHEY_SIMPLEX,0.5,new Scalar(0,255,0),2);
            Imgproc.putText(frame,label,new Point(cX+10,cY),Imgproc.FONT_HERSHEY_COMPLEX,0.5,new Scalar(0,222,0),2);
            Imgproc.circle(frame, new Point(cX,cY),5,new Scalar(0,255,0),-1);

        } // end of if largestContour

        position = determinePosition(new Point(cX, cY));

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        //Utils.matToBitmap(frame, b);
        //lastFrame.set(b);

        if (VIEW_DISPLAYED == 1) {
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return frame;
        } else if (VIEW_DISPLAYED == 2){
            Utils.matToBitmap(blurmat, b);
            lastFrame.set(b);
            return blurmat;
        } else if (VIEW_DISPLAYED == 3) {
            Utils.matToBitmap(hsvmat, b);
            lastFrame.set(b);
            return hsvmat;
        } else if (VIEW_DISPLAYED == 4) {
            Utils.matToBitmap(coremat, b);
            lastFrame.set(b);
            return coremat;
        } else if (VIEW_DISPLAYED == 5) {
            Utils.matToBitmap(erodedmat, b);
            lastFrame.set(b);
            return erodedmat;
        } else {
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return frame;
        } // end of if VIEW_DISPLAYED

        //return null;

    } // end of ProcessFrame

    private Mat preProcessFrame(Mat processFrame) {
        Imgproc.blur(processFrame, blurmat, new Size(7, 7));
        Imgproc.cvtColor(blurmat, hsvmat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(97, 100, 100); // the x values are the low and high hue range for blue
        Scalar highHSV = new Scalar(115, 255, 255); // picking the correct range for hue is important!

        if (alliance== Alliance.BLUE_ALLIANCE) {
            Core.inRange(hsvmat, blueLowHSV, blueHighHSV, coremat);
        } else {
            Core.inRange(hsvmat, redLowHSVrange1, redHighHSVrange1, thresh1mat);
            Core.inRange(hsvmat, redLowHSVrange2, redHighHSVrange2, thresh2mat);
            Core.add(thresh1mat, thresh2mat, coremat);

        } // end of if alliance

        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));
        // for (int i = 0; i < contours.size(); i++)
        Imgproc.erode(coremat, erodedmat, erodeElement);
        //Imgproc.erode(coremat, erodedmat, erodeElement);
        //Imgproc.dilate(erodedmat, erodedmat, dilateElement);
        for (int i = 1; i < ERODE_PASSES; i++) {
            Imgproc.erode(erodedmat, erodedmat, erodeElement);
        }
        for (int i = 1; i < DILATE_PASSES; i++) {
            Imgproc.dilate(erodedmat, erodedmat, dilateElement);
        }

        return coremat;
    } // end of preProcessFrame

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        } // end of for contour in contours

        return largestContour;
    } // end private findLargestContour

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    } // end of double calculateWidth

    private ParkingPosition determinePosition(Point center) {
        ParkingPosition tempPosition = ParkingPosition.CENTER;

        if (center.x < 200 && center.y > 200) {
            tempPosition = ParkingPosition.LEFT;
        } else if (center.x > 400 && center.y > 200) {
            tempPosition = ParkingPosition.RIGHT;
        }

        return tempPosition;
    } // end of determinePosition

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    } // end of getPosition

    @Override
    public void onDrawFrame(Canvas canvas,int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext)
    {

    } // end of onDrawFrame

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    } // end getFrameBitmap

} // end of FirstVisionProcessorNov26