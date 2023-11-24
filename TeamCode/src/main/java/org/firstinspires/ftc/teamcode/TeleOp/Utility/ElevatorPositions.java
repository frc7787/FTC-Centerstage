package org.firstinspires.ftc.teamcode.TeleOp.Utility;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.Constants;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class ElevatorPositions {
    public static int ext1, rot1, ext2, rot2, ext3, rot3, ext4, rot4, ext5, rot5, hang, launch;

    @SuppressLint("SdCardPath")
    private static final String filePath = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/ElevatorPositionsNov14.txt";
    public static String errorStatus     = "NoError";

    private static void readCSV() {
        try {
            Scanner scanner = new Scanner(new File(filePath));

            // Skip header
            if (scanner.hasNextLine()) { scanner.nextLine(); }

            // Read 7 lines and extract x, y values
            for (int i = 1; i <= 7 && scanner.hasNextLine(); i++) {
                String[] values = scanner.nextLine().split(",");
                if (values.length == 2) {
                    int rot = Integer.parseInt(values[0]);
                    int ext = Integer.parseInt(values[1]);

                    // Assign values to respective variables
                    assignValues(i, rot, ext);
                }
            }

            scanner.close();
        } catch (FileNotFoundException e) { errorStatus = "FileNotFound"; }
    }

    private static void assignValues(int index, int col1, int col2) {
        switch (index) {
            case 1:
                ext1 = col1;
                rot1 = col2;
                break;
            case 2:
                ext2 = col1;
                rot2 = col2;
                break;
            case 3:
                ext3 = col1;
                rot3 = col2;
                break;
            case 4:
                ext4 = col1;
                rot4 = col2;
                break;
            case 5:
                ext5 = col1;
                rot5 = col2;
                break;
            case 6:
                launch = col1;
                hang   = col2;
                break;
        }
    }
    public static void updateElevatorConstants(){
        readCSV();
        Constants.BOTTOM_EXTEND_POSITION = ext1;
        Constants.BOTTOM_ROT_POSITION    = rot1;

        Constants.LOW_EXTEND_POSITION = ext2;
        Constants.LOW_ROT_POSITION    = rot2;

        Constants.MED_EXTEND_POSITION = ext3;
        Constants.MED_ROT_POSITION    = rot3;

        Constants.HIGH_EXTEND_POSITION = ext4;
        Constants.HIGH_ROT_POSITION    = rot4;

        Constants.TOP_EXTEND_POSITION = ext5;
        Constants.TOP_ROT_POSITION    = rot5;

        Constants.LAUNCH_POSITION = launch;
        Constants.HANG_POSITION   = hang;
    }
}