package org.firstinspires.ftc.teamcode.TeleOp.Utility;

import android.annotation.SuppressLint;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class ElevatorPositions {
    public static int x1, r1, x2, r2, x3, r3, x4, r4, x5, r5, x6, r6, x7, r7;

    @SuppressLint("SdCardPath")
    private static final String filePath = "/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/ElevatorPositionsNov14.txt";
    public static String errorStatus     = "NoError";

    public static void readCSV() {
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
                x1 = col1;
                r1 = col2;
                break;
            case 2:
                x2 = col1;
                r2 = col2;
                break;
            case 3:
                x3 = col1;
                r3 = col2;
                break;
            case 4:
                x4 = col1;
                r4 = col2;
                break;
            case 5:
                x5 = col1;
                r5 = col2;
                break;
            case 6:
                x6 = col1;
                r6 = col2;
                break;
            case 7:
                x7 = col1;
                r7 = col2;
                break;
        }
    }
}