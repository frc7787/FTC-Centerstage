package org.firstinspires.ftc.teamcode;

import java.io.FileInputStream;
import java.util.Properties;

public class RobotPropertyParser {
    private static final Properties robotProperties = new Properties();

    public static void loadProperties() {
        try {
            robotProperties.load(new FileInputStream("/sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/robot.properties"));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets a double from the robot.properties file located in onbot Java.
     * @param key The name of the value you would like to read
     * @return The obtained value
     */
    public static double getDouble(String key) { return Double.parseDouble(robotProperties.getProperty(key)); }

    /**
     * Gets an integer for the robot.properties file located in onbot Java.
     * @param key The name of the value you would like to read
     * @return The obtained value
     */
    public static int getInt(String key) { return Integer.parseInt(robotProperties.getProperty(key)); }
}