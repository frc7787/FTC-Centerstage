package org.firstinspires.ftc.teamcode;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Objects;
import java.util.Properties;

public class GetRobotProperties {
    private static final Properties robotProperties = new Properties();

    private static final String rootPath           = Objects.requireNonNull(Thread.currentThread().getContextClassLoader()).getResource("").getPath();
    private static final String propertiesFilePath = rootPath + "robot.properties";

    public static void load() {
        try {
            robotProperties.load(Files.newInputStream(Paths.get(propertiesFilePath)));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static double readDouble(String key) { return Double.parseDouble(robotProperties.getProperty(key)); }

    public static int readInteger(String key) { return Integer.parseInt(robotProperties.getProperty(key)); }
}