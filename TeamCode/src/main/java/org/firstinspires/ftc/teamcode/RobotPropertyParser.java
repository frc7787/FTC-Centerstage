package org.firstinspires.ftc.teamcode;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Properties;

public class RobotPropertyParser {

    private static final Properties robotProperties = new Properties();

    /**
     * Loads the robot properties from the OnBot JAVA robot.properties file
     */
    public static void loadProperties() {
        try {
            robotProperties.load(new FileInputStream("/sdcard/FIRST/java/src/robot.properties"));
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

    public static void saveProperties(){
        try {
            Date d = new Date();

            SimpleDateFormat format = new SimpleDateFormat("yyyy-M-dd-hh-mm");

            OutputStream outputStream = new FileOutputStream("/sdcard/FIRST/java/src/robot-"+format.format(d)+".properties");

            robotProperties.store(outputStream,null);

            outputStream.flush();
            outputStream.close();

            populateProperties();

            OutputStream outputStream2 = new FileOutputStream("/sdcard/FIRST/java/src/robot.properties");

            robotProperties.store(outputStream2,null);

            outputStream2.flush();
            outputStream2.close();

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    public static void populateProperties(){
        Field[] fields = org.firstinspires.ftc.teamcode.Properties.class.getDeclaredFields();

        for (Field field: fields) {
            if(Modifier.isStatic(field.getModifiers())){

                String fieldName = field.getName();

                Class fieldType = field.getType();

                try {
                    if(fieldType.isAssignableFrom(double.class)) {
                        double value = field.getDouble(null);
                        robotProperties.setProperty(fieldName,Double.toString(value));
                    }

                    if(fieldType.isAssignableFrom(int.class)){
                        int value = field.getInt(null);
                        robotProperties.setProperty(fieldName,Integer.toString(value));
                    }

                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    public static void populateConstants(){
        Field[] fields = org.firstinspires.ftc.teamcode.Properties.class.getDeclaredFields();

        for (Field field: fields){
            if(Modifier.isStatic(field.getModifiers())){
                String fieldName = field.getName();

                Class fieldType = field.getType();

                if(robotProperties.containsKey(fieldName)){
                    try {
                        if(fieldType.isAssignableFrom(double.class)) {
                            field.setDouble(fieldName,getDouble(fieldName));
                        }
                        if(fieldType.isAssignableFrom(int.class)){
                            field.setInt(fieldName,getInt(fieldName));
                        }
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException(e);
                    }
                }
            }
        }
    }
}