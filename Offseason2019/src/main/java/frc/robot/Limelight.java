package frc.robot;

import edu.wpi.first.networktables.*;

/**
 * Limelight Vision Control
 */
public class Limelight {

    public static boolean isTargetFound() {
        double tv = getValue("tv", 0);
        return tv > 0.5;
    }

    public static double getTargetDistanceInInches() {
        double width = 3360 / getValue("thor", 0);
        return width;
    }

    public static double getTargetHorizontalAngle() {
        return getValue("tx", 0);
    }

    public static double getTargetVerticalAngle() {
        return getValue("ty", 0);
    }

    private static double getValue(String key, double defaultValue) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        return table.getEntry(key).getDouble(defaultValue);
    }
}