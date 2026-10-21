package org.firstinspires.ftc.teamcode.globals;

/**
 * Utility class containing various mathematical and helper functions
 * relevant for robot operations.
 */
public class RobotUtils {
    /**
     * Normalizes an angle in degrees to be within the range of -180 to 180 degrees.
     *
     * @param angle The angle in degrees to normalize.
     * @return The normalized angle in degrees, between -180 and 180.
     */
    public static double normalizeAngleInDegrees(double angle) {
        angle = angle % 360; // Basic reduction
        // Ensure range is (-180, 180]
        if (angle <= -180) { // If it's -180 or less, add 360
            angle += 360;    // e.g., -180 becomes 180; -190 becomes 170
        } else if (angle > 180) { // If it's greater than 180, subtract 360
            angle -= 360;    // e.g., 190 becomes -170
        }
        return angle;
    }


    /**
     * Normalizes an angle in radians to be within the range of -PI to PI radians.
     * This is achieved by first converting the angle to degrees, normalizing it
     * using {@link #normalizeAngleInDegrees(double)}, and then converting it back to radians.
     *
     * @param angle The angle in radians to normalize.
     * @return The normalized angle in radians, between -PI and PI.
     */
    public static double normalizeAngleInRadians(double angle) {
        return Math.toRadians(normalizeAngleInDegrees(Math.toDegrees(angle)));
    }


    /**
     * Checks if a given value falls within a specified range (inclusive of min and max).
     *
     * @param comparingValue The value to check.
     * @param min            The minimum bound of the range.
     * @param max            The maximum bound of the range.
     * @return {@code true} if the value is within the range [min, max], {@code false} otherwise.
     */
    public static boolean isWithinRange(double comparingValue, double min, double max) {
        return comparingValue >= min && comparingValue <= max;
    }

    /**
     * Determines if a point (x, y) is inside or on the boundary of a circle
     * centered at the origin (0,0) with a given radius.
     *
     * @param x The x-coordinate of the point.
     * @param y The y-coordinate of the point.
     * @param r The radius of the circle.
     * @return {@code true} if the point (x,y) is inside or on the circle, {@code false} otherwise.
     * Returns {@code false} if the radius is negative.
     */
    public static boolean isInsideCircle(double x, double y, double r) {
        if (r < 0) {
            return false;
        }
        return x * x + y * y <= r * r;
    }
}
