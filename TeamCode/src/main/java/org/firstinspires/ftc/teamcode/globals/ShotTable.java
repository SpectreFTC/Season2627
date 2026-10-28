package org.firstinspires.ftc.teamcode.globals;

import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.Arrays;
import java.util.Locale;

/**
 * Shot table: flywheel velocity and hood angle for a given shooting distance.
 *
 * <p>Between recorded distances, values come from SolversLib's {@link InterpLUT}: a monotone
 * cubic spline that passes exactly through every point and doesn't overshoot between them.
 * Outside the recorded range it holds the nearest end point's values (it doesn't
 * extrapolate), so record a little past the closest and farthest spots you'll shoot from.</p>
 *
 * <p>Fill in the three arrays below from ShooterDistanceTuner's output. With fewer than 2
 * points there's no table yet, and {@link #velocity} and {@link #hoodAngle} fall back to the
 * mVelocity/bVelocity and mHoodAngle/bHoodAngle lines in RobotConstants.</p>
 *
 * <p>ShooterDistanceTuner can replace the table with {@link #set} until the Robot Controller
 * app restarts, so you can test a new table in teleop before pasting it in here.</p>
 */
public final class ShotTable {

    // ---- Paste ShooterDistanceTuner's output here ----
    // Distance: inches, the same distance the match code computes (distance to goal - 22).
    // Velocity: ticks/sec. Hood angle: the same units as ShooterSubsystem.setHoodAngle().
    // Same length, any order, no two distances equal.
    public static final double[] DISTANCES = {};
    public static final double[] VELOCITIES = {};
    public static final double[] HOOD_ANGLES = {};

    // The table in use. Declared before the static block below, so their initial values are
    // set before reset() runs rather than overwriting what it builds.
    private static InterpLUT velocityLUT;
    private static InterpLUT hoodLUT;
    private static double minDistance;
    private static double maxDistance;
    private static int size;

    static {
        reset();
    }

    private ShotTable() {
    }

    /** Goes back to the table in this file. */
    public static void reset() {
        set(DISTANCES, VELOCITIES, HOOD_ANGLES);
    }

    /**
     * Replaces the table until the app restarts or {@link #reset()} is called. Points can be
     * in any order; they're sorted by distance, since InterpLUT needs increasing distances.
     *
     * @throws IllegalArgumentException if the arrays differ in length or two distances are equal
     */
    public static void set(double[] distances, double[] velocities, double[] hoodAngles) {
        int n = distances.length;
        if (velocities.length != n || hoodAngles.length != n) {
            throw new IllegalArgumentException(String.format(Locale.US,
                    "ShotTable: %d distances, %d velocities and %d hood angles; they must be the same length.",
                    n, velocities.length, hoodAngles.length));
        }
        if (n < 2) {
            velocityLUT = null;
            hoodLUT = null;
            minDistance = Double.NaN;
            maxDistance = Double.NaN;
            size = n;
            return;
        }

        Integer[] order = new Integer[n];
        for (int i = 0; i < n; i++) order[i] = i;
        Arrays.sort(order, (a, b) -> Double.compare(distances[a], distances[b]));

        // Build both before switching over, so a bad table can't leave a half-updated one
        InterpLUT newVelocity = new InterpLUT();
        InterpLUT newHood = new InterpLUT();
        for (int k = 0; k < n; k++) {
            int i = order[k];
            if (k > 0 && distances[i] == distances[order[k - 1]]) {
                throw new IllegalArgumentException(String.format(Locale.US,
                        "ShotTable: two points at %.1f in. Every distance must be different.", distances[i]));
            }
            newVelocity.add(distances[i], velocities[i]);
            newHood.add(distances[i], hoodAngles[i]);
        }
        newVelocity.createLUT();
        newHood.createLUT();

        velocityLUT = newVelocity;
        hoodLUT = newHood;
        minDistance = distances[order[0]];
        maxDistance = distances[order[n - 1]];
        size = n;
    }

    /** Flywheel velocity (ticks/sec) for a distance (inches). */
    public static double velocity(double distance) {
        if (velocityLUT == null) {
            return RobotConstants.mVelocity * distance + RobotConstants.bVelocity;
        }
        return velocityLUT.get(distance);
    }

    /** Hood angle for a distance (inches), in setHoodAngle() units. */
    public static double hoodAngle(double distance) {
        if (hoodLUT == null) {
            return RobotConstants.mHoodAngle * distance + RobotConstants.bHoodAngle;
        }
        return hoodLUT.get(distance);
    }

    /** True when there's a table (2+ points); false when using the RobotConstants lines. */
    public static boolean hasTable() {
        return velocityLUT != null;
    }

    /** True when the distance is outside the table, so the nearest end point's values are used. */
    public static boolean isOutside(double distance) {
        return hasTable() && (distance < minDistance || distance > maxDistance);
    }

    public static double minDistance() {
        return minDistance;
    }

    public static double maxDistance() {
        return maxDistance;
    }

    /** Short description for telemetry. */
    public static String describe() {
        if (!hasTable()) {
            return "no table yet: using the mVelocity/bVelocity lines in RobotConstants";
        }
        return String.format(Locale.US, "%d points, %.1f to %.1f in", size, minDistance, maxDistance);
    }
}