package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Bearing (radians, + = cluster is left) from robot center to a cluster. NaN if none.
 * getBearing() picks the cluster itself: upright, your alliance, closest to straight ahead,
 * and sticks with it while it stays visible. No odometry needed.
 */
public class ClusterBearing {
    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";

    public static String ALLIANCE = "";  // "RED", "BLUE", or "" for any
    public static double CAM_X = 1.38944882;      // in, +right of robot center
    public static double CAM_Y = -4.327992126;      // in, +forward of robot center
    public static double CAM_PITCH = 32.337;  // deg, +tilted up
    public static long HOLD_MS = 150;
    public static float DECIMATION = 2;  // higher = faster, less range

    private static final class Target {
        final String name;
        double bearing;
        long seenNs;
        Target(String name) { this.name = name; }
    }

    private final AprilTagProcessor tags;
    private final VisionPortal portal;
    private final HashMap<String, Target> targets = new HashMap<>(8);
    private final double cosP, sinP;
    private Target current;
    private boolean newFrame;

    public ClusterBearing(HardwareMap hw, String webcam) {
        cosP = Math.cos(Math.toRadians(CAM_PITCH));
        sinP = Math.sin(Math.toRadians(CAM_PITCH));

        tags = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .setDrawTagID(false)
                .build();
        tags.setDecimation(DECIMATION);

        portal = new VisionPortal.Builder()
                .setCamera(hw.get(WebcamName.class, webcam))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                .addProcessor(tags)
                .build();
    }

    public double getBearing() {
        update();
        return fresh(current) ? current.bearing : Double.NaN;
    }

    public double getBearing(String cluster) {
        update();
        Target t = targets.get(cluster);
        return fresh(t) ? t.bearing : Double.NaN;
    }

    /** Name of the cluster getBearing() is aiming at, or null. */
    public String getTargetName() { return fresh(current) ? current.name : null; }

    /** True if the last getBearing() call picked up a new camera frame. */
    public boolean isNewFrame() { return newFrame; }

    private boolean fresh(Target t) {
        return t != null && System.nanoTime() - t.seenNs < HOLD_MS * 1_000_000L;
    }

    private void update() {
        ArrayList<AprilTagDetection> fresh = tags.getFreshDetections();
        newFrame = fresh != null;
        if (fresh == null || fresh.isEmpty()) return;

        long now = System.nanoTime();
        Target best = null;
        double bestAbs = Double.MAX_VALUE;
        boolean currentInFrame = false, currentValid = false;

        for (int i = 0, n = fresh.size(); i < n; i++) {
            AprilTagDetection d = fresh.get(i);
            if (!(d instanceof AprilTagClusterDetection) || d.ftcPose == null) continue;

            Target t = target(((AprilTagClusterDetection) d).metadata.name);
            double px = d.ftcPose.x + CAM_X;
            double py = d.ftcPose.y * cosP - d.ftcPose.z * sinP + CAM_Y;
            t.bearing = Math.atan2(-px, py);
            t.seenNs = now;

            if (t == current) currentInFrame = true;
            if (Math.abs(d.ftcPose.roll) < Math.PI / 2 && t.name.startsWith(ALLIANCE)) {
                if (t == current) currentValid = true;
                double absB = Math.abs(t.bearing);
                if (absB < bestAbs) {
                    bestAbs = absB;
                    best = t;
                }
            }
        }

        if (!currentValid) {
            if (best != null) current = best;
            else if (currentInFrame) current = null;
        }
    }

    private Target target(String name) {
        Target t = targets.get(name);
        if (t == null) targets.put(name, t = new Target(name));
        return t;
    }

    public void setEnabled(boolean on) { portal.setProcessorEnabled(tags, on); }

    public void close() { portal.close(); }
}