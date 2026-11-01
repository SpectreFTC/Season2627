package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import com.seattlesolvers.solverslib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class AprilTagWebcam {
    private final AprilTagProcessor aprilTag;
    public AprilTagWebcam(AprilTagProcessor aprilTagProcessor) {
        this.aprilTag = aprilTagProcessor;

    }

    private AprilTagClusterDetection findTag(String name) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;
                if (clusterDetection.metadata.name.equals(name)) {
                    return clusterDetection;
                }
                else if (clusterDetection.ftcPose == null) {
                    return null;
                }
            }
        }
        return null;
    }
    //Mostly copied from the ConceptAprilTag class.
    public void telemetryAprilTag(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (!currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                    telemetry.addLine(String.format("ID: %d", singleDet.id));
                    telemetry.addLine(String.format("XYZ: %6.1f %6.1f %6.1f: ", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY: %6.1f %6.1f %6.1f: ", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));

                } else {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    telemetry.addLine(String.format("Cluster: %s", clusterDet.metadata.name));
                    telemetry.addLine(String.format("XYZ: %6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY: %6.1f %6.1f %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                }
            }
        }
    }

    public Pose2d relocalize(String name, Pose2d tagPose) {
        AprilTagClusterDetection detection = findTag(name);
        if (detection == null) {
            return null;
        }

        //multiply by neg 1 if heading is flipped
        double tag_heading = Math.PI + (detection.ftcPose.yaw);
        double botHeadingRad = Math.toRadians(tagPose.getHeading()) - tag_heading;

        //cam to april tag relative pos
        //correct for cam pitch
        double fr = (detection.ftcPose.y * Math.cos(Math.toRadians(RobotConstants.camPitchDeg))) - (detection.ftcPose.z * Math.sin(Math.toRadians(RobotConstants.camPitchDeg)));
        double lr = -detection.ftcPose.x;

        //camera pos, correcting for heading
        double cx = tagPose.getX() - (fr * Math.cos(botHeadingRad) - lr * Math.sin(botHeadingRad));
        double cy = tagPose.getY() - (fr * Math.sin(botHeadingRad) + lr * Math.cos(botHeadingRad));

        //robot pos, correcting for heading (for cam offsets)
        double rx = cx - (RobotConstants.fCamOffset * Math.cos(botHeadingRad) - RobotConstants.lCamOffset * Math.sin(botHeadingRad));
        double ry = cy - (RobotConstants.fCamOffset * Math.sin(botHeadingRad) + RobotConstants.lCamOffset * Math.cos(botHeadingRad));

        return new Pose2d(rx, ry, Math.toDegrees(botHeadingRad));
    }


    public Pose2d autoAlign(String name, Pose2d robotPose) {
        AprilTagClusterDetection detection = findTag(name);

        if (detection == null) {
            return null;
        }

        double targetHeading = robotPose.getHeading() +  Math.toRadians(detection.ftcPose.yaw);
        return (new Pose2d(robotPose.getX(), robotPose.getY(), targetHeading));
    }
}