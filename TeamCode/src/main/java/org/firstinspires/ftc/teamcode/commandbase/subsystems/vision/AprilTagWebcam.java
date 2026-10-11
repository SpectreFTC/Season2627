package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.robot.Robot;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.TagPoses;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {
    private final AprilTagProcessor aprilTag;
    private final TagPoses tagPoses;
    public AprilTagWebcam(AprilTagProcessor aprilTagProcessor, TagPoses tagPoses) {
        this.aprilTag = aprilTagProcessor;
        this.tagPoses = tagPoses;

    }

    private AprilTagDetection findTag(int tagID) {
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDetection = (AprilTagSingleDetection) detection;
                if (singleDetection.id == tagID) {
                    return singleDetection;
                }
                else if (singleDetection.ftcPose == null) {
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

    public RobotConstants.HiveState getHiveState(int tagID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection hiveTag = hiveTag = findTag(tagID);


        if (hiveTag == null) {
            return RobotConstants.HiveState.NOT_VISIBLE;
        }
        double cameraRadians = Math.toRadians(RobotConstants.cameraAngle);
        double hiveHeightInches = RobotConstants.cameraHeightIn + ((hiveTag.ftcPose.z * Math.cos(cameraRadians)) + (hiveTag.ftcPose.y * Math.sin(cameraRadians)));

        if (hiveHeightInches > RobotConstants.hiveHeightUp) {
            return RobotConstants.HiveState.UP;
        }
        else if (hiveHeightInches < RobotConstants.hiveHeightDown) {
            return RobotConstants.HiveState.DOWN;
        }
        else {
            return RobotConstants.HiveState.LEVEL;
        }
    }

    public Pose2d relocalize(int tagID, double headingRad) {
        Pose2d botPose = null;
        AprilTagDetection tag = findTag(tagID);

        RobotConstants.HiveState state = getHiveState(tagID);
        Pose2d tagPose = tagPoses.get(tagID, state);
        if (tagPose == null) {
            return null;
        }

        double pitch = Math.toRadians(RobotConstants.cameraAngle);
        double forward = tag.ftcPose.y * Math.cos(pitch) - tag.ftcPose.z * Math.sin(pitch);
        double left = -tag.ftcPose.x;

        double a = forward + RobotConstants.cameraForwardOffsetIn;
        double b = left + RobotConstants.cameraLeftOffsetIn;

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double x = tagPose.getX() - (a * cos - b * sin);
        double y = tagPose.getY() - (a * sin + b * cos);

        return new Pose2d(x, y, new Rotation2d(headingRad));
    }
}
