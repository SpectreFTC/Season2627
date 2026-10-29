package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.robot.Robot;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    public AprilTagWebcam(AprilTagProcessor aprilTagProcessor, TagPoses tagPoses) {
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

        double tagX = detection.metadata.fieldPosition.get(0);
        double tagY = detection.metadata.fieldPosition.get(1);

        double rx = detection.robotPose.getPosition().x;
        double ry = detection.robotPose.getPosition().y;
        double rh = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

        double x;
        double y;

        if (name.equals("BLUE AUDIENCE") || name.equals("RED AUDIENCE")) {
            x = tagX + rx;
            y = tagY - ry;
        }
        else {
            x = tagX - rx;
            y = tagY + ry;
            rh += Math.toRadians(180);
        }

        return (new Pose2d(x, y, rh));
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