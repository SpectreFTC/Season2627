package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.robot.Robot;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Rotation2d;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
import org.firstinspires.ftc.teamcode.globals.RobotUtils;
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

    public Pose relocalize(String name, MultipleTelemetry telemetry) {
        AprilTagClusterDetection detection = findTag(name);
        if (detection == null) {
            return null;
        }


        double tagX = TagPoses.get(name).getX();
        double tagY = TagPoses.get(name).getY();

        double rx = detection.robotPose.getPosition().x;
        double ry = detection.robotPose.getPosition().y;
        double yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
        double rh;

        double x;
        double y;

        if (name.equals("BLUE AUDIENCE") || name.equals("RED AUDIENCE")) {
            x = tagX + rx;
            y = tagY + ry;
            rh = 90 + yaw;
        }
        else {
            x = tagX - rx;
            y = tagY + ry;
            rh = 270 + yaw;
        }
        telemetry.addData("tagX", tagX);
        telemetry.addData("tagY", tagY);
        telemetry.addData("robX", rx);
        telemetry.addData("robY", ry);
        telemetry.addData("yaw", yaw);
        telemetry.addData("tag name", detection.metadata.name);
        return (new Pose(x, y, Math.toRadians(rh)));
    }


    public double autoAlign(String name, Pose2d robotPose) {
        AprilTagClusterDetection detection = findTag(name);
        double targetHeading = robotPose.getHeading();

        if (detection == null) {
            return 0;
        }
        //use this if the two sides change differently
//        if (name.equals("BLUE AUDIENCE") || name.equals("RED AUDIENCE")) {
//            targetHeading += Math.toRadians(detection.ftcPose.yaw);
//        }
//        else {
//            targetHeading += Math.toRadians(detection.ftcPose.yaw);
//        }

        targetHeading = Math.toRadians(detection.ftcPose.bearing);
        return (targetHeading);
    }
}