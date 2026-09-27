package org.firstinspires.ftc.teamcode.commandbase.subsystems.vision;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.globals.RobotConstants;
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

    public int hiveTipped(int tagID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection hiveTag = null;


        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                if (singleDet.id == tagID) {
                    hiveTag = detection;
                    break;
                }
            }
        }

        if (hiveTag == null) {
            return -1;
        }

        double hivePitch = hiveTag.ftcPose.pitch;

        if (hivePitch > RobotConstants.hiveUpPitch) {
            return 1;
        }
        else if (hivePitch < RobotConstants.hiveDownPitch) {
            return 2;
        } else {
            return 0;
        }
    }
}
