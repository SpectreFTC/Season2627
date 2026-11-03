package org.firstinspires.ftc.teamcode.globals;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

public class TagPoses {
    //Positions
    public static Pose2d redAudience = new Pose2d(57.758, 56.31, Math.toRadians(-90));
    public static Pose2d blueAudience = new Pose2d(83.258, 56.31, Math.toRadians(-90));
    public static Pose2d redScoring = new Pose2d(57.758, 83.69, Math.toRadians(90));
    public static Pose2d blueScoring = new Pose2d(83.258, 84.69, Math.toRadians(90));

    public TagPoses() {

    }

    public static Pose2d get(String name) {

        switch (name) {
            case "BLUE AUDIENCE": return blueAudience;
            case "RED AUDIENCE": return redAudience;
            case "RED SCORING": return redScoring;
            case "BLUE SCORING": return blueScoring;
        }
        return null;
    }
}
