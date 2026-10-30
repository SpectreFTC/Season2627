package org.firstinspires.ftc.teamcode.globals;

import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.geometry.Translation2d;

public class TagPoses {
    //Positions
    public static Translation2d redAudience = new Translation2d(57.758, 56.31);
    public static Translation2d blueAudience = new Translation2d(83.258, 56.31);
    public static Translation2d redScoring = new Translation2d(57.758, 83.69);
    public static Translation2d blueScoring = new Translation2d(83.258, 84.69);

    public TagPoses() {

    }

    public static Translation2d get(String name) {

        switch (name) {
            case "BLUE AUDIENCE": return blueAudience;
            case "RED AUDIENCE": return redAudience;
            case "RED SCORING": return redScoring;
            case "BLUE SCORING": return blueScoring;
        }
        return null;
    }
}
