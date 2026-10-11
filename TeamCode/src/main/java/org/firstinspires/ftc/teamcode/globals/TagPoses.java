package org.firstinspires.ftc.teamcode.globals;

import com.seattlesolvers.solverslib.geometry.Pose2d;

public class TagPoses {
    //Positions
    public Pose2d blue39Down;
    public Pose2d blue39Up;
    public Pose2d blue44Down;
    public Pose2d blue44Up;
    public Pose2d red36Down;
    public Pose2d red36Up;
    public Pose2d red31Down;
    public Pose2d red31Up;

    public TagPoses() {
        blue39Down = new Pose2d(-10.008, 11.608, 0);
        blue39Up = new Pose2d(-10.008, 13.106, 0);
        blue44Down = new Pose2d(-10.008, -11.608, 0);
        blue44Up = new Pose2d(-10.008, -13.106, 0);
        red36Down = new Pose2d(9.92, 11.608, 0);
        red36Up = new Pose2d(9.92, 13.106, 0);
        red31Down = new Pose2d(9.92, -11.608, 0);
        red31Up = new Pose2d(9.92, -13.106, 0);
    }

    public Pose2d get(int tagID, RobotConstants.HiveState state) {
        if (state != RobotConstants.HiveState.UP && state != RobotConstants.HiveState.DOWN) return null;
        boolean up = (state == RobotConstants.HiveState.UP);

        switch (tagID) {
            case 39: return up ? blue39Up : blue39Down;
            case 44: return up ? blue44Up : blue44Down;
            case 36: return up ? red36Up  : red36Down;
            case 31: return up ? red31Up  : red31Down;
            default: return null;
        }
    }
}
