package org.firstinspires.ftc.teamcode.opmodes.tuning;

//import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // ---- Settings (editable from FTC Dashboard) ----
    public static double START_POWER = 0.2;
    public static double END_POWER = 1;
    public static double POWER_STEP = 0.1;
    public static double SETTLE_SECONDS = 2.0;   // time for the flywheel to reach steady speed
    public static double SAMPLE_SECONDS = 1.0;   // time to average over at each step
    public static double VOLTAGE_READ_MS = 50;   // how often to read battery voltage (0 = every loop)
    public static int VELOCITY_WINDOW = 500;     // max velocity readings averaged per step
    public static int VOLTAGE_WINDOW = 50;       // max voltage readings averaged per step
}
