package org.frc3512.robot.subsystems.vision;

/** Constants for vision-based pose correction in autonomous routines. */
public final class VisionCorrectionConstants {
  public static final double VISION_CORRECTION_TOLERANCE_METERS = 0.1; // 10cm
  public static final double VISION_CORRECTION_ANGLE_TOLERANCE_DEGREES = 5.0;
  public static final double VISION_FUSION_WEIGHT = 0.5; // 50% vision, 50% odometry
  public static final double VISION_FUSION_MIN_WEIGHT = 0.1;
  public static final double VISION_FUSION_MAX_WEIGHT = 0.7;
  public static final double VISION_UPDATE_INTERVAL_SECONDS = 0.1; // 100ms
  public static final double VISION_TIMEOUT_SECONDS = 0.5;  
  public static final double REPLANNING_THRESHOLD_METERS = 0.2; // 20cm
  public static final double VISION_POSITION_DIFF_THRESHOLD_HIGH = 0.5; // 50cm
  public static final double VISION_POSITION_DIFF_THRESHOLD_LOW = 0.2; // 20cm

  // Key waypoint poses for vision correction (in meters)
  public static final class WaypointPoses {
    // NZ (Note Zone) positions
    public static final double NZ_LEFT_X = 2.0;
    public static final double NZ_LEFT_Y = 4.8;
    public static final double NZ_LEFT_HEADING_DEGREES = -120.0;

    public static final double NZ_RIGHT_X = 2.0;
    public static final double NZ_RIGHT_Y = 3.2;
    public static final double NZ_RIGHT_HEADING_DEGREES = 120.0;

    // HP (Human Player) positions
    public static final double HP_X = 1.7;
    public static final double HP_Y = 1.6;
    public static final double HP_HEADING_DEGREES = 0.0;

    // Shooting positions
    public static final double SHOOT_MID_X = 3.5;
    public static final double SHOOT_MID_Y = 4.0;
    public static final double SHOOT_MID_HEADING_DEGREES = 0.0;
  }

  private VisionCorrectionConstants() {
    // Utility class
  }
}
