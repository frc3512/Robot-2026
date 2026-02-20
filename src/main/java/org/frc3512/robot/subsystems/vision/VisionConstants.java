package org.frc3512.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static String frontLeftCamera = "Left Shooter";
  public static String frontRightCamera = "Right Shooter";

  // Robot to camera transforms
  public static Transform3d robotToLeft =
      new Transform3d(
          Units.inchesToMeters(0), // Left to Right
          Units.inchesToMeters(0), // Front to Back
          Units.inchesToMeters(0), // Bottom to Top
          new Rotation3d(0.0, Units.degreesToRadians(25), 0.0));

  public static Transform3d robotToRight =
      new Transform3d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          new Rotation3d(0.0, Units.degreesToRadians(25), 0.0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0 // Camera 2
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
