package org.frc3512.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Command that corrects robot pose using vision measurements at specific waypoints. Only corrects
 * once per command instance to avoid oscillation.
 */
public class PoseCorrector extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Pose2d expectedPose;
  private final double toleranceMeters;
  private final double angleToleranceDegrees;
  private boolean hasCorrected = false;

  public PoseCorrector(Drive drive, Vision vision, Pose2d expectedPose, double toleranceMeters) {
    this(drive, vision, expectedPose, toleranceMeters, 5.0);
  }

  public PoseCorrector(
      Drive drive,
      Vision vision,
      Pose2d expectedPose,
      double toleranceMeters,
      double angleToleranceDegrees) {
    this.drive = drive;
    this.vision = vision;
    this.expectedPose = expectedPose;
    this.toleranceMeters = toleranceMeters;
    this.angleToleranceDegrees = angleToleranceDegrees;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    hasCorrected = false;
    Logger.recordOutput("VisionPoseCorrector/State", "Initializing");
  }

  @Override
  public void execute() {
    // Only correct once per waypoint to avoid oscillation
    if (!hasCorrected && hasRecentVisionData()) {
      Pose2d currentPose = drive.getPose();
      double distanceError =
          currentPose.getTranslation().getDistance(expectedPose.getTranslation());
      double angleError =
          Math.abs(currentPose.getRotation().minus(expectedPose.getRotation()).getDegrees());

      Logger.recordOutput("VisionPoseCorrector/DistanceError", distanceError);
      Logger.recordOutput("VisionPoseCorrector/AngleError", angleError);
      Logger.recordOutput("VisionPoseCorrector/ExpectedPose", expectedPose);
      Logger.recordOutput("VisionPoseCorrector/CurrentPose", currentPose);

      if (distanceError > toleranceMeters || angleError > angleToleranceDegrees) {
        // Get vision pose - for now, we'll simulate this with a placeholder
        // In reality, you'd get this from your vision system
        Pose2d visionPose = getVisionPose();

        if (visionPose != null) {
          // Use vision to correct position, but keep heading from path if angle error is small
          Rotation2d correctedRotation =
              (angleError < angleToleranceDegrees)
                  ? expectedPose.getRotation()
                  : visionPose.getRotation();

          Pose2d correctedPose = new Pose2d(visionPose.getTranslation(), correctedRotation);
          drive.setPose(correctedPose);
          hasCorrected = true;

          Logger.recordOutput("VisionPoseCorrector/State", "Corrected");
          Logger.recordOutput("VisionPoseCorrector/CorrectedPose", correctedPose);
        } else {
          Logger.recordOutput("VisionPoseCorrector/State", "NoVisionData");
        }
      } else {
        Logger.recordOutput("VisionPoseCorrector/State", "WithinTolerance");
      }
    }
  }

  @Override
  public boolean isFinished() {
    return hasCorrected || !hasRecentVisionData();
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("VisionPoseCorrector/State", "Finished");
  }

  private boolean hasRecentVisionData() {
    return vision.hasRecentVisionData(0.5); // 500ms timeout
  }

  private Pose2d getVisionPose() {
    return vision.getLatestVisionPose();
  }
}
