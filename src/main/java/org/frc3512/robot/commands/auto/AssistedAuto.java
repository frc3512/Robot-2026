package org.frc3512.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc3512.robot.subsystems.drive.Drive;
import org.frc3512.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Command that provides continuous vision-weighted pose fusion during autonomous path following.
 * Blends odometry and vision measurements to improve position accuracy.
 */
public class AssistedAuto extends Command {
  private final Drive drive;
  private final Vision vision;
  private final Command pathCommand;
  private final double visionWeight;
  private final double minVisionWeight;
  private final double maxVisionWeight;
  private double lastVisionTime = 0.0;
  private static final double FUSION_UPDATE_INTERVAL = 0.1; // 100ms updates

  public AssistedAuto(Drive drive, Vision vision, Command pathCommand, double visionWeight) {
    this(drive, vision, pathCommand, visionWeight, 0.1, 0.7);
  }

  public AssistedAuto(
      Drive drive,
      Vision vision,
      Command pathCommand,
      double visionWeight,
      double minVisionWeight,
      double maxVisionWeight) {
    this.drive = drive;
    this.vision = vision;
    this.pathCommand = pathCommand;
    this.visionWeight = MathUtil.clamp(visionWeight, minVisionWeight, maxVisionWeight);
    this.minVisionWeight = minVisionWeight;
    this.maxVisionWeight = maxVisionWeight;
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(pathCommand);
    lastVisionTime = 0.0;
    Logger.recordOutput("VisionAssistedAuto/State", "Initializing");
    Logger.recordOutput("VisionAssistedAuto/VisionWeight", this.visionWeight);
  }

  @Override
  public void execute() {
    // Check if we should apply vision fusion
    if (shouldApplyVisionFusion()) {
      applyVisionFusion();
    }

    Logger.recordOutput("VisionAssistedAuto/PathCommandScheduled", pathCommand.isScheduled());
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand.isScheduled()) {
      pathCommand.cancel();
    }
    Logger.recordOutput("VisionAssistedAuto/State", "Finished");
  }

  @Override
  public boolean isFinished() {
    return !pathCommand.isScheduled();
  }

  private boolean shouldApplyVisionFusion() {
    // Check if we have recent vision data and enough time has passed
    return hasRecentVisionData()
        && (Timer.getFPGATimestamp() - lastVisionTime) > FUSION_UPDATE_INTERVAL;
  }

  private void applyVisionFusion() {
    Pose2d odometryPose = drive.getPose();
    Pose2d visionPose = getVisionPose();

    if (visionPose != null) {
      // Calculate dynamic vision weight based on confidence
      double dynamicVisionWeight = calculateDynamicVisionWeight(odometryPose, visionPose);

      // Weighted average between odometry and vision
      Translation2d fusedTranslation =
          new Translation2d(
              odometryPose.getX() * (1.0 - dynamicVisionWeight)
                  + visionPose.getX() * dynamicVisionWeight,
              odometryPose.getY() * (1.0 - dynamicVisionWeight)
                  + visionPose.getY() * dynamicVisionWeight);

      // Keep odometry rotation for stability, but blend if angle error is large
      double angleError =
          Math.abs(odometryPose.getRotation().minus(visionPose.getRotation()).getDegrees());
      Rotation2d fusedRotation =
          (angleError > 10.0)
              ? odometryPose
                  .getRotation()
                  .interpolate(visionPose.getRotation(), dynamicVisionWeight)
              : odometryPose.getRotation();

      Pose2d fusedPose = new Pose2d(fusedTranslation, fusedRotation);
      drive.setPose(fusedPose);

      lastVisionTime = Timer.getFPGATimestamp();

      Logger.recordOutput("VisionAssistedAuto/State", "FusionApplied");
      Logger.recordOutput("VisionAssistedAuto/DynamicVisionWeight", dynamicVisionWeight);
      Logger.recordOutput("VisionAssistedAuto/OdometryPose", odometryPose);
      Logger.recordOutput("VisionAssistedAuto/VisionPose", visionPose);
      Logger.recordOutput("VisionAssistedAuto/FusedPose", fusedPose);
    } else {
      Logger.recordOutput("VisionAssistedAuto/State", "NoVisionData");
    }
  }

  private double calculateDynamicVisionWeight(Pose2d odometryPose, Pose2d visionPose) {
    // Calculate position difference
    double positionDiff = odometryPose.getTranslation().getDistance(visionPose.getTranslation());

    // Reduce vision weight if the difference is too large (likely bad vision measurement)
    if (positionDiff > 0.5) { // 50cm threshold
      return minVisionWeight;
    } else if (positionDiff > 0.2) { // 20cm threshold
      // Linear interpolation between min and max weight
      double ratio = (0.5 - positionDiff) / (0.5 - 0.2);
      return minVisionWeight + ratio * (maxVisionWeight - minVisionWeight);
    } else {
      return visionWeight; // Use configured weight for good measurements
    }
  }

  private boolean hasRecentVisionData() {
    return vision.hasRecentVisionData(0.5);
  }

  private Pose2d getVisionPose() {
    return vision.getLatestVisionPose();
  }
}
