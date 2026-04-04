package org.frc3512.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc3512.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/**
 * Command that waits until the robot's vision-estimated pose is within specified tolerances of an
 * expected pose. Useful for verifying position before proceeding with autonomous actions.
 */
public class VerifyPosition extends Command {
  private final Vision vision;
  private final Pose2d expectedPose;
  private final double positionToleranceMeters;
  private final double angleToleranceDegrees;
  private double timeoutSeconds;
  private double startTime;

public VerifyPosition(
      Vision vision, Pose2d expectedPose, double positionToleranceMeters) {
    this(vision, expectedPose, positionToleranceMeters, 5.0, 2.0);
  }

public VerifyPosition(
      Vision vision,
      Pose2d expectedPose,
      double positionToleranceMeters,
      double angleToleranceDegrees,
      double timeoutSeconds) {
    this.vision = vision;
    this.expectedPose = expectedPose;
    this.positionToleranceMeters = positionToleranceMeters;
    this.angleToleranceDegrees = angleToleranceDegrees;
    this.timeoutSeconds = timeoutSeconds;
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    Logger.recordOutput("VerifyPositionWithVision/State", "Initializing");
    Logger.recordOutput("VerifyPositionWithVision/ExpectedPose", expectedPose);
    Logger.recordOutput("VerifyPositionWithVision/PositionTolerance", positionToleranceMeters);
    Logger.recordOutput("VerifyPositionWithVision/AngleTolerance", angleToleranceDegrees);
  }

  @Override
  public void execute() {
    if (hasRecentVisionData()) {
      Pose2d currentPose = getVisionPose();
      if (currentPose != null) {
        double positionError =
            currentPose.getTranslation().getDistance(expectedPose.getTranslation());
        double angleError =
            Math.abs(currentPose.getRotation().minus(expectedPose.getRotation()).getDegrees());

        Logger.recordOutput("VerifyPositionWithVision/CurrentPose", currentPose);
        Logger.recordOutput("VerifyPositionWithVision/PositionError", positionError);
        Logger.recordOutput("VerifyPositionWithVision/AngleError", angleError);
        Logger.recordOutput(
            "VerifyPositionWithVision/TimeRemaining",
            timeoutSeconds - (Timer.getFPGATimestamp() - startTime));
      } else {
        Logger.recordOutput("VerifyPositionWithVision/State", "NoVisionData");
      }
    } else {
      Logger.recordOutput("VerifyPositionWithVision/State", "WaitingForVision");
    }
  }

  @Override
  public boolean isFinished() {
    // Check timeout first
    if (Timer.getFPGATimestamp() - startTime > timeoutSeconds) {
      Logger.recordOutput("VerifyPositionWithVision/State", "TimedOut");
      return true;
    }

    // Check vision verification
    if (hasRecentVisionData()) {
      Pose2d currentPose = getVisionPose();
      if (currentPose != null) {
        double positionError =
            currentPose.getTranslation().getDistance(expectedPose.getTranslation());
        double angleError =
            Math.abs(currentPose.getRotation().minus(expectedPose.getRotation()).getDegrees());

        if (positionError <= positionToleranceMeters && angleError <= angleToleranceDegrees) {
          Logger.recordOutput("VerifyPositionWithVision/State", "Verified");
          return true;
        }
      }
    }

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Logger.recordOutput("VerifyPositionWithVision/State", "Interrupted");
    }
  }

  private boolean hasRecentVisionData() {
    return vision.hasRecentVisionData(0.5); // 500ms timeout
  }

  private Pose2d getVisionPose() {
    Pose2d visionPose = vision.getLatestVisionPose();
    return visionPose; // Pure vision pose for verification
  }
}
