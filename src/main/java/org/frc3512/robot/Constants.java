package org.frc3512.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Optional;

public final class Constants {

  public final class GeneralConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }
  }

  public static class FieldConstants {
    public static Translation2d GOAL_LOCATION = hubPosition();

    // Position constants from WCP CC
    public static Translation2d hubPosition() {
      final Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
        return new Translation2d(Inches.of(182.105), Inches.of(158.845));
      }
      return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    }
  }
}
