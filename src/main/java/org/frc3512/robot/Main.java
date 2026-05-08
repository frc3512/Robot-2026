package org.frc3512.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.frc3512.robot.ai.TrainingUIMain;

public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(TrainingUIMain::new);
  }
}
