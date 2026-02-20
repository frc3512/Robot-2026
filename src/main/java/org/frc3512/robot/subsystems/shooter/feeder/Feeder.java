package org.frc3512.robot.subsystems.shooter.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  private FeederIO io;
  private FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public Command setFeeder(double speed) {
    return runOnce(() -> io.setFeeder(speed));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }
}
