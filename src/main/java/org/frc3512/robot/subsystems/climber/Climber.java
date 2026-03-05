package org.frc3512.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public Command setClimber(double speed) {
    return runOnce(() -> io.setClimber(speed));
  }

  public Command climb() {
    return runOnce(() -> io.lowerClimber());
  }

  public Command raiseClimber() {
    return runOnce(() -> io.raiseClimber());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}
