package org.frc3512.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void setPositionDirect(double position) {
    io.setPosition(position);
  }

  public Command setPosition(double position) {
    return runOnce(() -> io.setPosition(position));
  }

  public boolean isAtTarget() {
    return io.isAtTarget();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }
}
