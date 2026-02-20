package org.frc3512.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

  private ConveyorIO io;
  private ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  public Conveyor(ConveyorIO io) {
    this.io = io;
  }

  public Command setHopper(double speed) {
    return runOnce(() -> io.setHopper(speed));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Conveyor", inputs);
  }
}
