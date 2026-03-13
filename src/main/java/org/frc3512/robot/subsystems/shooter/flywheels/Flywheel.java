package org.frc3512.robot.subsystems.shooter.flywheels;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void setRPMDirect(double rpm) {
    io.setRPM(rpm);
  }

  public Command setRPM(double rpm) {
    return runOnce(() -> io.setRPM(rpm));
  }

  public Command setOutput(double output) {
    return runOnce(() -> io.setOutput(output));
  }

  public Command stop() {
    return runOnce(() -> io.stop());
  }

  public boolean isVelocityWithinTolerance() {
    return io.isVelocityWithinTolerance();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}
