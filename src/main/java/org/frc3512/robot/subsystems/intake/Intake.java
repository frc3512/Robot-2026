package org.frc3512.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double lastExtensionPosition = 0.0;
  private double lastExtensionTimestamp = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setRollerDirect(double speed) {
    io.setRollerSpeed(speed);
  }

  public void setExtensionVelocity(double velocity) {
    io.setExtensionVelocity(velocity);
  }

  public Command setRollerSpeed(double speed) {
    return runOnce(() -> io.setRollerSpeed(speed));
  }

  public void setPositionDirect(IntakeConstants.IntakeState state) {
    io.setExtensionPosition(state);
  }

  public Command setPosition(IntakeConstants.IntakeState state) {
    return runOnce(() -> io.setExtensionPosition(state));
  }

  public boolean isExtensionStalled() {
    // Consider stalled if current is above threshold and velocity is very low
    return inputs.extensionAppliedVolts > 1.0 && Math.abs(getExtensionVelocity()) < 0.1;
  }

  public double getExtensionVelocity() {
    // Calculate velocity from position change
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - lastExtensionTimestamp;
    
    if (deltaTime > 0.0) {
      double velocity = (inputs.extensionPosition - lastExtensionPosition) / deltaTime;
      return velocity;
    }
    return 0.0;
  }

  public void rezeroExtension() {
    io.rezeroExtension();
  }

  @Override
  public void periodic() {
    // Update velocity tracking before updating inputs
    lastExtensionPosition = inputs.extensionPosition;
    lastExtensionTimestamp = Timer.getFPGATimestamp();
    
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
