package org.frc3512.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberIO_REAL implements ClimberIO {

  private TalonFX climber;

  private DigitalInput topLimit, bottomLimit;

  private boolean canRaiseClimber() {
    return topLimit.get();
  }

  private boolean canLowerClimber() {
    return bottomLimit.get();
  }

  public ClimberIO_REAL() {
    climber = new TalonFX(ClimberConstants.climberMotorID);

    topLimit = new DigitalInput(ClimberConstants.topLimitID);
    bottomLimit = new DigitalInput(ClimberConstants.bottomLimitID);

    climber.getConfigurator().apply(ClimberConstants.climber);
  }

  @Override
  public void setClimber(double speed) {
    climber.set(speed);
  }

  @Override
  public Command raiseClimber() {
    return Commands.sequence(
        Commands.runOnce(() -> setClimber(0.5)),
        Commands.waitUntil(() -> !canRaiseClimber()),
        Commands.runOnce(() -> setClimber(0.0)));
  }

  @Override
  public Command lowerClimber() {
    return Commands.sequence(
        Commands.runOnce(() -> setClimber(-0.5)),
        Commands.waitUntil(() -> !canLowerClimber()),
        Commands.runOnce(() -> setClimber(0.0)));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Log velocity
    inputs.climberVelocity = climber.getVelocity().getValueAsDouble() * 60.0;

    // Log applied volts
    inputs.climberVolts = climber.getStatorCurrent().getValueAsDouble();

    // Log Climber Limits
    inputs.canRaiseClimber = canRaiseClimber();
    inputs.canLowerClimber = canLowerClimber();
  }
}
