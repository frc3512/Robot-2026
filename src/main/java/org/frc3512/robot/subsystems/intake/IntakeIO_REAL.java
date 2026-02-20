package org.frc3512.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIO_REAL implements IntakeIO {

  private TalonFX rollerMotor;

  public IntakeIO_REAL() {
    rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);

    // Configure extension motor for position control
    rollerMotor.getConfigurator().apply(IntakeConstants.rollerMotorConfig);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60.0; // RPM
    inputs.rollerAppliedVolts = rollerMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }
}
