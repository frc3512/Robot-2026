package org.frc3512.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIO_REAL implements IntakeIO {

  private TalonFX rollerMotor, extensionMotor;

  private final PositionVoltage intakeRequest = new PositionVoltage(0.0);
  private double wantedPosition = 0.0;

  public IntakeIO_REAL() {
    // Initialize motors
    rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
    extensionMotor = new TalonFX(IntakeConstants.extensionMotorID);

    rollerMotor.getConfigurator().apply(IntakeConstants.rollerMotorConfig);
    extensionMotor.getConfigurator().apply(IntakeConstants.extensionMotorConfig);

    rollerMotor.optimizeBusUtilization();
    extensionMotor.optimizeBusUtilization();

    extensionMotor.setPosition(0.000000);
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  @Override
  public void setExtensionPosition(IntakeConstants.IntakeState state) {
    wantedPosition = state.position;
    intakeRequest.Position = wantedPosition; // Position in rotations of motor
    extensionMotor.setControl(intakeRequest);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60.0; // RPM
    inputs.rollerAppliedVolts = rollerMotor.getStatorCurrent().getValueAsDouble();

    inputs.extensionPosition = extensionMotor.getPosition().getValueAsDouble();
    inputs.extensionAppliedVolts = extensionMotor.getStatorCurrent().getValueAsDouble();
  }
}
