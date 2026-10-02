package org.frc3512.robot.subsystems.intake;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIO_REAL implements IntakeIO {

  private TalonFX rollerMotor, secondaryRollerMotor, extensionMotor, secondaryExtensionMotor;

  private final PositionVoltage leftIntakeRequest = new PositionVoltage(0.0);
  private final PositionVoltage rightIntakeRequest = new PositionVoltage(0.0);
  private double leftWantedPosition = 0.0;
  private double rightWantedPosition = 0.0;
  private double rollerTargetSpeed = 0.0;
  
  // Add motor status tracking
  private boolean rollerMotorInitialized = false;
  private boolean secondaryRollerMotorInitialized = false;
  private boolean extensionMotorInitialized = false;
  private boolean secondaryExtensionMotorInitialized = false;

  public IntakeIO_REAL() {
      // Initialize motors
      rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
      secondaryRollerMotor = new TalonFX(IntakeConstants.secondaryRollerMotorID);
      extensionMotor = new TalonFX(IntakeConstants.extensionMotorID);
      secondaryExtensionMotor = new TalonFX(IntakeConstants.secondaryExtensionMotorID);

      // Test motor connectivity by checking if we can get device ID
      rollerMotorInitialized = rollerMotor.getDeviceID() == IntakeConstants.rollerMotorID;
      secondaryRollerMotorInitialized = secondaryRollerMotor.getDeviceID() == IntakeConstants.secondaryRollerMotorID;
      extensionMotorInitialized = extensionMotor.getDeviceID() == IntakeConstants.extensionMotorID;
      secondaryExtensionMotorInitialized = secondaryExtensionMotor.getDeviceID() == IntakeConstants.secondaryExtensionMotorID;

      rollerMotor.getConfigurator().apply(IntakeConstants.rollerMotorConfig);
      secondaryRollerMotor.getConfigurator().apply(IntakeConstants.secondaryRollerMotorConfig);
      extensionMotor.getConfigurator().apply(IntakeConstants.extensionMotorConfig);
      secondaryExtensionMotor.getConfigurator().apply(IntakeConstants.extensionMotorConfig);

      rollerMotor.optimizeBusUtilization();
      secondaryRollerMotor.optimizeBusUtilization();
      extensionMotor.optimizeBusUtilization();
      secondaryExtensionMotor.optimizeBusUtilization();

      rezeroExtension();
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerTargetSpeed = speed;
    rollerMotor.set(speed);
    secondaryRollerMotor.set(speed);
  }

  @Override
  public void setExtensionVelocity(double velocity) {
    secondaryExtensionMotor.set(-velocity);
    extensionMotor.set(velocity);
  }

  @Override
  public void setExtensionPosition(IntakeConstants.IntakeState state) {
    leftWantedPosition = -state.position;
    rightWantedPosition = state.position;
    leftIntakeRequest.Position = leftWantedPosition; // Position in rotations of left motor
    rightIntakeRequest.Position = rightWantedPosition;
    secondaryExtensionMotor.setControl(leftIntakeRequest);
    extensionMotor.setControl(rightIntakeRequest);
  }

  @Override
  public void setExtensionPosition(double position) {
    leftWantedPosition = -position;
    leftIntakeRequest.Position = leftWantedPosition; // Position in rotations of left motor
    rightWantedPosition = position;
    rightIntakeRequest.Position = rightWantedPosition; // Position in rotations of left motor
    secondaryExtensionMotor.setControl(leftIntakeRequest);
    extensionMotor.setControl(rightIntakeRequest);
  }

  @Override
  public void rezeroExtension() {
    extensionMotor.setPosition(0.0);
    secondaryExtensionMotor.setPosition(0.0);
  }

  @Override
  public void leftExtension() {
    extensionMotor.set(0.2);

  }

  @Override
  public void rightExtension() {
    secondaryExtensionMotor.set(-0.2);
  }

  @Override
  public void zeroExtentsion() {
    extensionMotor.set(0.0);
    secondaryExtensionMotor.set(0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60.0; // RPM
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerTargetSpeed = rollerTargetSpeed;

    inputs.extensionPosition = extensionMotor.getPosition().getValueAsDouble();
    inputs.extensionAppliedVolts = extensionMotor.getMotorVoltage().getValueAsDouble();

    inputs.secondaryExtensionPosition = secondaryExtensionMotor.getPosition().getValueAsDouble();
    inputs.secondaryExtensionAppliedVolts = secondaryExtensionMotor.getMotorVoltage().getValueAsDouble();

    inputs.rollerMotorTemp = (rollerMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    inputs.secondaryRollerMotorTemp = (secondaryRollerMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    inputs.extensionMotorTemp = (extensionMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    inputs.secondaryExtensionMotorTemp = (secondaryExtensionMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    
    // Update motor connection status
    inputs.rollerMotorConnected = rollerMotorInitialized;
    inputs.secondaryRollerMotorConnected = secondaryRollerMotorInitialized;
    inputs.extensionMotorConnected = extensionMotorInitialized;
    inputs.secondaryExtensionMotorConnected = secondaryExtensionMotorInitialized;
  }
}
