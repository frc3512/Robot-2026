package org.frc3512.robot.subsystems.intake;

import org.frc3512.robot.subsystems.intake.IntakeConstants.IntakePosition;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class IntakeIO_REAL implements IntakeIO {

  private TalonFX rollerMotor, secondaryRollerMotor, extensionMotor;

  private final PositionVoltage intakeRequest = new PositionVoltage(0.0);
  private double wantedPosition = 0.0;
  private double rollerTargetSpeed = 0.0;

  private double lastExtensionPosition = 0.0;
  private double lastExtensionTimestamp = 0.0;

  private Timer feedingTimer = new Timer();
  
  // Add motor status tracking
  private boolean rollerMotorInitialized = false;
  private boolean secondaryRollerMotorInitialized = false;
  private boolean extensionMotorInitialized = false;

  public IntakeIO_REAL() {
      // Initialize motors
      rollerMotor = new TalonFX(IntakeConstants.rollerMotorID);
      secondaryRollerMotor = new TalonFX(IntakeConstants.secondaryRollerMotorID);
      extensionMotor = new TalonFX(IntakeConstants.extensionMotorID);

      // Test motor connectivity by checking if we can get device ID
      rollerMotorInitialized = rollerMotor.getDeviceID() == IntakeConstants.rollerMotorID;
      secondaryRollerMotorInitialized = secondaryRollerMotor.getDeviceID() == IntakeConstants.secondaryRollerMotorID;
      extensionMotorInitialized = extensionMotor.getDeviceID() == IntakeConstants.extensionMotorID;

      rollerMotor.getConfigurator().apply(IntakeConstants.rollerMotorConfig);
      secondaryRollerMotor.getConfigurator().apply(IntakeConstants.secondaryRollerMotorConfig);
      extensionMotor.getConfigurator().apply(IntakeConstants.extensionMotorConfig);

      rollerMotor.optimizeBusUtilization();
      secondaryRollerMotor.optimizeBusUtilization();
      extensionMotor.optimizeBusUtilization();

      rezeroExtension();

      feedingTimer.start();
  }

  @Override
  public void setRollerSpeed(double speed) {
    rollerTargetSpeed = speed;
    rollerMotor.set(speed);
    secondaryRollerMotor.set(speed);
  }

  @Override
  public void setExtensionVelocity(double velocity) {
    extensionMotor.set(velocity);
  }

  @Override
  public void setExtensionPosition(IntakeConstants.IntakePosition position) {
    wantedPosition = position.position;
    intakeRequest.Position = wantedPosition; // Position in rotations of motor
    extensionMotor.setControl(intakeRequest);
  }

  @Override
  public void setExtensionPosition(double position) {
    wantedPosition = position;
    intakeRequest.Position = wantedPosition; // Position in rotations of motor
    extensionMotor.setControl(intakeRequest);
  }

  @Override
  public void rezeroExtension() {
    extensionMotor.setPosition(0.0);
  }

  public boolean isExtensionStalled() {
    // Consider stalled if current is above threshold and velocity is very low
    return extensionMotor.getStatorCurrent().getValueAsDouble() > 1.0 && Math.abs(getExtensionVelocity()) < 0.1;
  }

  public double getExtensionVelocity() {
    // Calculate velocity from position change
    double currentTime = Timer.getFPGATimestamp();
    double deltaTime = currentTime - lastExtensionTimestamp;
    
    if (deltaTime > 0.0) {
      double velocity = (extensionMotor.getPosition().getValueAsDouble() - lastExtensionPosition) / deltaTime;
      return velocity;
    }
    return 0.0;
  }

  public void compressIntake() {
    setRollerSpeed(0.2);
    feedingTimer.reset();
    feedingTimer.start();
    double retractElapsed = feedingTimer.get();
    // Move from current position to STOWED over 2.0 seconds
    double retractDuration = 2.0;
    if (retractElapsed < retractDuration) {
        // Calculate interpolated position from current state to STOWED
        double progress = retractElapsed / retractDuration;
        double currentPosition = IntakePosition.EXTEND.position;
        double targetPosition = currentPosition * (1.0 - progress);
        
        // Set arbitrary position directly
        setExtensionPosition(targetPosition);
      } else {
        // Ensure we reach STOWED
        setExtensionPosition(IntakeConstants.IntakePosition.STOWED);
      }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerVelocity = rollerMotor.getVelocity().getValueAsDouble() * 60.0; // RPM
    inputs.rollerAppliedVolts = rollerMotor.getMotorVoltage().getValueAsDouble();
    inputs.rollerTargetSpeed = rollerTargetSpeed;

    inputs.extensionPosition = extensionMotor.getPosition().getValueAsDouble();
    inputs.extensionAppliedVolts = extensionMotor.getMotorVoltage().getValueAsDouble();

    inputs.rollerMotorTemp = (rollerMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    inputs.secondaryRollerMotorTemp = (secondaryRollerMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    inputs.extensionMotorTemp = (extensionMotor.getDeviceTemp().getValueAsDouble() * 1.8) + 32.0;
    
    // Update motor connection status
    inputs.rollerMotorConnected = rollerMotorInitialized;
    inputs.secondaryRollerMotorConnected = secondaryRollerMotorInitialized;
    inputs.extensionMotorConnected = extensionMotorInitialized;

    // Update velocity tracking
    lastExtensionPosition = inputs.extensionPosition;
    lastExtensionTimestamp = Timer.getFPGATimestamp();
  }
}
