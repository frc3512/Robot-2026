package org.frc3512.robot.subsystems.hopper.intake;

import static org.frc3512.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Intake IO implementation for Talon FX motor controllers.
 * Three Kraken X60 motors: two rollers and one extension motor.
 */
public class IntakeIOTalonFX implements IntakeIO {
  // Hardware objects
  private final TalonFX roller1;
  private final TalonFX roller2;
  private final TalonFX extension;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Status signals (declared after constructor to reference initialized fields)
  private final StatusSignal<Angle> roller1Position;
  private final StatusSignal<AngularVelocity> roller1Velocity;
  private final StatusSignal<Voltage> roller1AppliedVolts;
  private final StatusSignal<Current> roller1Current;
  private final StatusSignal<Temperature> roller1Temp;

  private final StatusSignal<Angle> roller2Position;
  private final StatusSignal<AngularVelocity> roller2Velocity;
  private final StatusSignal<Voltage> roller2AppliedVolts;
  private final StatusSignal<Current> roller2Current;
  private final StatusSignal<Temperature> roller2Temp;

  private final StatusSignal<Angle> extensionPosition;
  private final StatusSignal<AngularVelocity> extensionVelocity;
  private final StatusSignal<Voltage> extensionAppliedVolts;
  private final StatusSignal<Current> extensionCurrent;
  private final StatusSignal<Temperature> extensionTemp;

  public IntakeIOTalonFX(
      int roller1Id,
      int roller2Id,
      int extensionId) {
    roller1 = new TalonFX(roller1Id);
    roller2 = new TalonFX(roller2Id);
    extension = new TalonFX(extensionId);

    // Initialize status signals after hardware objects are created
    roller1Position = roller1.getPosition();
    roller1Velocity = roller1.getVelocity();
    roller1AppliedVolts = roller1.getMotorVoltage();
    roller1Current = roller1.getStatorCurrent();
    roller1Temp = roller1.getDeviceTemp();

    roller2Position = roller2.getPosition();
    roller2Velocity = roller2.getVelocity();
    roller2AppliedVolts = roller2.getMotorVoltage();
    roller2Current = roller2.getStatorCurrent();
    roller2Temp = roller2.getDeviceTemp();

    extensionPosition = extension.getPosition();
    extensionVelocity = extension.getVelocity();
    extensionAppliedVolts = extension.getMotorVoltage();
    extensionCurrent = extension.getStatorCurrent();
    extensionTemp = extension.getDeviceTemp();

    // Configure roller motors
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.Slot0.kP = 0.1;
    rollerConfig.Slot0.kV = 0.12;
    rollerConfig.Voltage.PeakForwardVoltage = 12.0;
    rollerConfig.Voltage.PeakReverseVoltage = -12.0;

    // Configure extension motor
    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0.kP = 2.0;
    extensionConfig.Slot0.kI = 0.0;
    extensionConfig.Slot0.kD = 0.1;
    extensionConfig.Voltage.PeakForwardVoltage = 12.0;
    extensionConfig.Voltage.PeakReverseVoltage = -12.0;

    // Apply configurations
    tryUntilOk(5, () -> roller1.getConfigurator().apply(rollerConfig));
    tryUntilOk(5, () -> roller2.getConfigurator().apply(rollerConfig));
    tryUntilOk(5, () -> extension.getConfigurator().apply(extensionConfig));

    // Optimize status signal rates
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        roller1Position,
        roller1Velocity,
        roller1AppliedVolts,
        roller1Current,
        roller1Temp,
        roller2Position,
        roller2Velocity,
        roller2AppliedVolts,
        roller2Current,
        roller2Temp,
        extensionPosition,
        extensionVelocity,
        extensionAppliedVolts,
        extensionCurrent,
        extensionTemp);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.roller1Connected = BaseStatusSignal.refreshAll(roller1Position, roller1Velocity, roller1AppliedVolts, roller1Current, roller1Temp).isOK();
    inputs.roller1VelocityRPS = roller1Velocity.getValueAsDouble();
    inputs.roller1AppliedVolts = roller1AppliedVolts.getValueAsDouble();
    inputs.roller1CurrentAmps = roller1Current.getValueAsDouble();
    inputs.roller1TempCelsius = roller1Temp.getValueAsDouble();

    inputs.roller2Connected = BaseStatusSignal.refreshAll(roller2Position, roller2Velocity, roller2AppliedVolts, roller2Current, roller2Temp).isOK();
    inputs.roller2VelocityRPS = roller2Velocity.getValueAsDouble();
    inputs.roller2AppliedVolts = roller2AppliedVolts.getValueAsDouble();
    inputs.roller2CurrentAmps = roller2Current.getValueAsDouble();
    inputs.roller2TempCelsius = roller2Temp.getValueAsDouble();

    inputs.extensionConnected = BaseStatusSignal.refreshAll(extensionPosition, extensionVelocity, extensionAppliedVolts, extensionCurrent, extensionTemp).isOK();
    inputs.extensionPositionInches = rotationsToInches(extensionPosition.getValueAsDouble());
    inputs.extensionVelocityInchesPerSec = rpsToInchesPerSec(extensionVelocity.getValueAsDouble());
    inputs.extensionAppliedVolts = extensionAppliedVolts.getValueAsDouble();
    inputs.extensionCurrentAmps = extensionCurrent.getValueAsDouble();
    inputs.extensionTempCelsius = extensionTemp.getValueAsDouble();
  }

  @Override
  public void setRollerVelocity(double mechanismRPM) {
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Intake.ROLLER_MOTOR_RPM_PER_MECHANISM_RPM;
    roller1.setControl(velocityRequest.withVelocity(motorRPS));
    roller2.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setRollerOpenLoop(double output) {
    roller1.setControl(voltageRequest.withOutput(output));
    roller2.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setExtensionPosition(double positionInches) {
    double motorRotations = inchesToRotations(positionInches);
    extension.setControl(velocityRequest.withVelocity(motorRotations));
  }

  @Override
  public void setExtensionOpenLoop(double output) {
    extension.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void stop() {
    roller1.setControl(voltageRequest.withOutput(0.0));
    roller2.setControl(voltageRequest.withOutput(0.0));
    extension.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {
    var rollerConfig = new TalonFXConfiguration();
    rollerConfig.CurrentLimits.SupplyCurrentLimit = supplyLimitAmps;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.StatorCurrentLimit = statorLimitAmps;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> roller1.getConfigurator().apply(rollerConfig));
    tryUntilOk(5, () -> roller2.getConfigurator().apply(rollerConfig));
    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.CurrentLimits.SupplyCurrentLimit = supplyLimitAmps;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimit = statorLimitAmps;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> extension.getConfigurator().apply(extensionConfig));
  }

  // Helper methods for position conversion (TODO: Update with actual gear ratios)
  private double rotationsToInches(double rotations) {
    return rotations * 2.0 * Math.PI * 1.0; // TODO: Update with actual drum radius
  }

  private double inchesToRotations(double inches) {
    return inches / (2.0 * Math.PI * 1.0); // TODO: Update with actual drum radius
  }

  private double rpsToInchesPerSec(double rps) {
    return rps * 2.0 * Math.PI * 1.0; // TODO: Update with actual drum radius
  }
}
