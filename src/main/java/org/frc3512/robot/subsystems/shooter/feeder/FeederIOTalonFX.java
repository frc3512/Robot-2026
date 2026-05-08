package org.frc3512.robot.subsystems.shooter.feeder;

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
 * Feeder IO implementation for Talon FX motor controllers.
 * Two Kraken X60 motors: one feeding into shooter tower, one booster below drum.
 */
public class FeederIOTalonFX implements FeederIO {
  // Hardware objects
  private final TalonFX feedMotor;
  private final TalonFX boosterMotor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Status signals (declared after constructor to reference initialized fields)
  private final StatusSignal<Angle> feedPosition;
  private final StatusSignal<AngularVelocity> feedVelocity;
  private final StatusSignal<Voltage> feedAppliedVolts;
  private final StatusSignal<Current> feedCurrent;
  private final StatusSignal<Temperature> feedTemp;

  private final StatusSignal<Angle> boosterPosition;
  private final StatusSignal<AngularVelocity> boosterVelocity;
  private final StatusSignal<Voltage> boosterAppliedVolts;
  private final StatusSignal<Current> boosterCurrent;
  private final StatusSignal<Temperature> boosterTemp;

  public FeederIOTalonFX(int feedMotorId, int boosterMotorId) {
    feedMotor = new TalonFX(feedMotorId);
    boosterMotor = new TalonFX(boosterMotorId);

    // Initialize status signals after hardware objects are created
    feedPosition = feedMotor.getPosition();
    feedVelocity = feedMotor.getVelocity();
    feedAppliedVolts = feedMotor.getMotorVoltage();
    feedCurrent = feedMotor.getStatorCurrent();
    feedTemp = feedMotor.getDeviceTemp();

    boosterPosition = boosterMotor.getPosition();
    boosterVelocity = boosterMotor.getVelocity();
    boosterAppliedVolts = boosterMotor.getMotorVoltage();
    boosterCurrent = boosterMotor.getStatorCurrent();
    boosterTemp = boosterMotor.getDeviceTemp();

    // Configure feed motor
    var feedConfig = new TalonFXConfiguration();
    feedConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    feedConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feedConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feedConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feedConfig.Slot0.kP = 0.1;
    feedConfig.Slot0.kV = 0.12;
    feedConfig.Voltage.PeakForwardVoltage = 12.0;
    feedConfig.Voltage.PeakReverseVoltage = -12.0;

    // Configure booster motor
    var boosterConfig = new TalonFXConfiguration();
    boosterConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    boosterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    boosterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    boosterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    boosterConfig.Slot0.kP = 0.1;
    boosterConfig.Slot0.kV = 0.12;
    boosterConfig.Voltage.PeakForwardVoltage = 12.0;
    boosterConfig.Voltage.PeakReverseVoltage = -12.0;

    // Apply configurations
    tryUntilOk(5, () -> feedMotor.getConfigurator().apply(feedConfig));
    tryUntilOk(5, () -> boosterMotor.getConfigurator().apply(boosterConfig));

    // Optimize status signal rates
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        feedPosition,
        feedVelocity,
        feedAppliedVolts,
        feedCurrent,
        feedTemp,
        boosterPosition,
        boosterVelocity,
        boosterAppliedVolts,
        boosterCurrent,
        boosterTemp);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.feedConnected = BaseStatusSignal.refreshAll(feedPosition, feedVelocity, feedAppliedVolts, feedCurrent, feedTemp).isOK();
    inputs.feedVelocityRPS = feedVelocity.getValueAsDouble();
    inputs.feedAppliedVolts = feedAppliedVolts.getValueAsDouble();
    inputs.feedCurrentAmps = feedCurrent.getValueAsDouble();
    inputs.feedTempCelsius = feedTemp.getValueAsDouble();

    inputs.boosterConnected = BaseStatusSignal.refreshAll(boosterPosition, boosterVelocity, boosterAppliedVolts, boosterCurrent, boosterTemp).isOK();
    inputs.boosterVelocityRPS = boosterVelocity.getValueAsDouble();
    inputs.boosterAppliedVolts = boosterAppliedVolts.getValueAsDouble();
    inputs.boosterCurrentAmps = boosterCurrent.getValueAsDouble();
    inputs.boosterTempCelsius = boosterTemp.getValueAsDouble();
  }

  @Override
  public void setFeedVelocity(double mechanismRPM) {
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
    feedMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setFeedOpenLoop(double output) {
    feedMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setBoosterVelocity(double mechanismRPM) {
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Feeder.FEED_MOTOR_RPM_PER_MECHANISM_RPM;
    boosterMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setBoosterOpenLoop(double output) {
    boosterMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void stop() {
    feedMotor.setControl(voltageRequest.withOutput(0.0));
    boosterMotor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setCurrentLimits(double currentLimitAmps) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> feedMotor.getConfigurator().apply(config));
    tryUntilOk(5, () -> boosterMotor.getConfigurator().apply(config));
  }

  @Override
  public void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = supplyLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> feedMotor.getConfigurator().apply(config));
    tryUntilOk(5, () -> boosterMotor.getConfigurator().apply(config));
  }
}
