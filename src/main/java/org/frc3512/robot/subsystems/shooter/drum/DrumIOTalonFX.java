package org.frc3512.robot.subsystems.shooter.drum;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.frc3512.robot.util.PhoenixUtil.tryUntilOk;

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
 * Drum IO implementation for Talon FX motor controllers.
 * Three Kraken X60 motors: two on one side, one on other side.
 * 1:1 gear ratio, 2-inch drum radius.
 */
public class DrumIOTalonFX implements DrumIO {
  // Hardware objects
  private final TalonFX motor1;
  private final TalonFX motor2;
  private final TalonFX motor3;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Status signals (declared after constructor to reference initialized fields)
  private final StatusSignal<Angle> motor1Position;
  private final StatusSignal<AngularVelocity> motor1Velocity;
  private final StatusSignal<Voltage> motor1AppliedVolts;
  private final StatusSignal<Current> motor1Current;
  private final StatusSignal<Temperature> motor1Temp;

  private final StatusSignal<Angle> motor2Position;
  private final StatusSignal<AngularVelocity> motor2Velocity;
  private final StatusSignal<Voltage> motor2AppliedVolts;
  private final StatusSignal<Current> motor2Current;
  private final StatusSignal<Temperature> motor2Temp;

  private final StatusSignal<Angle> motor3Position;
  private final StatusSignal<AngularVelocity> motor3Velocity;
  private final StatusSignal<Voltage> motor3AppliedVolts;
  private final StatusSignal<Current> motor3Current;
  private final StatusSignal<Temperature> motor3Temp;

  public DrumIOTalonFX(int motor1Id, int motor2Id, int motor3Id) {
    motor1 = new TalonFX(motor1Id);
    motor2 = new TalonFX(motor2Id);
    motor3 = new TalonFX(motor3Id);

    // Initialize status signals after hardware objects are created
    motor1Position = motor1.getPosition();
    motor1Velocity = motor1.getVelocity();
    motor1AppliedVolts = motor1.getMotorVoltage();
    motor1Current = motor1.getStatorCurrent();
    motor1Temp = motor1.getDeviceTemp();

    motor2Position = motor2.getPosition();
    motor2Velocity = motor2.getVelocity();
    motor2AppliedVolts = motor2.getMotorVoltage();
    motor2Current = motor2.getStatorCurrent();
    motor2Temp = motor2.getDeviceTemp();

    motor3Position = motor3.getPosition();
    motor3Velocity = motor3.getVelocity();
    motor3AppliedVolts = motor3.getMotorVoltage();
    motor3Current = motor3.getStatorCurrent();
    motor3Temp = motor3.getDeviceTemp();

    // Configure motors 1 and 2 with same settings
    var configA = new TalonFXConfiguration();
    configA.CurrentLimits.SupplyCurrentLimit = 40.0;
    configA.CurrentLimits.SupplyCurrentLimitEnable = true;
    configA.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configA.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configA.Slot0.kP = 0.25;
    configA.Slot0.kV = 12.0 / RPM.of(6000.0).in(RotationsPerSecond);
    configA.Voltage.PeakReverseVoltage = 0;

    // Configure motor 3 with inverted direction
    var configB = new TalonFXConfiguration();
    configB.CurrentLimits.SupplyCurrentLimit = 40.0;
    configB.CurrentLimits.SupplyCurrentLimitEnable = true;
    configB.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverted from motors 1&2
    configB.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configB.Slot0.kP = 0.25;
    configB.Slot0.kV = 12.0 / RPM.of(6000.0).in(RotationsPerSecond);
    configB.Voltage.PeakReverseVoltage = 0;

    // Apply configurations
    tryUntilOk(5, () -> motor1.getConfigurator().apply(configA));
    tryUntilOk(5, () -> motor2.getConfigurator().apply(configA));
    tryUntilOk(5, () -> motor3.getConfigurator().apply(configB));

    // Optimize status signal rates
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        motor1Position, motor1Velocity, motor1AppliedVolts, motor1Current, motor1Temp,
        motor2Position, motor2Velocity, motor2AppliedVolts, motor2Current, motor2Temp,
        motor3Position, motor3Velocity, motor3AppliedVolts, motor3Current, motor3Temp);
  }

  @Override
  public void updateInputs(DrumIO.DrumIOInputs inputs) {
    // Update motor 1
    inputs.motor1Connected = BaseStatusSignal.refreshAll(motor1Position, motor1Velocity, motor1AppliedVolts, motor1Current, motor1Temp).isOK();
    inputs.motor1VelocityRPS = motor1Velocity.getValueAsDouble();
    inputs.motor1AppliedVolts = motor1AppliedVolts.getValueAsDouble();
    inputs.motor1CurrentAmps = motor1Current.getValueAsDouble();
    inputs.motor1TempCelsius = motor1Temp.getValueAsDouble();

    // Update motor 2
    inputs.motor2Connected = BaseStatusSignal.refreshAll(motor2Position, motor2Velocity, motor2AppliedVolts, motor2Current, motor2Temp).isOK();
    inputs.motor2VelocityRPS = motor2Velocity.getValueAsDouble();
    inputs.motor2AppliedVolts = motor2AppliedVolts.getValueAsDouble();
    inputs.motor2CurrentAmps = motor2Current.getValueAsDouble();
    inputs.motor2TempCelsius = motor2Temp.getValueAsDouble();

    // Update motor 3
    inputs.motor3Connected = BaseStatusSignal.refreshAll(motor3Position, motor3Velocity, motor3AppliedVolts, motor3Current, motor3Temp).isOK();
    inputs.motor3VelocityRPS = motor3Velocity.getValueAsDouble();
    inputs.motor3AppliedVolts = motor3AppliedVolts.getValueAsDouble();
    inputs.motor3CurrentAmps = motor3Current.getValueAsDouble();
    inputs.motor3TempCelsius = motor3Temp.getValueAsDouble();
  }

  @Override
  public void setVelocity(double mechanismRPM) {
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Drum.GEAR_RATIO;
    motor1.setControl(velocityRequest.withVelocity(motorRPS));
    motor2.setControl(velocityRequest.withVelocity(motorRPS));
    motor3.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setOpenLoop(double output) {
    motor1.setControl(voltageRequest.withOutput(output));
    motor2.setControl(voltageRequest.withOutput(output));
    motor3.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void stop() {
    motor1.setControl(voltageRequest.withOutput(0.0));
    motor2.setControl(voltageRequest.withOutput(0.0));
    motor3.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = supplyLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> motor1.getConfigurator().apply(config));
    tryUntilOk(5, () -> motor2.getConfigurator().apply(config));
    tryUntilOk(5, () -> motor3.getConfigurator().apply(config));
  }
}
