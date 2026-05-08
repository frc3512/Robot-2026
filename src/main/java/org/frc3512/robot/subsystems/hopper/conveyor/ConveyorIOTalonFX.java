package org.frc3512.robot.subsystems.hopper.conveyor;

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
 * Conveyor IO implementation for Talon FX motor controller.
 * One Kraken X44 motor driving a roller floor.
 */
public class ConveyorIOTalonFX implements ConveyorIO {
  // Hardware object
  private final TalonFX motor;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  // Status signals (declared after constructor to reference initialized field)
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temp;

  public ConveyorIOTalonFX(int motorId) {
    motor = new TalonFX(motorId);

    // Initialize status signals after hardware object is created
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
    temp = motor.getDeviceTemp();

    // Configure motor
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 20.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = 0.1;
    config.Slot0.kV = 0.12;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config));

    // Optimize status signal rates
    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        position,
        velocity,
        appliedVolts,
        current,
        temp);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temp).isOK();
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setVelocity(double mechanismRPM) {
    double motorRPS = mechanismRPM / 60.0 * org.frc3512.robot.constants.MechanicalConstants.Conveyor.ROLLER_MOTOR_RPS_PER_MECHANISM_RPM;
    motor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  @Override
  public void setOpenLoop(double output) {
    motor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void stop() {
    motor.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setCurrentLimit(double currentLimitAmps) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = currentLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  @Override
  public void setCurrentLimits(double supplyLimitAmps, double statorLimitAmps) {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = supplyLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = statorLimitAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    
    tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }
}
