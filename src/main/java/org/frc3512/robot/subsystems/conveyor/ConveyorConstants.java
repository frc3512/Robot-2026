package org.frc3512.robot.subsystems.conveyor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ConveyorConstants {

  public static final int hopperMotorID = 17;

  public static final TalonFXConfiguration hopper =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(30)
                  .withSupplyCurrentLimitEnable(true));
}
