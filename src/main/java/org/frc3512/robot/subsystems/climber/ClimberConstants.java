package org.frc3512.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberConstants {

  public static final int climberMotorID = 23;

  public static final int topLimitID = 0;
  public static final int bottomLimitID = 1;

  public static final TalonFXConfiguration climber =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake));
}
