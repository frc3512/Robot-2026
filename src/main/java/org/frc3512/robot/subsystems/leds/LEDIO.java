package org.frc3512.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public double[] ledBuffer = new double[0];
    public int patternLength = 0;
    public String currentPattern = "OFF";
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LEDIOInputs inputs) {}

  /** Sets the LED buffer to a solid color */
  default void setSolidColor(int r, int g, int b) {}

  /** Sets the LED buffer to a pattern */
  default void setPattern(double[] buffer, int length) {}

  /** Sets all LEDs to off */
  default void setOff() {}

  /** Gets the number of LEDs */
  default int getLength() { return 0; }
}
