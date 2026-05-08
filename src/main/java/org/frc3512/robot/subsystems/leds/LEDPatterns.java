package org.frc3512.robot.subsystems.leds;

public class LEDPatterns {
  public static final int LED_COUNT = 60; // Adjust based on your LED strip length
  
  // Colors
  public static final int[] OFF = {0, 0, 0};
  public static final int[] BLUE = {0, 0, 255};
  public static final int[] RED = {255, 0, 0};
  public static final int[] GREEN = {0, 255, 0};
  public static final int[] YELLOW = {255, 255, 0};
  public static final int[] PURPLE = {255, 0, 255};
  public static final int[] CYAN = {0, 255, 255};
  public static final int[] WHITE = {255, 255, 255};
  public static final int[] ORANGE = {255, 165, 0};

  /**
   * Creates a breathing pattern (fade in and out)
   */
  public static double[] breathingPattern(int[] color, int length, double phase) {
    double[] pattern = new double[length * 3];
    double brightness = (Math.sin(phase) + 1.0) / 2.0; // 0.0 to 1.0
    
    for (int i = 0; i < length; i++) {
      pattern[i * 3] = (color[0] / 255.0) * brightness;
      pattern[i * 3 + 1] = (color[1] / 255.0) * brightness;
      pattern[i * 3 + 2] = (color[2] / 255.0) * brightness;
    }
    return pattern;
  }

  /**
   * Creates a rainbow pattern
   */
  public static double[] rainbowPattern(int length, double phase) {
    double[] pattern = new double[length * 3];
    
    for (int i = 0; i < length; i++) {
      double hue = ((i / (double) length) + phase) % 1.0;
      double[] rgb = hsvToRgb(hue, 1.0, 1.0);
      pattern[i * 3] = rgb[0];
      pattern[i * 3 + 1] = rgb[1];
      pattern[i * 3 + 2] = rgb[2];
    }
    return pattern;
  }

  /**
   * Creates a chase pattern (moving dots)
   */
  public static double[] chasePattern(int[] color, int length, int dotSize, double phase) {
    double[] pattern = new double[length * 3];
    int position = (int) (phase * length) % length;
    
    for (int i = 0; i < length; i++) {
      if ((i >= position && i < position + dotSize) || 
          (position + dotSize > length && i < (position + dotSize) % length)) {
        pattern[i * 3] = color[0] / 255.0;
        pattern[i * 3 + 1] = color[1] / 255.0;
        pattern[i * 3 + 2] = color[2] / 255.0;
      } else {
        pattern[i * 3] = 0.0;
        pattern[i * 3 + 1] = 0.0;
        pattern[i * 3 + 2] = 0.0;
      }
    }
    return pattern;
  }

  /**
   * Creates a flash pattern
   */
  public static double[] flashPattern(int[] color, int length, boolean on) {
    double[] pattern = new double[length * 3];
    double brightness = on ? 1.0 : 0.0;
    
    for (int i = 0; i < length; i++) {
      pattern[i * 3] = (color[0] / 255.0) * brightness;
      pattern[i * 3 + 1] = (color[1] / 255.0) * brightness;
      pattern[i * 3 + 2] = (color[2] / 255.0) * brightness;
    }
    return pattern;
  }

  /**
   * Creates a gradient pattern
   */
  public static double[] gradientPattern(int[] startColor, int[] endColor, int length) {
    double[] pattern = new double[length * 3];
    
    for (int i = 0; i < length; i++) {
      double ratio = i / (double) (length - 1);
      pattern[i * 3] = (startColor[0] / 255.0) * (1 - ratio) + (endColor[0] / 255.0) * ratio;
      pattern[i * 3 + 1] = (startColor[1] / 255.0) * (1 - ratio) + (endColor[1] / 255.0) * ratio;
      pattern[i * 3 + 2] = (startColor[2] / 255.0) * (1 - ratio) + (endColor[2] / 255.0) * ratio;
    }
    return pattern;
  }

  /**
   * Converts HSV to RGB
   */
  private static double[] hsvToRgb(double h, double s, double v) {
    double c = v * s;
    double x = c * (1 - Math.abs((h * 6) % 2 - 1));
    double m = v - c;
    
    double r, g, b;
    if (h < 1.0 / 6.0) {
      r = c; g = x; b = 0;
    } else if (h < 2.0 / 6.0) {
      r = x; g = c; b = 0;
    } else if (h < 3.0 / 6.0) {
      r = 0; g = c; b = x;
    } else if (h < 4.0 / 6.0) {
      r = 0; g = x; b = c;
    } else if (h < 5.0 / 6.0) {
      r = x; g = 0; b = c;
    } else {
      r = c; g = 0; b = x;
    }
    
    return new double[]{r + m, g + m, b + m};
  }
}
