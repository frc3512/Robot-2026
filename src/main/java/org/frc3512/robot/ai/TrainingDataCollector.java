package org.frc3512.robot.ai;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.littletonrobotics.junction.Logger;

/**
 * Collects training data for AI shooting model.
 * Captures shooting parameters, results, and environmental conditions.
 */
public class TrainingDataCollector {
  private static final String DATA_DIR = "/tmp/robot_ai_data";
  private static final String FILE_PREFIX = "shooting_data_";
  
  private final ConcurrentLinkedQueue<ShootingDataPoint> dataBuffer = new ConcurrentLinkedQueue<>();
  private final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
  private final int maxBufferSize = 1000;
  private final long dataSaveIntervalMs = 30000; // Save every 30 seconds
  private long lastSaveTime = 0;
  
  private boolean collecting = false;
  private String currentSessionId;

  /**
   * Data structure for a single shooting data point.
   */
  public static class ShootingDataPoint {
    // Input features
    public final double distanceToHub;
    public final double robotVelocityX;
    public final double robotVelocityY;
    public final double radialVelocity;
    public final double tangentialVelocity;
    public final double batteryVoltage;
    public final double drumTemperature;
    public final double hoodAngle;
    public final long timestamp;
    
    // Target outputs (what we want to predict)
    public final double targetRPM;
    public final double targetHoodAngle;
    
    // Results (for supervised learning)
    public final boolean shotSuccessful;
    public final double actualDistanceError;
    public final double shotAccuracy;
    
    public ShootingDataPoint(
        double distanceToHub, double robotVelocityX, double robotVelocityY,
        double radialVelocity, double tangentialVelocity, double batteryVoltage,
        double drumTemperature, double hoodAngle, long timestamp,
        double targetRPM, double targetHoodAngle, boolean shotSuccessful,
        double actualDistanceError, double shotAccuracy) {
      
      this.distanceToHub = distanceToHub;
      this.robotVelocityX = robotVelocityX;
      this.robotVelocityY = robotVelocityY;
      this.radialVelocity = radialVelocity;
      this.tangentialVelocity = tangentialVelocity;
      this.batteryVoltage = batteryVoltage;
      this.drumTemperature = drumTemperature;
      this.hoodAngle = hoodAngle;
      this.timestamp = timestamp;
      
      this.targetRPM = targetRPM;
      this.targetHoodAngle = targetHoodAngle;
      this.shotSuccessful = shotSuccessful;
      this.actualDistanceError = actualDistanceError;
      this.shotAccuracy = shotAccuracy;
    }
    
    /**
     * Converts data point to CSV format.
     */
    public String toCSV() {
      return String.format(
          "%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.1f,%.2f,%d,%.1f,%.1f,%b,%.4f,%.3f",
          distanceToHub, robotVelocityX, robotVelocityY, radialVelocity, tangentialVelocity,
          batteryVoltage, drumTemperature, hoodAngle, timestamp, targetRPM, targetHoodAngle,
          shotSuccessful, actualDistanceError, shotAccuracy
      );
    }
    
    /**
     * Gets CSV header.
     */
    public static String getCSVHeader() {
      return "distance_to_hub,robot_velocity_x,robot_velocity_y,radial_velocity,tangential_velocity," +
             "battery_voltage,drum_temperature,hood_angle,timestamp,target_rpm,target_hood_angle," +
             "shot_successful,actual_distance_error,shot_accuracy";
    }
  }

  /**
   * Starts data collection for a new session.
   */
  public void startCollection() {
    if (!collecting) {
      collecting = true;
      currentSessionId = dateFormat.format(new Date());
      dataBuffer.clear();
      lastSaveTime = System.currentTimeMillis();
      
      // Create data directory
      try {
        Files.createDirectories(Paths.get(DATA_DIR));
      } catch (IOException e) {
        Logger.recordOutput("AI/DataCollector/Error", "Failed to create data directory: " + e.getMessage());
      }
      
      Logger.recordOutput("AI/DataCollector/Collecting", true);
      Logger.recordOutput("AI/DataCollector/SessionId", currentSessionId);
    }
  }

  /**
   * Stops data collection and saves remaining data.
   */
  public void stopCollection() {
    if (collecting) {
      collecting = false;
      saveBufferedData();
      Logger.recordOutput("AI/DataCollector/Collecting", false);
    }
  }

  /**
   * Adds a shooting data point to the collection buffer.
   */
  public void addDataPoint(ShootingDataPoint dataPoint) {
    if (!collecting) return;
    
    dataBuffer.offer(dataPoint);
    
    // Save buffer if it's full or enough time has passed
    long currentTime = System.currentTimeMillis();
    if (dataBuffer.size() >= maxBufferSize || (currentTime - lastSaveTime) >= dataSaveIntervalMs) {
      saveBufferedData();
      lastSaveTime = currentTime;
    }
  }

  /**
   * Saves buffered data to file.
   */
  private void saveBufferedData() {
    if (dataBuffer.isEmpty()) return;
    
    List<ShootingDataPoint> dataToSave = new ArrayList<>();
    while (!dataBuffer.isEmpty()) {
      ShootingDataPoint point = dataBuffer.poll();
      if (point != null) {
        dataToSave.add(point);
      }
    }
    
    if (dataToSave.isEmpty()) return;
    
    String filename = String.format("%s/%s%s.csv", DATA_DIR, FILE_PREFIX, currentSessionId);
    boolean fileExists = Files.exists(Paths.get(filename));
    
    try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename, true))) {
      // Write header if file is new
      if (!fileExists) {
        writer.write(ShootingDataPoint.getCSVHeader());
        writer.newLine();
      }
      
      // Write data points
      for (ShootingDataPoint point : dataToSave) {
        writer.write(point.toCSV());
        writer.newLine();
      }
      
      Logger.recordOutput("AI/DataCollector/LastSaveFile", filename);
      Logger.recordOutput("AI/DataCollector/PointsSaved", dataToSave.size());
      
    } catch (IOException e) {
      Logger.recordOutput("AI/DataCollector/Error", "Failed to save data: " + e.getMessage());
    }
  }

  /**
   * Gets the current collection status.
   */
  public boolean isCollecting() {
    return collecting;
  }

  /**
   * Gets the current session ID.
   */
  public String getCurrentSessionId() {
    return currentSessionId;
  }

  /**
   * Gets the buffer size.
   */
  public int getBufferSize() {
    return dataBuffer.size();
  }

  /**
   * Gets all data files in the data directory.
   */
  public List<String> getDataFiles() {
    List<String> files = new ArrayList<>();
    try {
      Files.list(Paths.get(DATA_DIR))
          .filter(path -> path.toString().endsWith(".csv"))
          .sorted()
          .forEach(path -> files.add(path.toString()));
    } catch (IOException e) {
      Logger.recordOutput("AI/DataCollector/Error", "Failed to list data files: " + e.getMessage());
    }
    return files;
  }

  /**
   * Periodic method to handle automatic saving.
   */
  public void periodic() {
    if (collecting) {
      long currentTime = System.currentTimeMillis();
      if ((currentTime - lastSaveTime) >= dataSaveIntervalMs) {
        saveBufferedData();
        lastSaveTime = currentTime;
      }
      
      Logger.recordOutput("AI/DataCollector/BufferSize", dataBuffer.size());
      Logger.recordOutput("AI/DataCollector/TimeSinceLastSave", (currentTime - lastSaveTime) / 1000.0);
    }
  }
}
