package org.frc3512.robot.ai;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.littletonrobotics.junction.Logger;

/**
 * Training pipeline for the shooting neural network.
 * Handles data loading, preprocessing, training, and model evaluation.
 */

@SuppressWarnings("unused")
public class TrainingPipeline {
  private static final String DATA_DIR = "/tmp/robot_ai_data";
  private static final String MODEL_DIR = "/tmp/robot_ai_models";
  private static final int MAX_EPOCHS = 100;
  private static final double VALIDATION_SPLIT = 0.2;
  private static final int BATCH_SIZE = 32;
  private static final double EARLY_STOPPING_PATIENCE = 10;
  
  private final ShootingNeuralNetwork network;
  private final Random random;
  private boolean isTraining = false;
  private double bestValidationLoss = Double.MAX_VALUE;
  private int epochsWithoutImprovement = 0;

  public TrainingPipeline(ShootingNeuralNetwork network) {
    this.network = network;
    this.random = new Random(42); // Fixed seed for reproducibility
  }

  /**
   * Trains the network with all available data.
   */
  public TrainingResult trainNetwork() {
    if (isTraining) {
      return new TrainingResult(false, "Training already in progress", 0.0, 0.0);
    }
    
    try {
      isTraining = true;
      Logger.recordOutput("AI/TrainingPipeline/Status", "Starting training");
      
      // Load training data
      List<TrainingData> allData = loadAllTrainingData();
      if (allData.isEmpty()) {
        return new TrainingResult(false, "No training data available", 0.0, 0.0);
      }
      
      Logger.recordOutput("AI/TrainingPipeline/DataPoints", allData.size());
      
      // Split data into training and validation sets
      List<TrainingData> trainingData = new ArrayList<>();
      List<TrainingData> validationData = new ArrayList<>();
      
      shuffleData(allData);
      int splitIndex = (int) (allData.size() * (1.0 - VALIDATION_SPLIT));
      
      for (int i = 0; i < allData.size(); i++) {
        if (i < splitIndex) {
          trainingData.add(allData.get(i));
        } else {
          validationData.add(allData.get(i));
        }
      }
      
      Logger.recordOutput("AI/TrainingPipeline/TrainingPoints", trainingData.size());
      Logger.recordOutput("AI/TrainingPipeline/ValidationPoints", validationData.size());
      
      // Update normalization parameters
      List<double[]> inputs = new ArrayList<>();
      List<double[]> outputs = new ArrayList<>();
      for (TrainingData data : trainingData) {
        inputs.add(data.inputs);
        outputs.add(data.outputs);
      }
      network.updateNormalization(inputs, outputs);
      
      // Training loop
      bestValidationLoss = Double.MAX_VALUE;
      epochsWithoutImprovement = 0;
      double finalTrainingLoss = 0.0;
      double finalValidationLoss = 0.0;
      
      for (int epoch = 0; epoch < MAX_EPOCHS; epoch++) {
        // Train on batches
        double epochLoss = trainEpoch(trainingData);
        double validationLoss = evaluateModel(validationData);
        
        Logger.recordOutput("AI/TrainingPipeline/Epoch", epoch);
        Logger.recordOutput("AI/TrainingPipeline/TrainingLoss", epochLoss);
        Logger.recordOutput("AI/TrainingPipeline/ValidationLoss", validationLoss);
        
        // Check for improvement
        if (validationLoss < bestValidationLoss) {
          bestValidationLoss = validationLoss;
          epochsWithoutImprovement = 0;
          network.saveModel();
          Logger.recordOutput("AI/TrainingPipeline/BestModelSaved", true);
        } else {
          epochsWithoutImprovement++;
        }
        
        // Early stopping
        if (epochsWithoutImprovement >= EARLY_STOPPING_PATIENCE) {
          Logger.recordOutput("AI/TrainingPipeline/EarlyStopping", epoch);
          break;
        }
        
        finalTrainingLoss = epochLoss;
        finalValidationLoss = validationLoss;
      }
      
      // Load best model
      network.loadModel();
      
      isTraining = false;
      Logger.recordOutput("AI/TrainingPipeline/Status", "Training completed");
      
      return new TrainingResult(true, "Training completed successfully", 
                             finalTrainingLoss, finalValidationLoss);
      
    } catch (Exception e) {
      isTraining = false;
      Logger.recordOutput("AI/TrainingPipeline/Error", "Training failed: " + e.getMessage());
      return new TrainingResult(false, "Training failed: " + e.getMessage(), 0.0, 0.0);
    }
  }

  /**
   * Trains one epoch on the training data.
   */
  private double trainEpoch(List<TrainingData> trainingData) {
    double totalLoss = 0.0;
    int batchCount = 0;
    
    // Create mini-batches
    for (int i = 0; i < trainingData.size(); i += BATCH_SIZE) {
      int endIndex = Math.min(i + BATCH_SIZE, trainingData.size());
      List<TrainingData> batch = trainingData.subList(i, endIndex);
      
      // Train on batch
      for (TrainingData data : batch) {
        network.trainStep(data.inputs, data.outputs);
        
        // Calculate loss (MSE)
        double[] prediction = network.predict(data.inputs);
        double batchLoss = 0.0;
        for (int j = 0; j < prediction.length; j++) {
          double error = prediction[j] - data.outputs[j];
          batchLoss += error * error;
        }
        totalLoss += batchLoss / prediction.length;
      }
      
      batchCount++;
    }
    
    return totalLoss / (batchCount * BATCH_SIZE);
  }

  /**
   * Evaluates the model on validation data.
   */
  private double evaluateModel(List<TrainingData> validationData) {
    double totalLoss = 0.0;
    
    for (TrainingData data : validationData) {
      double[] prediction = network.predict(data.inputs);
      
      // Calculate MSE loss
      double loss = 0.0;
      for (int i = 0; i < prediction.length; i++) {
        double error = prediction[i] - data.outputs[i];
        loss += error * error;
      }
      totalLoss += loss / prediction.length;
    }
    
    return totalLoss / validationData.size();
  }

  /**
   * Loads all training data from CSV files.
   */
  private List<TrainingData> loadAllTrainingData() throws IOException {
    List<TrainingData> allData = new ArrayList<>();
    
    if (!Files.exists(Paths.get(DATA_DIR))) {
      Logger.recordOutput("AI/TrainingPipeline/Error", "Data directory does not exist");
      return allData;
    }
    
    Files.list(Paths.get(DATA_DIR))
        .filter(path -> path.toString().endsWith(".csv"))
        .sorted()
        .forEach(path -> {
          try {
            List<TrainingData> fileData = loadTrainingDataFile(path.toString());
            allData.addAll(fileData);
            Logger.recordOutput("AI/TrainingPipeline/LoadedFile", path.getFileName().toString());
          } catch (IOException e) {
            Logger.recordOutput("AI/TrainingPipeline/Error", 
                             "Failed to load file " + path.getFileName() + ": " + e.getMessage());
          }
        });
    
    return allData;
  }

  /**
   * Loads training data from a single CSV file.
   */
  private List<TrainingData> loadTrainingDataFile(String filename) throws IOException {
    List<TrainingData> data = new ArrayList<>();
    
    try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
      String line = reader.readLine(); // Skip header
      if (line == null) return data;
      
      while ((line = reader.readLine()) != null) {
        String[] values = line.split(",");
        if (values.length >= 14) { // Ensure we have all required columns
          try {
            // Parse input features (8 features)
            double[] inputs = new double[8];
            inputs[0] = Double.parseDouble(values[0]); // distance_to_hub
            inputs[1] = Double.parseDouble(values[1]); // robot_velocity_x
            inputs[2] = Double.parseDouble(values[2]); // robot_velocity_y
            inputs[3] = Double.parseDouble(values[3]); // radial_velocity
            inputs[4] = Double.parseDouble(values[4]); // tangential_velocity
            inputs[5] = Double.parseDouble(values[5]); // battery_voltage
            inputs[6] = Double.parseDouble(values[6]); // drum_temperature
            inputs[7] = Double.parseDouble(values[7]); // hood_angle
            
            // Parse target outputs (2 outputs)
            double[] outputs = new double[2];
            outputs[0] = Double.parseDouble(values[8]); // target_rpm
            outputs[1] = Double.parseDouble(values[9]); // target_hood_angle
            
            // Only use successful shots for training
            boolean shotSuccessful = Boolean.parseBoolean(values[10]);
            double accuracyError = Double.parseDouble(values[13]);
            
            // Filter out bad data points
            if (shotSuccessful && accuracyError < 0.5) {
              data.add(new TrainingData(inputs, outputs));
            }
            
          } catch (NumberFormatException e) {
            Logger.recordOutput("AI/TrainingPipeline/Error", 
                             "Failed to parse line in " + filename + ": " + line);
          }
        }
      }
    }
    
    return data;
  }

  /**
   * Shuffles data randomly.
   */
  private void shuffleData(List<TrainingData> data) {
    for (int i = data.size() - 1; i > 0; i--) {
      int j = random.nextInt(i + 1);
      TrainingData temp = data.get(i);
      data.set(i, data.get(j));
      data.set(j, temp);
    }
  }

  /**
   * Gets training status.
   */
  public boolean isTraining() {
    return isTraining;
  }

  /**
   * Data structure for training data.
   */
  private static class TrainingData {
    final double[] inputs;
    final double[] outputs;
    
    TrainingData(double[] inputs, double[] outputs) {
      this.inputs = inputs;
      this.outputs = outputs;
    }
  }

  /**
   * Training result data structure.
   */
  public static class TrainingResult {
    public final boolean success;
    public final String message;
    public final double trainingLoss;
    public final double validationLoss;
    
    public TrainingResult(boolean success, String message, double trainingLoss, double validationLoss) {
      this.success = success;
      this.message = message;
      this.trainingLoss = trainingLoss;
      this.validationLoss = validationLoss;
    }
    
    @Override
    public String toString() {
      return String.format("TrainingResult{success=%s, message='%s', trainingLoss=%.6f, validationLoss=%.6f}", 
                         success, message, trainingLoss, validationLoss);
    }
  }
}
