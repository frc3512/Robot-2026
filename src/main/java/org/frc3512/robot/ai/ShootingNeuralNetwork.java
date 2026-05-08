package org.frc3512.robot.ai;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.Random;

import org.littletonrobotics.junction.Logger;

/**
 * Simple neural network for shooting parameter prediction.
 * Uses feedforward architecture with ReLU activation.
 * Designed to run efficiently on roboRIO.
 */
public class ShootingNeuralNetwork {
  private static final String MODEL_DIR = "/tmp/robot_ai_models";
  private static final String MODEL_FILE = "shooting_model.nn";
  
  // Network architecture
  private final int inputSize;
  private final int hiddenSize;
  private final int outputSize;
  
  // Weights and biases
  private double[][] weights1; // input -> hidden
  private double[] biases1;     // hidden biases
  private double[][] weights2; // hidden -> output
  private double[] biases2;     // output biases
  
  // Training parameters
  private final double learningRate;
  private final double regularization;
  private final Random random;
  
  // Normalization parameters
  private double[] inputMean;
  private double[] inputStd;
  private double[] outputMean;
  private double[] outputStd;
  
  private boolean isInitialized = false;
  private long lastUpdateTime = 0;

  /**
   * Creates a new neural network with specified architecture.
   */
  public ShootingNeuralNetwork(int inputSize, int hiddenSize, int outputSize) {
    this(inputSize, hiddenSize, outputSize, 0.01, 0.001);
  }

  /**
   * Creates a new neural network with training parameters.
   */
  public ShootingNeuralNetwork(int inputSize, int hiddenSize, int outputSize, 
                              double learningRate, double regularization) {
    this.inputSize = inputSize;
    this.hiddenSize = hiddenSize;
    this.outputSize = outputSize;
    this.learningRate = learningRate;
    this.regularization = regularization;
    this.random = new Random(42); // Fixed seed for reproducibility
    
    initializeNetwork();
  }

  /**
   * Initializes network weights and biases with Xavier initialization.
   */
  private void initializeNetwork() {
    // Initialize weights with Xavier initialization
    double inputScale = Math.sqrt(2.0 / inputSize);
    double hiddenScale = Math.sqrt(2.0 / hiddenSize);
    
    weights1 = new double[inputSize][hiddenSize];
    biases1 = new double[hiddenSize];
    weights2 = new double[hiddenSize][outputSize];
    biases2 = new double[outputSize];
    
    // Initialize weights
    for (int i = 0; i < inputSize; i++) {
      for (int j = 0; j < hiddenSize; j++) {
        weights1[i][j] = random.nextGaussian() * inputScale;
      }
    }
    
    for (int i = 0; i < hiddenSize; i++) {
      biases1[i] = 0.0;
      for (int j = 0; j < outputSize; j++) {
        weights2[i][j] = random.nextGaussian() * hiddenScale;
      }
    }
    
    for (int i = 0; i < outputSize; i++) {
      biases2[i] = 0.0;
    }
    
    // Initialize normalization parameters
    inputMean = new double[inputSize];
    inputStd = new double[inputSize];
    outputMean = new double[outputSize];
    outputStd = new double[outputSize];
    
    // Set default normalization values
    for (int i = 0; i < inputSize; i++) {
      inputMean[i] = 0.0;
      inputStd[i] = 1.0;
    }
    for (int i = 0; i < outputSize; i++) {
      outputMean[i] = 0.0;
      outputStd[i] = 1.0;
    }
    
    isInitialized = true;
    lastUpdateTime = System.currentTimeMillis();
  }

  /**
   * ReLU activation function.
   */
  private double relu(double x) {
    return Math.max(0, x);
  }

  /**
   * ReLU derivative.
   */
  private double reluDerivative(double x) {
    return x > 0 ? 1.0 : 0.0;
  }

  /**
   * Normalizes input values.
   */
  private double[] normalizeInput(double[] input) {
    double[] normalized = new double[input.length];
    for (int i = 0; i < input.length; i++) {
      normalized[i] = (input[i] - inputMean[i]) / (inputStd[i] + 1e-8);
    }
    return normalized;
  }

  /**
   * Denormalizes output values.
   */
  private double[] denormalizeOutput(double[] output) {
    double[] denormalized = new double[output.length];
    for (int i = 0; i < output.length; i++) {
      denormalized[i] = output[i] * outputStd[i] + outputMean[i];
    }
    return denormalized;
  }

  /**
   * Forward pass through the network.
   */
  public double[] predict(double[] input) {
    if (!isInitialized) {
      Logger.recordOutput("AI/NeuralNetwork/Error", "Network not initialized");
      return new double[outputSize];
    }
    
    if (input.length != inputSize) {
      Logger.recordOutput("AI/NeuralNetwork/Error", "Input size mismatch");
      return new double[outputSize];
    }
    
    // Normalize input
    double[] normalizedInput = normalizeInput(input);
    
    // Hidden layer
    double[] hidden = new double[hiddenSize];
    for (int i = 0; i < hiddenSize; i++) {
      hidden[i] = biases1[i];
      for (int j = 0; j < inputSize; j++) {
        hidden[i] += normalizedInput[j] * weights1[j][i];
      }
      hidden[i] = relu(hidden[i]);
    }
    
    // Output layer
    double[] output = new double[outputSize];
    for (int i = 0; i < outputSize; i++) {
      output[i] = biases2[i];
      for (int j = 0; j < hiddenSize; j++) {
        output[i] += hidden[j] * weights2[j][i];
      }
    }
    
    // Denormalize output
    return denormalizeOutput(output);
  }

  /**
   * Training step with backpropagation.
   */
  public void trainStep(double[] input, double[] target) {
    if (!isInitialized || input.length != inputSize || target.length != outputSize) {
      return;
    }
    
    // Normalize input and target
    double[] normalizedInput = normalizeInput(input);
    double[] normalizedTarget = new double[outputSize];
    for (int i = 0; i < outputSize; i++) {
      normalizedTarget[i] = (target[i] - outputMean[i]) / (outputStd[i] + 1e-8);
    }
    
    // Forward pass (store intermediate values)
    double[] hidden = new double[hiddenSize];
    double[] hiddenRaw = new double[hiddenSize];
    for (int i = 0; i < hiddenSize; i++) {
      hiddenRaw[i] = biases1[i];
      for (int j = 0; j < inputSize; j++) {
        hiddenRaw[i] += normalizedInput[j] * weights1[j][i];
      }
      hidden[i] = relu(hiddenRaw[i]);
    }
    
    double[] output = new double[outputSize];
    for (int i = 0; i < outputSize; i++) {
      output[i] = biases2[i];
      for (int j = 0; j < hiddenSize; j++) {
        output[i] += hidden[j] * weights2[j][i];
      }
    }
    
    // Backward pass
    double[] outputError = new double[outputSize];
    for (int i = 0; i < outputSize; i++) {
      outputError[i] = output[i] - normalizedTarget[i];
    }
    
    // Update output layer weights and biases
    for (int i = 0; i < hiddenSize; i++) {
      for (int j = 0; j < outputSize; j++) {
        double gradient = outputError[j] * hidden[i];
        weights2[i][j] -= learningRate * (gradient + regularization * weights2[i][j]);
      }
    }
    
    for (int i = 0; i < outputSize; i++) {
      biases2[i] -= learningRate * outputError[i];
    }
    
    // Hidden layer error
    double[] hiddenError = new double[hiddenSize];
    for (int i = 0; i < hiddenSize; i++) {
      for (int j = 0; j < outputSize; j++) {
        hiddenError[i] += outputError[j] * weights2[i][j];
      }
      hiddenError[i] *= reluDerivative(hiddenRaw[i]);
    }
    
    // Update hidden layer weights and biases
    for (int i = 0; i < inputSize; i++) {
      for (int j = 0; j < hiddenSize; j++) {
        double gradient = hiddenError[j] * normalizedInput[i];
        weights1[i][j] -= learningRate * (gradient + regularization * weights1[i][j]);
      }
    }
    
    for (int i = 0; i < hiddenSize; i++) {
      biases1[i] -= learningRate * hiddenError[i];
    }
    
    lastUpdateTime = System.currentTimeMillis();
  }

  /**
   * Updates normalization parameters from training data.
   */
  public void updateNormalization(List<double[]> inputs, List<double[]> outputs) {
    if (inputs.isEmpty() || outputs.isEmpty()) return;
    
    // Calculate input statistics
    for (int i = 0; i < inputSize; i++) {
      double sum = 0.0;
      double sumSq = 0.0;
      int count = 0;
      
      for (double[] input : inputs) {
        sum += input[i];
        sumSq += input[i] * input[i];
        count++;
      }
      
      inputMean[i] = sum / count;
      inputStd[i] = Math.sqrt((sumSq / count) - (inputMean[i] * inputMean[i]));
      if (inputStd[i] < 1e-8) inputStd[i] = 1.0;
    }
    
    // Calculate output statistics
    for (int i = 0; i < outputSize; i++) {
      double sum = 0.0;
      double sumSq = 0.0;
      int count = 0;
      
      for (double[] output : outputs) {
        sum += output[i];
        sumSq += output[i] * output[i];
        count++;
      }
      
      outputMean[i] = sum / count;
      outputStd[i] = Math.sqrt((sumSq / count) - (outputMean[i] * outputMean[i]));
      if (outputStd[i] < 1e-8) outputStd[i] = 1.0;
    }
  }

  /**
   * Saves the model to file.
   */
  public void saveModel() {
    try {
      Files.createDirectories(Paths.get(MODEL_DIR));
      String filename = Paths.get(MODEL_DIR, MODEL_FILE).toString();
      
      try (BufferedWriter writer = new BufferedWriter(new FileWriter(filename))) {
        // Save architecture
        writer.write(inputSize + "," + hiddenSize + "," + outputSize);
        writer.newLine();
        
        // Save normalization parameters
        for (int i = 0; i < inputSize; i++) {
          writer.write(inputMean[i] + "," + inputStd[i]);
          if (i < inputSize - 1) writer.write(",");
        }
        writer.newLine();
        
        for (int i = 0; i < outputSize; i++) {
          writer.write(outputMean[i] + "," + outputStd[i]);
          if (i < outputSize - 1) writer.write(",");
        }
        writer.newLine();
        
        // Save weights and biases
        saveMatrix(writer, weights1);
        saveArray(writer, biases1);
        saveMatrix(writer, weights2);
        saveArray(writer, biases2);
      }
      
      Logger.recordOutput("AI/NeuralNetwork/ModelSaved", filename);
      
    } catch (IOException e) {
      Logger.recordOutput("AI/NeuralNetwork/Error", "Failed to save model: " + e.getMessage());
    }
  }

  /**
   * Loads the model from file.
   */
  public boolean loadModel() {
    String filename = Paths.get(MODEL_DIR, MODEL_FILE).toString();
    
    try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
      // Load architecture
      String[] arch = reader.readLine().split(",");
      if (arch.length != 3) return false;
      
      int loadedInputSize = Integer.parseInt(arch[0]);
      int loadedHiddenSize = Integer.parseInt(arch[1]);
      int loadedOutputSize = Integer.parseInt(arch[2]);
      
      if (loadedInputSize != inputSize || loadedHiddenSize != hiddenSize || loadedOutputSize != outputSize) {
        Logger.recordOutput("AI/NeuralNetwork/Error", "Model architecture mismatch");
        return false;
      }
      
      // Load normalization parameters
      String[] inputNorm = reader.readLine().split(",");
      String[] outputNorm = reader.readLine().split(",");
      
      for (int i = 0; i < inputSize; i++) {
        inputMean[i] = Double.parseDouble(inputNorm[i * 2]);
        inputStd[i] = Double.parseDouble(inputNorm[i * 2 + 1]);
      }
      
      for (int i = 0; i < outputSize; i++) {
        outputMean[i] = Double.parseDouble(outputNorm[i * 2]);
        outputStd[i] = Double.parseDouble(outputNorm[i * 2 + 1]);
      }
      
      // Load weights and biases
      loadMatrix(reader, weights1);
      loadArray(reader, biases1);
      loadMatrix(reader, weights2);
      loadArray(reader, biases2);
      
      isInitialized = true;
      lastUpdateTime = System.currentTimeMillis();
      Logger.recordOutput("AI/NeuralNetwork/ModelLoaded", filename);
      return true;
      
    } catch (IOException | NumberFormatException e) {
      Logger.recordOutput("AI/NeuralNetwork/Error", "Failed to load model: " + e.getMessage());
      return false;
    }
  }

  private void saveMatrix(BufferedWriter writer, double[][] matrix) throws IOException {
    for (int i = 0; i < matrix.length; i++) {
      for (int j = 0; j < matrix[i].length; j++) {
        writer.write(String.valueOf(matrix[i][j]));
        if (j < matrix[i].length - 1) writer.write(",");
      }
      writer.newLine();
    }
  }

  private void saveArray(BufferedWriter writer, double[] array) throws IOException {
    for (int i = 0; i < array.length; i++) {
      writer.write(String.valueOf(array[i]));
      if (i < array.length - 1) writer.write(",");
    }
    writer.newLine();
  }

  private void loadMatrix(BufferedReader reader, double[][] matrix) throws IOException {
    for (int i = 0; i < matrix.length; i++) {
      String[] values = reader.readLine().split(",");
      for (int j = 0; j < matrix[i].length; j++) {
        matrix[i][j] = Double.parseDouble(values[j]);
      }
    }
  }

  private void loadArray(BufferedReader reader, double[] array) throws IOException {
    String[] values = reader.readLine().split(",");
    for (int i = 0; i < array.length; i++) {
      array[i] = Double.parseDouble(values[i]);
    }
  }

  /**
   * Gets network information.
   */
  public boolean isInitialized() {
    return isInitialized;
  }

  public long getLastUpdateTime() {
    return lastUpdateTime;
  }

  public int getInputSize() {
    return inputSize;
  }

  public int getHiddenSize() {
    return hiddenSize;
  }

  public int getOutputSize() {
    return outputSize;
  }
}
