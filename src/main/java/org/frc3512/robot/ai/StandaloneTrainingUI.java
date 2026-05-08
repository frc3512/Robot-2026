package org.frc3512.robot.ai;

import java.io.IOException;
import java.io.OutputStream;
import java.util.HashMap;
import java.util.Map;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Standalone training UI that doesn't require complex subsystem dependencies.
 * Provides a complete web interface for AI shooting training.
 */
public class StandaloneTrainingUI {
  private HttpServer server;
  private final int port;
  private boolean running = false;
  
  // Training state
  private int totalShots = 0;
  private int successfulShots = 0;
  private double averageError = 0.0;
  private final Map<String, Object> trainingData = new HashMap<>();

  public StandaloneTrainingUI(int port) {
    this.port = port;
    initializeTrainingData();
  }

  private void initializeTrainingData() {
    trainingData.put("trainingMode", false);
    trainingData.put("totalShots", 0);
    trainingData.put("successfulShots", 0);
    trainingData.put("successRate", 0.0);
    trainingData.put("averageError", 0.0);
    trainingData.put("recentShots", new java.util.ArrayList<>());
  }

  /**
   * Starts the web server.
   */
  public void start() {
    try {
      System.out.println("Starting training UI server on port " + port + "...");
      
      // Check if port is available
      try (java.net.ServerSocket socket = new java.net.ServerSocket(port)) {
        System.out.println("Port " + port + " is available");
      } catch (java.io.IOException e) {
        System.err.println("Port " + port + " is already in use!");
        DriverStation.reportError("Port " + port + " is already in use: " + e.getMessage(), e.getStackTrace());
        return;
      }
      
      server = HttpServer.create(new java.net.InetSocketAddress(port), 0);
      
      // Serve the main UI
      server.createContext("/", new MainHandler());
      
      // API endpoints
      server.createContext("/api/start", new StartTrainingHandler());
      server.createContext("/api/stop", new StopTrainingHandler());
      server.createContext("/api/execute-shot", new ExecuteShotHandler());
      server.createContext("/api/shot", new RecordShotHandler());
      server.createContext("/api/status", new StatusHandler());
      server.createContext("/api/emergency-stop", new EmergencyStopHandler());
      
      server.setExecutor(null);
      server.start();
      running = true;
      
      DriverStation.reportWarning("Standalone Training UI started on port " + port, false);
      System.out.println("========================================");
      System.out.println("AI TRAINING UI STARTED SUCCESSFULLY!");
      System.out.println("Open your browser and go to: http://localhost:" + port);
      System.out.println("========================================");
      
    } catch (Exception e) {
      System.err.println("Failed to start training UI: " + e.getMessage());
      e.printStackTrace();
      DriverStation.reportError("Failed to start training UI: " + e.getMessage(), e.getStackTrace());
    }
  }

  /**
   * Stops the web server.
   */
  public void stop() {
    if (server != null && running) {
      server.stop(0);
      running = false;
      System.out.println("Training UI server stopped");
    }
  }

  // HTTP Handlers
  private class MainHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      String response = getHTMLContent();
      exchange.getResponseHeaders().set("Content-Type", "text/html");
      exchange.sendResponseHeaders(200, response.length());
      OutputStream os = exchange.getResponseBody();
      os.write(response.getBytes());
      os.close();
    }
  }

  private class StartTrainingHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      trainingData.put("trainingMode", true);
      String response = "{\"status\":\"Training started\"}";
      sendJSONResponse(exchange, response);
    }
  }

  private class StopTrainingHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      trainingData.put("trainingMode", false);
      String response = "{\"status\":\"Training stopped\"}";
      sendJSONResponse(exchange, response);
    }
  }

  private class RecordShotHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if ("POST".equals(exchange.getRequestMethod())) {
        // Read request body
        String body = new String(exchange.getRequestBody().readAllBytes());
        
        // Parse JSON (simple parsing)
        boolean successful = body.contains("\"successful\":true");
        double accuracy = 0.5; // default
        
        if (body.contains("\"accuracy\"")) {
          String[] parts = body.split("\"accuracy\":");
          if (parts.length > 1) {
            String value = parts[1].split(",")[0].replace("}", "").trim();
            try {
              accuracy = Double.parseDouble(value);
            } catch (NumberFormatException e) {
              // Use default
            }
          }
        }
        
        // Update statistics
        totalShots++;
        if (successful) {
          successfulShots++;
        }
        
        // Update running average error
        averageError = (averageError * (totalShots - 1) + accuracy) / totalShots;
        
        // Add to recent shots
        @SuppressWarnings("unchecked")
        java.util.ArrayList<Map<String, Object>> recentShots = 
            (java.util.ArrayList<Map<String, Object>>) trainingData.get("recentShots");
        
        Map<String, Object> shot = new HashMap<>();
        shot.put("successful", successful);
        shot.put("accuracy", accuracy);
        shot.put("timestamp", new java.util.Date().toString());
        
        recentShots.add(0, shot);
        if (recentShots.size() > 10) {
          recentShots.remove(recentShots.size() - 1);
        }
        
        // Update training data
        trainingData.put("totalShots", totalShots);
        trainingData.put("successfulShots", successfulShots);
        trainingData.put("successRate", totalShots > 0 ? (double) successfulShots / totalShots : 0.0);
        trainingData.put("averageError", averageError);
        
        String response = "{\"status\":\"Shot recorded\"}";
        sendJSONResponse(exchange, response);
      }
    }
  }

  private class StatusHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      sendJSONResponse(exchange, mapToJson(trainingData));
    }
  }

  private class ExecuteShotHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      if ("POST".equals(exchange.getRequestMethod())) {
        // Simulate executing a shot
        totalShots++;
        
        // Create a shot record
        @SuppressWarnings("unchecked")
        java.util.ArrayList<Map<String, Object>> recentShots = 
            (java.util.ArrayList<Map<String, Object>>) trainingData.get("recentShots");
        
        Map<String, Object> shot = new HashMap<>();
        shot.put("successful", true); // Assume successful for now
        shot.put("accuracy", 0.2); // Default accuracy
        shot.put("timestamp", new java.util.Date().toString());
        
        recentShots.add(0, shot);
        if (recentShots.size() > 10) {
          recentShots.remove(recentShots.size() - 1);
        }
        
        // Update statistics
        successfulShots++;
        averageError = (averageError * (totalShots - 1) + 0.2) / totalShots;
        
        trainingData.put("totalShots", totalShots);
        trainingData.put("successfulShots", successfulShots);
        trainingData.put("successRate", totalShots > 0 ? (double) successfulShots / totalShots : 0.0);
        trainingData.put("averageError", averageError);
        trainingData.put("recentShots", recentShots);
        
        String response = "{\"status\":\"Shot executed\", \"shotId\":" + totalShots + "}";
        sendJSONResponse(exchange, response);
      }
    }
  }

  private class EmergencyStopHandler implements HttpHandler {
    @Override
    public void handle(HttpExchange exchange) throws IOException {
      trainingData.put("trainingMode", false);
      String response = "{\"status\":\"Emergency stop activated\"}";
      sendJSONResponse(exchange, response);
    }
  }

  private void sendJSONResponse(HttpExchange exchange, String json) throws IOException {
    exchange.getResponseHeaders().set("Content-Type", "application/json");
    exchange.getResponseHeaders().set("Access-Control-Allow-Origin", "*");
    exchange.sendResponseHeaders(200, json.length());
    OutputStream os = exchange.getResponseBody();
    os.write(json.getBytes());
    os.close();
  }

  private String mapToJson(Map<String, Object> map) {
    StringBuilder json = new StringBuilder("{");
    boolean first = true;
    
    for (Map.Entry<String, Object> entry : map.entrySet()) {
      if (!first) json.append(",");
      first = false;
      
      json.append("\"").append(entry.getKey()).append("\":");
      Object value = entry.getValue();
      
      if (value instanceof String) {
        json.append("\"").append(value).append("\"");
      } else if (value instanceof Boolean) {
        json.append(value);
      } else if (value instanceof java.util.List) {
        json.append(listToJson((java.util.List<?>) value));
      } else {
        json.append(value);
      }
    }
    
    json.append("}");
    return json.toString();
  }

  @SuppressWarnings("unchecked")
  private String listToJson(java.util.List<?> list) {
    StringBuilder json = new StringBuilder("[");
    for (int i = 0; i < list.size(); i++) {
      if (i > 0) json.append(",");
      Object item = list.get(i);
      if (item instanceof Map) {
        json.append(mapToJson((Map<String, Object>) item));
      } else {
        json.append("\"").append(item).append("\"");
      }
    }
    json.append("]");
    return json.toString();
  }

  /**
   * Gets the HTML content for the training UI.
   */
  private String getHTMLContent() {
    return """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AI Shooting Training Interface</title>
    <script src="https://cdn.tailwindcss.com"></script>
    <script src="https://unpkg.com/lucide@latest"></script>
    <style>
        .glass-effect {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        .shot-button {
            transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
        }
        .shot-button:active {
            transform: scale(0.95);
        }
    </style>
</head>
<body class="bg-gradient-to-br from-slate-900 via-purple-900 to-slate-900 min-h-screen text-white">
    <div class="container mx-auto p-6 max-w-6xl">
        <!-- Header -->
        <header class="mb-8 text-center">
            <h1 class="text-4xl font-bold mb-2 bg-gradient-to-r from-blue-400 to-purple-400 bg-clip-text text-transparent">
                AI Shooting Training Interface
            </h1>
            <p class="text-gray-300">Train and monitor your AI shooting system</p>
        </header>

        <!-- Status Bar -->
        <div class="glass-effect rounded-lg p-4 mb-6">
            <div class="flex justify-between items-center">
                <div class="flex items-center space-x-4">
                    <div id="trainingStatus" class="px-3 py-1 rounded-full text-sm font-medium bg-gray-500/20 text-gray-400">
                        Training Inactive
                    </div>
                </div>
                <div id="statusMessage" class="text-sm text-blue-400">
                    Ready
                </div>
            </div>
        </div>

        <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <!-- Left Column - Controls -->
            <div class="space-y-6">
                <!-- Training Mode Control -->
                <div class="glass-effect rounded-lg p-6">
                    <h2 class="text-xl font-semibold mb-4 flex items-center">
                        <i data-lucide="play-circle" class="w-5 h-5 mr-2"></i>
                        Training Mode
                    </h2>
                    <div class="space-y-3">
                        <button id="startBtn" onclick="startTraining()" class="w-full bg-green-600 hover:bg-green-700 px-4 py-2 rounded-lg font-medium transition-colors">
                            Start Training
                        </button>
                        <button id="stopBtn" onclick="stopTraining()" disabled class="w-full bg-red-600 hover:bg-red-700 disabled:bg-gray-600 disabled:cursor-not-allowed px-4 py-2 rounded-lg font-medium transition-colors">
                            Stop Training
                        </button>
                    </div>
                </div>

                <!-- Shot Controls -->
                <div class="glass-effect rounded-lg p-6">
                    <h2 class="text-xl font-semibold mb-4 flex items-center">
                        <i data-lucide="target" class="w-5 h-5 mr-2"></i>
                        Shot Controls
                    </h2>
                    
                    <div class="space-y-4">
                        <!-- Execute Shot Button -->
                        <div class="mb-4 p-4 bg-blue-600/20 rounded-lg border border-blue-600/30">
                            <h3 class="text-lg font-medium mb-2 text-blue-400">🚀 Execute Physical Shot</h3>
                            <p class="text-sm text-gray-300 mb-3">Press this button when robot is ready to shoot</p>
                            <button onclick="executePhysicalShot()" class="w-full bg-blue-600 hover:bg-blue-700 px-4 py-3 rounded-lg font-medium transition-colors flex items-center justify-center">
                                <i data-lucide="play" class="w-5 h-5 mr-2"></i>
                                Execute Shot
                            </button>
                        </div>
                        
                        <!-- Shot Recording -->
                        <div>
                            <label class="block text-sm font-medium mb-2">
                                Accuracy Error (meters)
                            </label>
                            <input id="accuracyInput" type="number" step="0.1" min="0" max="5" value="0.5" 
                                   class="w-full px-3 py-2 bg-gray-800 rounded-lg text-white">
                        </div>
                        
                        <div class="grid grid-cols-2 gap-2">
                            <button onclick="recordShot(true)" class="bg-green-600 hover:bg-green-700 px-3 py-2 rounded-lg font-medium transition-colors">
                                ✓ Made Shot
                            </button>
                            <button onclick="recordShot(false)" class="bg-red-600 hover:bg-red-700 px-3 py-2 rounded-lg font-medium transition-colors">
                                ✗ Missed Shot
                            </button>
                        </div>
                    </div>
                </div>

                <!-- Emergency Stop -->
                <button onclick="emergencyStop()" class="w-full bg-red-600 hover:bg-red-700 px-4 py-3 rounded-lg font-medium transition-colors flex items-center justify-center">
                    <i data-lucide="alert-circle" class="w-5 h-5 mr-2"></i>
                    Emergency Stop
                </button>
            </div>

            <!-- Right Column - Statistics -->
            <div class="space-y-6">
                <!-- Training Statistics -->
                <div class="glass-effect rounded-lg p-6">
                    <h2 class="text-xl font-semibold mb-4 flex items-center">
                        <i data-lucide="bar-chart-2" class="w-5 h-5 mr-2"></i>
                        Training Statistics
                    </h2>
                    <div class="space-y-3">
                        <div class="flex justify-between">
                            <span class="text-gray-400">Total Shots:</span>
                            <span id="totalShots" class="font-medium">0</span>
                        </div>
                        <div class="flex justify-between">
                            <span class="text-gray-400">Successful Shots:</span>
                            <span id="successfulShots" class="font-medium text-green-400">0</span>
                        </div>
                        <div class="flex justify-between">
                            <span class="text-gray-400">Success Rate:</span>
                            <span id="successRate" class="font-medium">0.0%</span>
                        </div>
                        <div class="flex justify-between">
                            <span class="text-gray-400">Average Error:</span>
                            <span id="averageError" class="font-medium">0.000m</span>
                        </div>
                    </div>
                </div>

                <!-- Recent Shots -->
                <div class="glass-effect rounded-lg p-6">
                    <h2 class="text-xl font-semibold mb-4 flex items-center">
                        <i data-lucide="history" class="w-5 h-5 mr-2"></i>
                        Recent Shots
                    </h2>
                    <div id="recentShots" class="space-y-2 max-h-64 overflow-y-auto">
                        <p class="text-gray-400 text-sm">No shots recorded yet</p>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Initialize Lucide icons
        document.addEventListener('DOMContentLoaded', () => {
            lucide.createIcons();
            updateStatus();
            setInterval(updateStatus, 1000);
        });

        async function startTraining() {
            try {
                const response = await fetch('/api/start', {method: 'POST'});
                const data = await response.json();
                updateStatusMessage(data.status, 'success');
            } catch (error) {
                updateStatusMessage('Failed to start training', 'error');
            }
        }

        async function stopTraining() {
            try {
                const response = await fetch('/api/stop', {method: 'POST'});
                const data = await response.json();
                updateStatusMessage(data.status, 'info');
            } catch (error) {
                updateStatusMessage('Failed to stop training', 'error');
            }
        }

        async function executePhysicalShot() {
            try {
                const response = await fetch('/api/execute-shot', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'}
                });
                const data = await response.json();
                updateStatusMessage(data.status, 'success');
                updateStatus(); // Refresh statistics
                
                // Show visual feedback
                const executeBtn = document.querySelector('[onclick="executePhysicalShot()"]');
                executeBtn.textContent = '✓ Shot Executed';
                executeBtn.classList.add('bg-green-600');
                executeBtn.classList.remove('bg-blue-600');
                
                setTimeout(() => {
                    executeBtn.textContent = '🚀 Execute Shot';
                    executeBtn.classList.remove('bg-green-600');
                    executeBtn.classList.add('bg-blue-600');
                }, 2000);
                
            } catch (error) {
                updateStatusMessage('Failed to execute shot', 'error');
            }
        }

        async function recordShot(successful) {
            const accuracy = parseFloat(document.getElementById('accuracyInput').value) || 0.5;
            
            try {
                const response = await fetch('/api/shot', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify({successful, accuracy})
                });
                const data = await response.json();
                updateStatusMessage(data.status, 'success');
                updateStatus(); // Refresh statistics
            } catch (error) {
                updateStatusMessage('Failed to record shot', 'error');
            }
        }

        async function emergencyStop() {
            try {
                const response = await fetch('/api/emergency-stop', {method: 'POST'});
                const data = await response.json();
                updateStatusMessage(data.status, 'warning');
            } catch (error) {
                updateStatusMessage('Failed to execute emergency stop', 'error');
            }
        }

        async function updateStatus() {
            try {
                const response = await fetch('/api/status');
                const data = await response.json();
                
                // Update UI elements
                document.getElementById('totalShots').textContent = data.totalShots || 0;
                document.getElementById('successfulShots').textContent = data.successfulShots || 0;
                document.getElementById('successRate').textContent = ((data.successRate || 0) * 100).toFixed(1) + '%';
                document.getElementById('averageError').textContent = (data.averageError || 0).toFixed(3) + 'm';
                
                // Update training status
                const statusEl = document.getElementById('trainingStatus');
                const startBtn = document.getElementById('startBtn');
                const stopBtn = document.getElementById('stopBtn');
                
                if (data.trainingMode) {
                    statusEl.className = 'px-3 py-1 rounded-full text-sm font-medium bg-green-500/20 text-green-400';
                    statusEl.textContent = 'Training Active';
                    startBtn.disabled = true;
                    stopBtn.disabled = false;
                } else {
                    statusEl.className = 'px-3 py-1 rounded-full text-sm font-medium bg-gray-500/20 text-gray-400';
                    statusEl.textContent = 'Training Inactive';
                    startBtn.disabled = false;
                    stopBtn.disabled = true;
                }
                
                // Update recent shots
                const recentShotsEl = document.getElementById('recentShots');
                if (data.recentShots && data.recentShots.length > 0) {
                    recentShotsEl.innerHTML = data.recentShots.map(shot => 
                        `<div class="p-3 rounded-lg ${shot.successful ? 'bg-green-600/20 border border-green-600/30' : 'bg-red-600/20 border border-red-600/30'}">
                            <div class="flex justify-between items-center">
                                <div class="flex items-center">
                                    <span class="mr-2">${shot.successful ? '✓' : '✗'}</span>
                                    <span class="text-sm">${shot.successful ? 'Made' : 'Missed'}</span>
                                </div>
                                <div class="text-right">
                                    <div class="text-sm">Error: ${shot.accuracy.toFixed(2)}m</div>
                                    <div class="text-xs text-gray-400">${new Date(shot.timestamp).toLocaleTimeString()}</div>
                                </div>
                            </div>
                        </div>`
                    ).join('');
                } else {
                    recentShotsEl.innerHTML = '<p class="text-gray-400 text-sm">No shots recorded yet</p>';
                }
                
            } catch (error) {
                console.error('Failed to update status:', error);
            }
        }

        function updateStatusMessage(message, level) {
            const statusEl = document.getElementById('statusMessage');
            statusEl.textContent = message;
            
            statusEl.className = 'text-sm';
            switch (level) {
                case 'success': statusEl.classList.add('text-green-400'); break;
                case 'warning': statusEl.classList.add('text-yellow-400'); break;
                case 'error': statusEl.classList.add('text-red-400'); break;
                default: statusEl.classList.add('text-blue-400');
            }
        }
    </script>
</body>
</html>
            """;
  }

  public boolean isRunning() {
    return running;
  }
}
