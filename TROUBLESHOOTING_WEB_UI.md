# Web UI Connection Troubleshooting

## Issue: Can't connect to training UI at http://localhost:8080

### 🔍 Quick Diagnosis Steps

#### 1. Check Simulation Output
When you run `./gradlew simulatejava`, look for these messages:
```
Starting training UI server on port 8080...
Port 8080 is available
========================================
AI TRAINING UI STARTED SUCCESSFULLY!
Open your browser and go to: http://localhost:8080
========================================
```

If you don't see these messages, the server isn't starting properly.

#### 2. Test Connection Manually
Open Command Prompt and run:
```bash
curl http://localhost:8080
```
Or open browser and try accessing: http://127.0.0.1:8080

### 🛠️ Common Solutions

#### Port Already in Use
**Symptoms:** "Port 8080 is already in use!" message

**Solutions:**
1. **Change port in code:**
   ```java
   // In TrainingUIMain.java, change:
   trainingUI = new StandaloneTrainingUI(8081); // Use different port
   ```

2. **Kill processes using port 8080:**
   ```bash
   netstat -ano | findstr :8080
   taskkill /PID <PID> /F
   ```

#### Firewall Blocking
**Symptoms:** Connection timeout or refused

**Solutions:**
1. **Allow Java in Windows Firewall**
2. **Try different browser** (Chrome, Firefox, Edge)
3. **Disable VPN** if running

#### Simulation Environment Issues
**Symptoms:** Server starts but can't connect

**Solutions:**
1. **Wait 10-15 seconds** after simulation starts
2. **Check simulator console** for error messages
3. **Restart simulation** completely

### 🧪 Testing Steps

#### Step 1: Verify Server Startup
1. Run `./gradlew simulatejava`
2. Look for "AI TRAINING UI STARTED SUCCESSFULLY!" message
3. If not seen, check for error messages

#### Step 2: Test Browser Access
1. Wait 5 seconds after server starts
2. Open browser to `http://localhost:8080`
3. Try `http://127.0.0.1:8080` if first doesn't work
4. Check browser console (F12) for errors

#### Step 3: Test API Endpoints
1. Open browser to `http://localhost:8080/api/status`
2. Should see JSON like: `{"trainingMode":false,"totalShots":0,...}`

### 🔧 Debug Mode

If still having issues, add this to `TrainingUIMain.java`:

```java
@Override
public void robotInit() {
    System.out.println("=== DEBUG: Starting robot init ===");
    trainingUI = new StandaloneTrainingUI(8080);
    trainingUI.start();
    System.out.println("=== DEBUG: Robot init complete ===");
}

@Override
public void robotPeriodic() {
    // Add this to see if robot is running
    static int counter = 0;
    if (++counter % 100 == 0) {
        System.out.println("=== DEBUG: Robot running, counter=" + counter + " ===");
    }
}
```

### 📋 Alternative Solutions

#### Solution 1: Use Different Port
```java
// In TrainingUIMain.java
trainingUI = new StandaloneTrainingUI(8081);
```
Then access: http://localhost:8081

#### Solution 2: Check Network Interface
Sometimes localhost doesn't work in simulation. Try:
- http://127.0.0.1:8080
- http://0.0.0.0:8080

#### Solution 3: Manual Server Test
Create simple test file to verify server works:
```java
// Save as SimpleServerTest.java
import java.io.*;
import java.net.*;

public class SimpleServerTest {
    public static void main(String[] args) throws IOException {
        ServerSocket server = new ServerSocket(8080);
        System.out.println("Test server running on port 8080");
        while (true) {
            Socket client = server.accept();
            PrintWriter out = new PrintWriter(client.getOutputStream());
            out.println("HTTP/1.1 200 OK");
            out.println("Content-Type: text/plain");
            out.println();
            out.println("Server is working!");
            client.close();
        }
    }
}
```

Run with: `java SimpleServerTest.java` then test http://localhost:8080

### 🆘 If Nothing Works

1. **Check Java version:** Should be Java 11+
2. **Rebuild project:** `./gradlew clean build`
3. **Restart IDE:** Close and reopen VSCode/Eclipse
4. **Check WPILib version:** Update if outdated

### 📞 Getting Help

If still stuck:
1. **Copy full simulation output** including any error messages
2. **Screenshot** of browser error (F12 console)
3. **Test with different browser**
4. **Check Windows Event Viewer** for Java errors

---

**Most common fix:** Wait 10-15 seconds after simulation starts, then try http://127.0.0.1:8080 instead of localhost.
