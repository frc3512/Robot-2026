import java.net.HttpURLConnection;
import java.net.URL;
import java.io.BufferedReader;
import java.io.InputStreamReader;

public class test_server_connection {
    public static void main(String[] args) {
        try {
            // Test if we can connect to the training UI
            URL url = new URL("http://localhost:8080");
            HttpURLConnection conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("GET");
            conn.setConnectTimeout(5000);
            conn.setReadTimeout(5000);
            
            System.out.println("Testing connection to http://localhost:8080...");
            
            int responseCode = conn.getResponseCode();
            System.out.println("Response Code: " + responseCode);
            
            if (responseCode == 200) {
                System.out.println("✅ Server is running and accessible!");
                
                // Read some content
                BufferedReader in = new BufferedReader(new InputStreamReader(conn.getInputStream()));
                String inputLine;
                StringBuilder content = new StringBuilder();
                
                while ((inputLine = in.readLine()) != null) {
                    content.append(inputLine);
                    if (content.length() > 200) break; // Just read first 200 chars
                }
                in.close();
                
                System.out.println("Content preview: " + content.substring(0, Math.min(content.length(), 100)) + "...");
            } else {
                System.out.println("❌ Server responded with code: " + responseCode);
            }
            
        } catch (Exception e) {
            System.out.println("❌ Connection failed: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
