#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h> // Include WebServer
#include "config.h"
#include "web_index.h" // Include the HTML

// Function declarations
void setupCamera();
void setupWiFi();
void setupWebServer();
void detectBall();

WebServer server(80); // Create server on port 80

// Global variables for detection status
bool ballDetected = false;
int ballX = 0;
int ballY = 0;
// Global String for distance
String distStr = "-";
uint32_t lastPixelCount = 0;
int globalObjectCount = 0;

struct DetectedObject {
  int x, y, pixels;
  String label;
};
DetectedObject objects[10]; // Max 10
int numObjects = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32-CAM Robot...");

  setupCamera();
  setupWiFi();
  setupWebServer(); // Initialize Web Server
}

void loop() {
  server.handleClient(); // Handle incoming web requests
  detectBall();
  delay(10); // Small delay
}

void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; // Fixed deprecated macro
  config.pin_sccb_scl = SIOC_GPIO_NUM; // Fixed deprecated macro
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565; // Use RGB565 for color detection
  config.frame_size = FRAMESIZE_QVGA;     // 320x240 for max FPS
  config.jpeg_quality = 30;               // Lower quality (higher number) = higher FPS
  config.fb_count = 2;                    // Double buffering for smoother stream (Requires PSRAM)

  // Note: These pins correspond to CAMERA_MODEL_AI_THINKER

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  Serial.println("Camera initialized successfully");
}

void setupWiFi() {
  WiFi.mode(WIFI_AP); 
  
  // Try to start AP with a specific channel (e.g., channel 1) 
  bool result = WiFi.softAP(WIFI_SSID, WIFI_PASS, 1, 0, 4); 

  if (result) {
    Serial.println("WiFi AP Started successfully!");
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  } else {
    Serial.println("WiFi AP Failed to start!");
  }
}

// Function to handle streaming
// --- Minimal Font 5x7 ---
const uint8_t font5x7[][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
  {0x00, 0x00, 0x00, 0x00, 0x00}, // Space (10)
  {0x00, 0x36, 0x36, 0x00, 0x00}, // : (11)
  {0x63, 0x14, 0x08, 0x14, 0x63}, // X (12)
  {0x07, 0x08, 0x70, 0x08, 0x07}, // Y (13)
};

int getFontIndex(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c == ' ') return 10;
  if (c == ':') return 11;
  if (c == 'X') return 12;
  if (c == 'Y') return 13;
  return 10;
}

void drawChar(camera_fb_t *fb, int x, int y, char c, uint16_t color) {
  int idx = getFontIndex(c);
  for (int i = 0; i < 5; i++) {
    uint8_t line = font5x7[idx][i];
    for (int j = 0; j < 7; j++) {
      if (line & (1 << j)) {
        int px = x + i;
        int py = y + j;
        if (px >= 0 && px < fb->width && py >= 0 && py < fb->height) {
             int index = (py * fb->width + px) * 2;
             fb->buf[index] = (color >> 8) & 0xFF;
             fb->buf[index+1] = color & 0xFF; 
        }
      }
    }
  }
}

void drawString(camera_fb_t *fb, int x, int y, String str, uint16_t color) {
  for (int i = 0; i < str.length(); i++) {
    drawChar(fb, x + (i * 6), y, str[i], color);
  }
}

void handleStream() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (client.connected()) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      break;
    }

    // --- Multi-Object Detection (Grid Clustering) ---
    // Grid Size: 320x240 -> 16x12 grid (20x20 blocks)
    #define GRID_W 16
    #define GRID_H 12
    #define BLOCK_SIZE 20
    
    uint8_t grid[GRID_H][GRID_W] = {0}; // Store pixel counts
    
    // 1. Fill Grid
    // Optimization: Check every 4th pixel (4x4 subsampling) -> 16x faster color check
    // We still map to 20x20 blocks
    for (int y = 0; y < fb->height; y += 4) {
      for (int x = 0; x < fb->width; x += 4) {
        int index = (y * fb->width + x) * 2;
        uint8_t r = (fb->buf[index] & 0xF8);
        uint8_t g = ((fb->buf[index] & 0x07) << 5) | ((fb->buf[index+1] & 0xE0) >> 3);
        uint8_t b = (fb->buf[index+1] & 0x1F) << 3;

        // Restore Color Detection (Orange/Yellow)
        // High Red, Medium Green, Low Blue
        // Thresholds: R > 150, G > 50, B < 110
        if (r > 150 && g > 50 && b < 110) { 
           int gx = x / BLOCK_SIZE;
           int gy = y / BLOCK_SIZE;
           if (gx < GRID_W && gy < GRID_H) {
             if (grid[gy][gx] < 255) grid[gy][gx]++;
           }
        }
      }
    }

    // 2. Cluster & Process
    bool visited[GRID_H][GRID_W] = {false};
    
    // Reset global list
    struct ObjInfo {
        int x, y, pixels;
        int min_gx, max_gx, min_gy, max_gy;
    };
    ObjInfo candidates[10]; // Max 10 candidates
    int candidateCount = 0;

    for (int gy = 0; gy < GRID_H; gy++) {
      for (int gx = 0; gx < GRID_W; gx++) {
        // Threshold: Block must have significant pixels
        // Since we subsample 4x4, we have fewer hits per block. 
        // 20x20 block = 400 pixels. 4x4 subsample = 25 checks.
        // Threshold > 2 hits seems reasonable for a valid colorful block.
        if (grid[gy][gx] > 2 && !visited[gy][gx]) {
           // New Cluster / Flood Fill
           
           int qx[GRID_W * GRID_H];
           int qy[GRID_W * GRID_H];
           int head = 0, tail = 0;
           
           qx[tail] = gx; qy[tail] = gy; tail++;
           visited[gy][gx] = true;
           
           int min_gx = gx, max_gx = gx;
           int min_gy = gy, max_gy = gy;
           int totalClusterPixels = 0;
           
           while(head < tail) {
             int cx = qx[head];
             int cy = qy[head];
             head++;
             
             totalClusterPixels += grid[cy][cx];
             
             if (cx < min_gx) min_gx = cx;
             if (cx > max_gx) max_gx = cx;
             if (cy < min_gy) min_gy = cy;
             if (cy > max_gy) max_gy = cy;
             
             int dx[] = {-1, 1, 0, 0};
             int dy[] = {0, 0, -1, 1};
             
             for (int i=0; i<4; i++) {
               int nx = cx + dx[i];
               int ny = cy + dy[i];
               
               if (nx >=0 && nx < GRID_W && ny >= 0 && ny < GRID_H) {
                 if (grid[ny][nx] > 2 && !visited[ny][nx]) {
                   visited[ny][nx] = true;
                   qx[tail] = nx; qy[tail] = ny; tail++;
                 }
               }
             }
           }
           
           // Validations
           int w_blocks = (max_gx - min_gx + 1);
           int h_blocks = (max_gy - min_gy + 1);
           
           float ratio = (float)w_blocks / h_blocks;
           
           // Strictness:
           // Total pixels will be smaller due to subsampling. 
           // > 10 subsampled hits (approx 160 real pixels coverage?)
           if (totalClusterPixels > 10 && ratio > 0.6 && ratio < 1.6) {
              if (candidateCount < 10) {
                  int cX = ((min_gx + max_gx) * BLOCK_SIZE) / 2 + (BLOCK_SIZE/2);
                  int cY = ((min_gy + max_gy) * BLOCK_SIZE) / 2 + (BLOCK_SIZE/2);
                  
                  // Store real pixels estimate = subsampled * 16? 
                  // Actually just use subsampled for sorting, scale for display if needed.
                  // Or just keep it relative. sorting works.
                  // For distance calc, we need to adjust K.
                  candidates[candidateCount] = {cX, cY, totalClusterPixels * 16, min_gx, max_gx, min_gy, max_gy};
                  candidateCount++;
              }
           }
        }
      }
    }

    // 3. Global Logic - Sort & Label
    // Sort candidates by pixels (Descending) -> Largest = Closest
    for (int i = 0; i < candidateCount - 1; i++) {
        for (int j = 0; j < candidateCount - i - 1; j++) {
            if (candidates[j].pixels < candidates[j + 1].pixels) {
                ObjInfo temp = candidates[j];
                candidates[j] = candidates[j + 1];
                candidates[j + 1] = temp;
            }
        }
    }
    
    // --- Focus Effect: Blur Background ---
    // Iterate 4x4 blocks. If NOT in a candidate box, pixelate it.
    for (int y = 0; y < fb->height; y += 4) {
      for (int x = 0; x < fb->width; x += 4) {
          
          bool isClean = false;
          // Check if inside any candidate
          for (int c = 0; c < candidateCount; c++) {
              int px_min_x = candidates[c].min_gx * BLOCK_SIZE;
              int px_max_x = (candidates[c].max_gx + 1) * BLOCK_SIZE;
              int px_min_y = candidates[c].min_gy * BLOCK_SIZE;
              int px_max_y = (candidates[c].max_gy + 1) * BLOCK_SIZE;
              
              // If this 4x4 block overlaps with object box (simple check: if top-left is inside? or nearby)
              if (x >= px_min_x && x <= px_max_x && y >= px_min_y && y <= px_max_y) {
                  isClean = true;
                  break;
              }
          }
          
          if (!isClean) {
              // Apply Blur/Pixelate
              // Get color of top-left pixel
              int index = (y * fb->width + x) * 2;
              uint8_t c1 = fb->buf[index];
              uint8_t c2 = fb->buf[index+1];
              
              // Fill 4x4 block with this color
              for (int by = 0; by < 4; by++) {
                  for (int bx = 0; bx < 4; bx++) {
                      int fy = y + by;
                      int fx = x + bx;
                      if (fy < fb->height && fx < fb->width) {
                          int fidx = (fy * fb->width + fx) * 2;
                          fb->buf[fidx] = c1;
                          fb->buf[fidx+1] = c2;
                      }
                  }
              }
          }
      }
    }

    numObjects = 0;
    
    for (int i = 0; i < candidateCount; i++) {
        objects[numObjects].x = candidates[i].x;
        objects[numObjects].y = candidates[i].y;
        objects[numObjects].pixels = candidates[i].pixels;
        
        // Label: Object 1, Object 2...
        String label = "Object " + String(i + 1);
        objects[numObjects].label = label;
        
        // Distance Estimation (Heuristic)
        // Tune K: Assume 1000 pixels = 40cm? 
        // Formula: dist = K / sqrt(pixels)
        // Let's try K = 1200.0 based on typical 320x240 view
        float dist_cm = 1200.0 / sqrt((float)candidates[i].pixels);
        if (dist_cm < 5) dist_cm = 5; // Min cap
        if (dist_cm > 200) dist_cm = 200; // Max cap
        
        // Draw Bounding Box (Green)
        int px_min_x = candidates[i].min_gx * BLOCK_SIZE;
        int px_max_x = (candidates[i].max_gx + 1) * BLOCK_SIZE - 1;
        int px_min_y = candidates[i].min_gy * BLOCK_SIZE;
        int px_max_y = (candidates[i].max_gy + 1) * BLOCK_SIZE - 1;

        // Box
        for (int x = px_min_x; x <= px_max_x; x++) {
             int idx1 = (px_min_y * fb->width + x) * 2;
             fb->buf[idx1] = 0x07; fb->buf[idx1+1] = 0xE0;
             int idx2 = (px_max_y * fb->width + x) * 2;
             fb->buf[idx2] = 0x07; fb->buf[idx2+1] = 0xE0;
        }
        for (int y = px_min_y; y <= px_max_y; y++) {
             int idx1 = (y * fb->width + px_min_x) * 2;
             fb->buf[idx1] = 0x07; fb->buf[idx1+1] = 0xE0;
             int idx2 = (y * fb->width + px_max_x) * 2;
             fb->buf[idx2] = 0x07; fb->buf[idx2+1] = 0xE0;
        }
        
        // Draw Red Crosshair & Coordinates
        int cx = candidates[i].x;
        int cy = candidates[i].y;
        int size = 10;
        
        // Horizontal Line
        for (int x = cx - size; x <= cx + size; x++) {
            if (x >= 0 && x < fb->width) {
                int idx = (cy * fb->width + x) * 2;
                fb->buf[idx] = 0xF8; fb->buf[idx+1] = 0x00; 
            }
        }
        // Vertical Line
        for (int y = cy - size; y <= cy + size; y++) {
            if (y >= 0 && y < fb->height) {
                int idx = (y * fb->width + cx) * 2;
                fb->buf[idx] = 0xF8; fb->buf[idx+1] = 0x00;
            }
        }
        
        // Draw Text: "Object 1 (25cm)"
        String textStr = label + " (" + String((int)dist_cm) + "cm)";
        int tx = px_min_x;
        int ty = px_min_y - 10;
        if (ty < 0) ty = px_max_y + 5; 
        
        drawString(fb, tx, ty, textStr, 0xFFFF); // White Text

        numObjects++;
    }

    ballDetected = (numObjects > 0);
    if (ballDetected) {
       // Main object is closest (index 0)
       ballX = candidates[0].x;
       ballY = candidates[0].y;
       lastPixelCount = candidates[0].pixels;
    }

    // Convert RGB565 to JPEG for streaming
    // Quality 30 for higher FPS
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool jpeg_converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 30, &jpg_buf, &jpg_len);
    
    esp_camera_fb_return(fb); // Release original frame buffer

    if (!jpeg_converted) {
      Serial.println("JPEG compression failed");
      break;
    }

    // Send JPEG Frame
    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n\r\n";
    server.sendContent(response);
    
    client.write(jpg_buf, jpg_len);
    server.sendContent("\r\n");
    
    free(jpg_buf); // Release JPEG buffer
    
    // Allow other tasks to run?
    // yield(); 
  }
}



void setupWebServer() {
  // Serve the HTML page
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", index_html);
  });

  // API to get status
  server.on("/status", HTTP_GET, []() {
    
    // Calculate Distance String based on pixel count
    // NOTE: Pixel count is updated in handleStream OR detectBall
    // Since handleStream is a loop, we need to ensure pixel_count is accessible or stored globally
    // Currently pixel_count is local to handleStream. We should make it global or passed.
    
    // Let's use the 'lastPixelCount' we added (needs to be updated in handleStream/detectBall)
    
    if (ballDetected) {
       // Heuristic for distance (QVGA 320x240 = 76800 pixels)
       // Very close: > 1000 pixels (subsampled) -> > 4000 pixels real
       // Close: 500 - 1000
       // Medium: 100 - 500
       // Far: < 100
       
       if (lastPixelCount > 1000) distStr = "Very Close (< 20cm)";
       else if (lastPixelCount > 400) distStr = "Close (20-50cm)";
       else if (lastPixelCount > 100) distStr = "Medium (50cm-1m)";
       else distStr = "Far (> 1m)";
    } else {
       distStr = "-";
    }

    String json = "{";
    json += "\"detected\":";
    json += ballDetected ? "true" : "false";
    json += ", \"x\":";
    json += String(ballX);
    json += "\", \"object_count\":";
    json += String(numObjects);
    json += ", \"objects\":[";
    for (int i=0; i < numObjects; i++) {
        if (i > 0) json += ",";
        
        // Calc distance again for JSON (or store it in struct, but calc is cheap)
        float dist_cm = 1200.0 / sqrt((float)objects[i].pixels);
        if (dist_cm < 5) dist_cm = 5;
        if (dist_cm > 200) dist_cm = 200;
        String distStr = String((int)dist_cm) + " cm";

        json += "{";
        json += "\"label\":\"" + objects[i].label + "\",";
        json += "\"x\":" + String(objects[i].x) + ",";
        json += "\"y\":" + String(objects[i].y) + ",";
        json += "\"pixels\":" + String(objects[i].pixels) + ",";
        json += "\"distance_str\":\"" + distStr + "\"";
        json += "}";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
  });

  // MJPEG Stream
  server.on("/stream", HTTP_GET, handleStream);

  server.begin();
  Serial.println("Web Server started!");
}

void detectBall() {
   // This function is now redundant if streaming is active, 
   // BUT we keep it for when the stream is NOT active (loop fallback).
   
   // NOTE: We need to handle client connection. If client is connected to /stream, 
   // the handleStream loop takes over. 
   // If NO client is streaming, we still want to detect?
   // For now, let's just do detection inside handleStream (when viewing) 
   // and here (when not viewing).
  
   // ... [Duplicate Logic for non-streaming detection] ...
   // Ideally, we refactor detection into a helper function.
   
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  uint32_t x_sum = 0;
  uint32_t y_sum = 0;
  uint32_t pixel_count = 0;

  for (int y = 0; y < fb->height; y++) {
    for (int x = 0; x < fb->width; x++) {
      int index = (y * fb->width + x) * 2;
      uint8_t byte1 = fb->buf[index];
      uint8_t byte2 = fb->buf[index + 1];
      uint8_t r = (byte1 & 0xF8);
      uint8_t g = ((byte1 & 0x07) << 5) | ((byte2 & 0xE0) >> 3);
      uint8_t b = (byte2 & 0x1F) << 3;

      if (r > 140 && g > 70 && b < 110) {
         x_sum += x;
         y_sum += y;
         pixel_count++;
      }
    }
  }

  if (pixel_count > 10) { 
    ballDetected = true;
    ballX = x_sum / pixel_count;
    ballY = y_sum / pixel_count;
  } else {
    ballDetected = false;
  }
  
  esp_camera_fb_return(fb);
}