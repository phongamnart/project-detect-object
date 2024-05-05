#include <esp_now.h>
#include <WiFi.h>

#include <espcamdetecobject_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <HTTPClient.h>
#include <ArtronShop_LineNotify.h>

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Camera model not selected"
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

#define WIFI_SSID "Terk"
#define WIFI_PASSWORD "12345678"
#define LINE_TOKEN "fbIzZs8EUDTvsoBZvdsdRMGIqnT3gULg7eVxzvG7MOb"

String message = "https%3A%2F%2Fwww.google.com%2Fmaps%2Fplace%2F";
String la = "7.012919";
String lon = "100.473209";

int SENSOR_PIN = A0;

int RPWM_Output = 22;
int LPWM_Output = 23;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

const int pingPin1 = 5;
const int pingPin2 = 6;
int inPin1 = 18;
int inPin2 = 19;

typedef struct struct_message
{
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

static bool debug_nn = false;
static bool is_initialised = false;
uint8_t *snapshot_buf;

uint8_t broadcastAddress[] = {0x30, 0xC6, 0xF7, 0x2A, 0x04, 0x84};

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
void sendLabelsOverESPNow(const char *labels, size_t length);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

static const int RXPin = 22, TXPin = 21;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

void setup()
{
    Serial.begin(115200);

    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);

    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(RPWM_Output, ledChannel);

    Serial.println("\nStarting continuous inference in 2 seconds...");
    delay(2000);

    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    Serial.println("start");

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("connecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println();
    Serial.print("connected: ");
    Serial.println(WiFi.localIP());

    esp_now_register_recv_cb(OnDataRecv);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
        Serial.println("Failed to add peer");
        return;
    }

    if (ei_camera_init() == false)
    {
        Serial.println("Failed to initialize Camera!");
    }
    else
    {
        Serial.println("Camera initialized");
        delay(5000);
    }
}

void loop()
{
    int sensorValue = map(analogRead(33), 0, 4095, 0, 255);
    Serial.println(sensorValue);

    digitalWrite(LPWM_Output, 0);
    ledcWrite(ledChannel, sensorValue);
    int sentA = 0;
    int sentB = 0;
    int sentC = 0;

    esp_err_t send_result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    if (send_result == ESP_OK)
    {
        Serial.println("Sent with success");
    }
    else
    {
        Serial.println("Error sending the data");
    }

    delay(2000);

    snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    if (snapshot_buf == nullptr)
    {
        Serial.println("ERR: Failed to allocate snapshot buffer!");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false)
    {
        Serial.println("Failed to capture image");
        free(snapshot_buf);
        return;
    }

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK)
    {
        Serial.printf("ERR: Failed to run classifier (%d)\n", err);
        free(snapshot_buf);
        return;
    }

    bool bb_found = result.bounding_boxes[0].value > 0;
    for (size_t ix = 0; ix < result.bounding_boxes_count; ix++)
    {
        auto bb = result.bounding_boxes[ix];
        if (bb.value == 0)
        {
            continue;
        }

        if (strcmp(bb.label, "A") == 0)
        {
            strcpy(myData.a, "A");
            myData.b = 0;
            myData.c = 0.0;
            myData.d = false;
            Serial.println("Type A detected");
        }
        else if (strcmp(bb.label, "B") == 0)
        {
            strcpy(myData.a, "B");
            myData.b = 1;
            myData.c = 0.0;
            myData.d = false;
            Serial.println("Type B detected");
        }
        else if (strcmp(bb.label, "C") == 0)
        {
            strcpy(myData.a, "C");
            myData.b = 0;
            myData.c = 1.2;
            myData.d = false;
            Serial.println("Type C detected");
        }

        esp_err_t send_result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
        if (send_result == ESP_OK)
        {
            Serial.println("Sent with success");
        }
        else
        {
            Serial.println("Error sending the data");
        }
    }

    free(snapshot_buf);

    delay(2000);

    gps_sen();
    delay(1000);

    long duration1, duration2, cm1, cm2;

    pinMode(pingPin1, OUTPUT);
    pinMode(pingPin2, OUTPUT);

    digitalWrite(pingPin1, LOW);
    digitalWrite(pingPin2, LOW);
    delayMicroseconds(2);
    digitalWrite(pingPin1, HIGH);
    digitalWrite(pingPin2, HIGH);
    delayMicroseconds(5);
    digitalWrite(pingPin1, LOW);
    digitalWrite(pingPin2, LOW);

    pinMode(inPin1, INPUT);
    pinMode(inPin2, INPUT);

    duration1 = pulseIn(inPin1, HIGH);
    duration2 = pulseIn(inPin2, HIGH);

    cm1 = microsecondsToCentimeters(duration1);
    cm2 = microsecondsToCentimeters(duration2);

    if (cm1 > 7)
    {
        Serial.println("Sensor 1: ระยะห่างเกินกำหนด!");
    }
    else
    {
        Serial.print("Sensor 1: ");
        Serial.print(cm1);
        Serial.println("cm");
    }

    if (cm2 > 7)
    {
        Serial.println("Sensor 2: ระยะห่างเกินกำหนด!");
    }
    else
    {
        Serial.print("Sensor 2: ");
        Serial.print(cm2);
        Serial.println("cm");
    }

    delay(1000);
}

long microsecondsToCentimeters(long microseconds)
{
    return microseconds / 29 / 2;
}

void gps_sen()
{
    while (gpsSerial.available() > 0)
    {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated())
        {
            la = String(gps.location.lat(), 6);
            lon = String(gps.location.lng(), 6);
            Serial.print("Latitude= ");
            Serial.print(la);
            Serial.print(" Longitude= ");
            Serial.println(lon);
            message = "https%3A%2F%2Fwww.google.com%2Fmaps%2Fplace%2F";
            message += la + "%2C%2B" + lon + "%2F%40" + la + "%40" + lon + "%2C18z  ";
            Line_Notify(message);
        }
    }
}

void Line_Notify(String message)
{
    HTTPClient http;

    http.begin("https://notify-api.line.me/api/notify");
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    http.addHeader("Authorization", "Bearer " + String(LINE_TOKEN));

    int httpResponseCode = http.POST("message=" + message);

    if (httpResponseCode > 0)
    {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
    }
    else
    {
        Serial.println("Error on sending POST: " + String(httpResponseCode));
    }

    http.end();
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Types of railway damage: ");
    Serial.println(myData.a);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        Serial.println("Data sent successfully");
    }
    else
    {
        Serial.println("Error sending data");
    }
}

bool ei_camera_init(void)
{
    if (is_initialised)
        return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID)
    {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

void ei_camera_deinit(void)
{
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK)
    {
        Serial.println("Camera deinit failed");
        return;
    }
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
    bool do_resize = false;

    if (!is_initialised)
    {
        Serial.println("ERR: Camera is not initialized");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb)
    {
        Serial.println("Camera capture failed");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

    esp_camera_fb_return(fb);

    if (!converted)
    {
        Serial.println("Conversion failed");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS))
    {
        do_resize = true;
    }

    if (do_resize)
    {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0)
    {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif