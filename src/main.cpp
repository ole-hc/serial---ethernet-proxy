#include <Arduino.h>
#include <ETH.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

#define ETH_ADDR 0
#define ETH_POWER_PIN 5
#define ETH_MDC_PIN 23
#define ETH_MDIO_PIN 18
#define ETH_TYPE ETH_PHY_LAN8720
#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT

#define PORT 6638

#define RX_PIN 36
#define TX_PIN 4
#define SERIAL_CONFIG SERIAL_8E1

#define QUEUE_LENGTH 512

WiFiServer server(6638); 
WiFiClient client;

QueueHandle_t serialToEthernet;
QueueHandle_t ethernetToSerial;

TaskHandle_t serialToEthernetInQueue;
TaskHandle_t serialToEthernetOutQueue;
TaskHandle_t EthernetToSerialInQueue;
TaskHandle_t EthernetToSerialOutQueue;

void readSerial(void *pvParameters) {
    while (true)
    {
        if(Serial1.available()) {
            uint8_t incomingByte = Serial1.read();
            xQueueSend(serialToEthernet, &incomingByte, portMAX_DELAY);
            Serial.println("Added Serial input to serialToEthernet queue!");
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void printToEthernet(void *pvParameters) {
    while (true)
    {
        BaseType_t response;
        uint8_t incomingByte;
        response = xQueueReceive(serialToEthernet, &incomingByte, portMAX_DELAY);
        if(response == pdTRUE) { // data has been copied to the specified buffer.
            client.write(incomingByte);
            Serial.println("Send data from Serial queue to Ethernet");
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void readEthernet(void *pvParameters) {
    while (true)
    {
        if(client.available()) {
            uint8_t incomingByte = client.read();
            xQueueSend(ethernetToSerial, &incomingByte, portMAX_DELAY);
            Serial.println("Added Ethernet input to ethernetToSerial queue!");
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void printToSerial(void *pvParameters) {
    while (true)
    {
        BaseType_t response;
        uint8_t incomingByte;
        response = xQueueReceive(ethernetToSerial, &incomingByte, portMAX_DELAY);
        if(response == pdTRUE) { // data has been copied to the specified buffer.
            Serial1.write(incomingByte);
            Serial.println("Send data from Ethernet queue to Serial device");
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void setup() {
    Serial.begin(115200);

    ETH.begin(ETH_ADDR, ETH_POWER_PIN, ETH_MDC_PIN, ETH_MDIO_PIN, ETH_TYPE, ETH_CLK_MODE);
    while (!ETH.linkUp()) {
        delay(100);
        Serial.println("Waiting for ethernet link...");
    }
    Serial.println("Ethernet setup successfully!");
    Serial.print("IP-Address: ");
    Serial.println(ETH.localIP());
    server.begin();

    Serial1.begin(115200, SERIAL_CONFIG, RX_PIN, TX_PIN);
    Serial.println("Serial setup successfully!");

    serialToEthernet = xQueueCreate(QUEUE_LENGTH, sizeof(uint8_t));
    ethernetToSerial = xQueueCreate(QUEUE_LENGTH, sizeof(uint8_t));

    xTaskCreate(readSerial, "SerialToEthernet - Queue input", 4096, nullptr, 1, &serialToEthernetInQueue);
    xTaskCreate(printToEthernet, "SerialToEthernet - Queue output", 4096, nullptr, 1, &serialToEthernetOutQueue);
    xTaskCreate(readEthernet, "EthernetToSerial - Queue input", 4096, nullptr, 1, &EthernetToSerialInQueue);
    xTaskCreate(printToSerial, "EthernetToSerial - Queue output", 4096, nullptr, 1, &EthernetToSerialOutQueue);
}

void loop() {
    if (!client || !client.connected()) { 
        client = server.available(); 
    }
}
