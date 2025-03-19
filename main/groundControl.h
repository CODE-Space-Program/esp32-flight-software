#pragma once

#include <NetBIOS.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <vector>
#include <functional>
#include <ArduinoJson.h>
#include "sensors.h"
#include <Ticker.h>

class GroundControl
{
public:
    using Listener = std::function<void(const String &)>;

    GroundControl(const String &baseUrl) : baseUrl(baseUrl)
    {
    }

    void subscribe(Listener listener)
    {
        listeners.push_back(listener);
    }

    void connect()
    {
        performApiHealthcheck();
        fetchFlightId();
        requestTimer.attach(1.0, std::bind(&GroundControl::makeRequest, this));
    }

    void sendTelemetry(const Data &data)
    {
        StaticJsonDocument<256> logEntry;
        logEntry["raw_altitude"] = data.raw_altitude;
        logEntry["altitude"] = data.estimated_altitude;
        logEntry["velocity"] = data.velocity;
        // logEntry["temperature"] = data.temperature;
        // logEntry["gyro_x"] = data.gyro.x;
        // logEntry["gyro_y"] = data.gyro.y;
        // logEntry["gyro_z"] = data.gyro.z;
        logEntry["pitch"] = data.estimated_pitch;
        logEntry["yaw"] = data.estimated_yaw;
        logEntry["sent"] = millis();

        telemetryBuffer.push_back(logEntry);
        checkAndSendTelemetry();
    }

private:
    String baseUrl;
    String flightId;
    String token;
    std::vector<Listener> listeners;
    std::vector<StaticJsonDocument<256>> telemetryBuffer;

    int fetchFlightId()
    {
        HTTPClient http;
        http.setTimeout(5000);
        http.begin(baseUrl + "/api/flights");
        http.addHeader("Content-Type", "application/json");

        unsigned long startTime = millis();
        int httpCode = http.POST("{}");
        unsigned long endTime = millis();

        if (httpCode != 200)
        {
            http.end();
            Serial.println("Error: Failed to fetch flight ID, HTTP " + String(httpCode));
            Serial.println("Request timed out or failed in " + String(endTime - startTime) + " ms");
            return 1;
        }

        String payload = http.getString();
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, payload);
        http.end();

        if (error)
        {
            Serial.println("Error: Failed to parse flight ID JSON");
            return 1;
        }

        flightId = doc["data"]["flightId"].as<String>();
        token = doc["data"]["token"].as<String>();
        Serial.println("Flight ID: " + flightId);
        return 0;
    }

    unsigned long lastRequestTime = 0;
    const unsigned long requestInterval = 0;
    Ticker requestTimer;

    void makeRequest()
    {
        if (flightId.isEmpty() || millis() - lastRequestTime < requestInterval)
            return;

        lastRequestTime = millis();

        HTTPClient http;
        http.begin(baseUrl + "/api/flights/" + flightId + "/events");
        http.addHeader("Authorization", "Bearer " + token);
        int httpCode = http.GET();

        if (httpCode != 200)
        {
            Serial.println("Error: Failed to fetch events, HTTP " + String(httpCode));
            http.end();
            return;
        }

        String payload = http.getString();
        http.end();
        processResponse(payload);
    }

    void processResponse(const String &payload)
    {
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (error)
        {
            Serial.println("Error: Failed to parse events JSON");
            return;
        }

        JsonArray data = doc["data"].as<JsonArray>();
        for (JsonObject obj : data)
        {
            String command = obj["command"].as<String>();
            emitEvent(command);
        }
    }

    void emitEvent(const String &command)
    {
        for (auto &listener : listeners)
        {
            listener(command);
        }
    }

    void performApiHealthcheck()
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("Error: WiFi not connected");
            return;
        }

        Serial.println("WiFi is connected, checking API health...");

        WiFiClient client;
        IPAddress serverIP;
        IPAddress ipifyIP;
        String host = "spaceprogram.bolls.dev";

        if (!WiFi.hostByName(host.c_str(), serverIP))
        {
            Serial.println("Error: Failed to resolve host " + host);
            return;
        }
        Serial.println("Resolved " + host + " to " + serverIP.toString());

        if (!WiFi.hostByName("api.ipify.org", ipifyIP))
        {
            Serial.println("Error: Failed to resolve host api.ipify.org");
            return;
        }
        Serial.println("Resolved api.ipify.org to " + ipifyIp.toString());

        Serial.println("Pinging " + baseUrl + "/api/health");

        HTTPClient http;
        http.begin(baseUrl + "/api/health");
        int httpCode = http.GET();

        if (httpCode != 200)
        {
            Serial.println("Error: Failed to perform API healthcheck, HTTP " + String(httpCode));
            Serial.println("Response: " + http.getString());
        }
        else
        {
            Serial.println("API healthcheck successful");
        }
        http.end();

        Serial.println("Pinging https://api.ipify.org");

        http.begin("https://api.ipify.org");
        httpCode = http.GET();

        if (httpCode != 200)
        {
            Serial.println("Error: Failed to ping ipify, HTTP " + String(httpCode));
            Serial.println("Response: " + http.getString());
        }
        else
        {
            Serial.println("Public IP: " + http.getString());
        }
        http.end();
    }

    void checkAndSendTelemetry()
    {
        if (telemetryBuffer.empty())
            return;

        unsigned long oldestTimestamp = telemetryBuffer.front()["sent"].as<unsigned long>();
        if (millis() - oldestTimestamp >= 1000)
        {
            DynamicJsonDocument doc(1024);
            JsonArray logs = doc.to<JsonArray>();

            for (auto &entry : telemetryBuffer)
            {
                logs.add(entry);
            }

            telemetryBuffer.clear();

            String jsonPayload;
            serializeJson(doc, jsonPayload);

            HTTPClient http;
            http.setTimeout(5000);
            http.begin(baseUrl + "/api/flights/" + flightId + "/logs");
            http.addHeader("Content-Type", "application/json");
            http.addHeader("Authorization", "Bearer " + token);

            unsigned long startTime = millis();
            int httpCode = http.POST(jsonPayload);
            unsigned long endTime = millis();

            if (httpCode != 200)
            {
                Serial.println("Error: Failed to send telemetry, HTTP " + String(httpCode));
                Serial.println("Request timed out or failed in " + String(endTime - startTime) + " ms");
                http.end();
                return;
            }

            String response = http.getString();
            http.end();

            StaticJsonDocument<256> responseDoc;
            DeserializationError error = deserializeJson(responseDoc, response);
            if (error || !responseDoc["data"]["ok"].as<bool>())
            {
                Serial.println("Error: Invalid response from telemetry endpoint");
            }
            else
            {
                Serial.println("Telemetry sent successfully");
            }
        }
    }
};
