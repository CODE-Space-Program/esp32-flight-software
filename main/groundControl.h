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
        fetchFlightId();
        requestTimer.attach(1.0, std::bind(&GroundControl::makeRequest, this));
    }

    void sendTelemetry(const Data &data)
    {
        StaticJsonDocument<256> logEntry;
        logEntry["altitude"] = data.estimated_altitude;
        logEntry["velocity"] = data.velocity;
        logEntry["temperature"] = data.temperature;
        logEntry["gyro_x"] = data.gyro.x;
        logEntry["gyro_y"] = data.gyro.y;
        logEntry["gyro_z"] = data.gyro.z;
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
        http.begin(baseUrl + "/api/flights");
        http.addHeader("Content-Type", "application/json");
        int httpCode = http.POST("{}");

        if (httpCode != 200)
        {
            http.end();
            Serial.println("Error: Failed to fetch flight ID, HTTP " + String(httpCode));
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
            http.begin(baseUrl + "/api/flights/" + flightId + "/logs");
            http.addHeader("Content-Type", "application/json");
            http.addHeader("Authorization", "Bearer " + token);
            int httpCode = http.POST(jsonPayload);

            if (httpCode != 200)
            {
                Serial.println("Error: Failed to send telemetry, HTTP " + String(httpCode));
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
