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
        logEntry["raw_altitude"] = data.raw_altitude;
        logEntry["altitude"] = data.estimated_altitude;
        logEntry["velocity"] = data.velocity;
        //logEntry["temperature"] = data.temperature;
        //logEntry["gyro_x"] = data.gyro.x;
        //logEntry["gyro_y"] = data.gyro.y;
        //logEntry["gyro_z"] = data.gyro.z;
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

    void fetchFlightId()
    {
        HTTPClient http;
        http.begin(baseUrl + "/api/flights");
        http.addHeader("Content-Type", "application/json");
        int httpCode = http.POST("{}");

        if (httpCode > 0)
        {
            String payload = http.getString();
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, payload);
            if (!error)
            {
                flightId = doc["data"]["flightId"].as<String>();
                token = doc["data"]["token"].as<String>();
                Serial.println("Flight ID: " + flightId);
                makeRequest();
            }
            else
            {
                Serial.println("Failed to parse flight ID");
            }
        }
        else
        {
            Serial.println("Failed to fetch flight ID");
        }
        http.end();
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

        if (httpCode > 0)
        {
            String payload = http.getString();
            processResponse(payload);
        }

        http.end();
    }

    void processResponse(const String &payload)
    {
        StaticJsonDocument<512> doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (error)
        {
            Serial.println("Failed to parse JSON");
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

            if (httpCode > 0)
            {
                Serial.println("Telemetry sent");
            }
            else
            {
                Serial.println("Failed to send telemetry");
            }
            http.end();
        }
    }
};
