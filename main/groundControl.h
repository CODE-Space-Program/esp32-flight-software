#pragma once

#include <NetBIOS.h>

#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <vector>
#include <functional>
#include <ArduinoJson.h>
#include "sensors.h"
#include <Ticker.h>

/**
 * GroundControl class that connects to the Space Program API
 *
 * Usage:
 * GroundControl groundControl("https://spaceprogram.bolls.dev");
 *
 * groundControl.subscribe([](const String &command) {
 *     Serial.println("Received command: " + command);
 * });
 *
 * groundControl.connect();
 */
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
        StaticJsonDocument<256> doc;
        doc["altitude"] = data.estimated_altitude;
        doc["velocity"] = data.velocity;
        doc["temperature"] = data.temperature;
        doc["gyro_x"] = data.gyro.x;
        doc["gyro_y"] = data.gyro.y;
        doc["gyro_z"] = data.gyro.z;
        doc["pitch"] = data.estimated_pitch;
        doc["yaw"] = data.estimated_yaw;
        doc["sent"] = millis();

        String jsonPayload;
        serializeJson(doc, jsonPayload);

        HTTPClient http;
        http.begin(baseUrl + "/api/flights/" + flightId + "/logs");
        http.addHeader("Content-Type", "application/json");
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

private:
    String baseUrl;
    String flightId;
    std::vector<Listener> listeners;

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
};
