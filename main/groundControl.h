#pragma once

#include <vector>
#include <functional>

#include <NetBIOS.h>
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>

#include "sensors.h"

/**
 * @class GroundControl
 * @brief Handles two-way communication with the ground control server (our backend) via HTTP.
 *
 * Example usage:
 * @code
 * GroundControl groundControl("https://spaceprogram.bolls.dev");
 *
 * groundControl.connect();
 *
 * groundControl.subscribe([](const String &command, const JsonVariant &args) {
 *     // handle incoming command
 * });
 *
 * groundControl.sendTelemetry(datapoint);
 * @endcode
 */
class GroundControl
{
public:
    using Listener = std::function<void(const String &command, const JsonVariant &args)>;

    GroundControl(const String &baseUrl) : baseUrl(baseUrl)
    {
    }

    /**
     * Subscribes a callback to be triggered on commands received from our backend.
     *
     * @param handler A lambda that takes the command name and JSON arguments.
     */
    void subscribe(Listener listener)
    {
        listeners.push_back(listener);
    }

    bool isConnecting = false;
    bool isConnected = false;

    /**
     * Connects to the backend and starts polling for commands.
     */
    void connect()
    {
        if (isConnecting || isConnected)
            return;

        isConnecting = true;

        fetchFlightId();
        requestTimer.attach(1.0, std::bind(&GroundControl::makeRequest, this));

        isConnecting = false;
        isConnected = true;
    }

    /**
     * Queues a telemetry log to be sent to the backend.
     * The queued logs are batched and sent once a second.
     *
     * @param data The telemetry data to send.
     */
    void sendTelemetry(const Data &data)
    {
        StaticJsonDocument<256> logEntry;
        logEntry["raw_altitude"] = data.raw_altitude;
        logEntry["altitude"] = data.estimated_altitude;
        logEntry["velocity"] = data.velocity;
        logEntry["pitch"] = data.estimated_pitch;
        logEntry["yaw"] = data.estimated_yaw;
        logEntry["roll"] = data.estimated_roll;
        logEntry["temperature"] = data.temperature;
        logEntry["sent"] = millis();
        logEntry["state"] = data.state;
        logEntry["nominalPitchServoDegrees"] = data.nominalPitchServoDegrees;
        logEntry["nominalYawServoDegrees"] = data.nominalYawServoDegrees;
        logEntry["servosLocked"] = data.servosLocked;
        logEntry["seaLevelPressure"] = data.sea_level_pressure;

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
        http.setTimeout(3000);
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
            if (httpCode < 0)
            {
                debugNetworkConnection();
            }
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
            JsonVariant args = obj["args"];

            Serial.println("processResponse command: " + command);

            serializeJsonPretty(args, Serial);
            Serial.println();

            emitEvent(command, args);
        }
    }

    void emitEvent(const String &command, const JsonVariant &args)
    {
        for (auto &listener : listeners)
        {
            Serial.println("emitEvent command: " + command);

            serializeJsonPretty(args, Serial);
            Serial.println();

            listener(command, args);
        }
    }

    void
    debugNetworkConnection()
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("Error: WiFi not connected");
            return;
        }
        Serial.println("Debugging network connection...");

        Serial.println("✓ WiFi is connected");

        Serial.setDebugOutput(true); // Enable WiFi debug output

        WiFiClient client;
        IPAddress serverIP;
        IPAddress ipifyIP;
        String host = "spaceprogram.bolls.dev";

        if (!WiFi.hostByName(host.c_str(), serverIP))
        {
            Serial.println("× Failed to resolve ip of " + host);
        }
        else
        {
            Serial.println("✓ Resolved ip of " + host + " to " + serverIP.toString());
        }

        if (!WiFi.hostByName("api.ipify.org", ipifyIP))
        {
            Serial.println("× Failed to resolve ip of api.ipify.org");
        }
        else
        {
            Serial.println("✓ Resolved ip of api.ipify.org to " + ipifyIP.toString());
        }

        HTTPClient http;

        // Test 1: your own API
        http.begin(baseUrl + "/api/health");
        int httpCode = http.GET();
        String payload = http.getString();

        if (httpCode != 200)
        {
            Serial.println("× Failed to ping " + baseUrl + "/api/health, HTTP " + String(httpCode));
            Serial.println("Response: " + payload);
        }
        else
        {
            Serial.println("✓ Successfully pinged /api/health");
            Serial.println("Response: " + payload);
        }
        http.end();

        // Test 2: external API
        http.begin("https://api.ipify.org");
        httpCode = http.GET();
        payload = http.getString();

        if (httpCode != 200)
        {
            Serial.println("× Failed to ping https://api.ipify.org, HTTP " + String(httpCode));
            Serial.println("Response: " + payload);
        }
        else
        {
            Serial.println("✓ Successfully pinged https://api.ipify.org");
            Serial.println("Response: " + payload);
        }
        http.end();

        Serial.setDebugOutput(false); // Disable WiFi debug output
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