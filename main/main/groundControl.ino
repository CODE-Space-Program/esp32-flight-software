#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <vector>
#include <functional>
#include <ArduinoJson.h>

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
    using Listener = std::function<void(const String &)>

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
                makeRequest(); // Start polling once flightId is obtained
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

    void makeRequest()
    {
        if (flightId.isEmpty())
            return;

        HTTPClient http;
        http.begin(baseUrl + "/api/flights/" + flightId + "/events");
        int httpCode = http.GET();

        if (httpCode > 0)
        {
            String payload = http.getString();
            processResponse(payload);
        }

        http.end();
        makeRequest(); // Immediately send next request
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