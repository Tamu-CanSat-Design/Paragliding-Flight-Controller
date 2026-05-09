#include <Arduino.h>
#include "Common.h"

namespace{
    Common::Window<5> testWindow;

    float ascendingData[] = {-1, -2, -3, -4, -5};
    float descendingData[] = {-5, -4, -3, -2, -1};
    float staticData[] = {-10.0, -10.02, -9.98, -10.01, -9.99};
    unsigned long lastUpdate = 0;
    int dataIndex = 0;
    int testState = 0; //0 = ascending, 1 = descending, 2 = static
}



void setup(){
    Serial.begin(115200);
    delay(1000);




}

void loop(){

    if(millis() - lastUpdate >= 1000) {

        lastUpdate = millis();

        switch(testState) {

            // ==========================================
            // ASCENDING TEST
            // ==========================================
            case 0:

                testWindow.push(ascendingData[dataIndex]);

                Serial.println("ASCENDING TEST");
                Serial.print("Pushed: ");
                Serial.println(ascendingData[dataIndex]);

                dataIndex++;

                if(dataIndex >= 5) {

                    Serial.print("Average: ");
                    Serial.println(testWindow.average());

                    Serial.print("isAscending: ");
                    Serial.println(testWindow.isAscending());

                    Serial.print("isDescending: ");
                    Serial.println(testWindow.isDescending());

                    Serial.print("isStatic: ");
                    Serial.println(testWindow.isStatic());

                    Serial.println();

                    testWindow.clear();

                    dataIndex = 0;
                    testState = 1;
                }

                break;

            // ==========================================
            // DESCENDING TEST
            // ==========================================
            case 1:

                testWindow.push(descendingData[dataIndex]);

                Serial.println("DESCENDING TEST");
                Serial.print("Pushed: ");
                Serial.println(descendingData[dataIndex]);

                dataIndex++;

                if(dataIndex >= 5) {

                    Serial.print("Average: ");
                    Serial.println(testWindow.average());

                    Serial.print("isAscending: ");
                    Serial.println(testWindow.isAscending());

                    Serial.print("isDescending: ");
                    Serial.println(testWindow.isDescending());

                    Serial.print("isStatic: ");
                    Serial.println(testWindow.isStatic());

                    Serial.println();

                    testWindow.clear();

                    dataIndex = 0;
                    testState = 2;
                }

                break;

            // ==========================================
            // STATIC TEST
            // ==========================================
            case 2:

                testWindow.push(staticData[dataIndex]);

                Serial.println("STATIC TEST");
                Serial.print("Pushed: ");
                Serial.println(staticData[dataIndex], 3);

                dataIndex++;

                if(dataIndex >= 5) {

                    Serial.print("Average: ");
                    Serial.println(testWindow.average(), 3);

                    Serial.print("isAscending: ");
                    Serial.println(testWindow.isAscending());

                    Serial.print("isDescending: ");
                    Serial.println(testWindow.isDescending());

                    Serial.print("isStatic: ");
                    Serial.println(testWindow.isStatic());

                    Serial.println();
                    Serial.println("Restarting Tests...");
                    Serial.println();

                    testWindow.clear();

                    dataIndex = 0;
                    testState = 0;
                }

                break;
        }
    }
    
}