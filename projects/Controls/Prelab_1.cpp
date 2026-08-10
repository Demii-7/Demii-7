#include <iostream>
#include "daq.h" // DAQ header file 

using namespace std;

// --- Function Initialisations ---
float64 Read_Voltage();

// --- Main Program ---

int main() {
    // Define parameters for configuration
    float64 SAMPLE_RATE = 1000.0;   // 1 kHz frequency
    int32 TIMEOUT = 5;              // 5 second timeout
    float64 V_MIN = -10.0;          // Minimum voltage range
    float64 V_MAX = 10.0;           // Maximum voltage range

    // Configure Analog Input (AI 0)
    AI_Configuration(SAMPLE_RATE, TIMEOUT, V_MIN, V_MAX);

    // Configure Analog Output (AO 0)
    AO_Configuration(V_MIN, V_MAX);

    cout << "DAQ System initialized. Starting voltage follower..." << endl;

    // Control Loop
    while (true) {
        // Read the input voltage once a sample is ready using Read_Voltage()
        float64 V_IN = Read_Voltage();

        // Write the same voltage to the output (AO 0)
        Write_Voltage(V_IN);

    }

    return 0;
}

// --- Function Definitions ---

float64 Read_Voltage() {
    // Wait until the background callback sets sample_ready to 1
    while (sample_ready != 1) {
        // Waits for the hardware to finish a sample
    }

    // Acknowledge the sample by resetting the flag to 0
    sample_ready = 0;

    // Return the value stored by the callback
    return read_voltage[0];
}