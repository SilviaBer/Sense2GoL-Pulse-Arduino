#include <IFXRadarPulsedDoppler.h>
#include <LED.h>
#include <arduinoFFT.h>

#define SAMPLES 256
#define SAMPLING_FREQ 10
#define BRPM_LOW 12
#define BRPM_HIGH 25
#define WINDOW_SIZE 5  // Numero di prese

IFXRadarPulsedDoppler radarDev;
LED Led;

// Variabili per l'analisi FFT
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLING_FREQ);

// Buffer per i dati grezzi
float raw_i[SAMPLES];
float raw_q[SAMPLES];
bool newDataAvailable = false;
unsigned long lastAnalysisTime = 0;

// Variabili per rimuovere il background
double baselineBRPM = 0;
bool baselineSet = false;

// Buffer per i valori recenti
double bufferData[WINDOW_SIZE] = {0};  
int brpmIndex = 0;

double averageBreath(double newBRPM) {
    bufferData[brpmIndex] = newBRPM;
    brpmIndex = (brpmIndex + 1) % WINDOW_SIZE;

    double sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += bufferData[i];
    }
    return sum / WINDOW_SIZE;
}

void myErrorCallback(uint32_t error) {
    Serial.print("ERROR: 0x");
    Serial.println(error, HEX);
    while(1);
}

void myRawDataCallback(raw_data_context_t context) {
    radarDev.getRawData(context, raw_i, raw_q, SAMPLES);
    newDataAvailable = true;

}

void processFFT() {
    for (int i = 0; i < SAMPLES; i++) {
        // Finestra di Hann applicata sui dati reali e immaginari
        double window = 0.5 * (1 - cos(2 * PI * i / (SAMPLES - 1))); // Finestra di Hann
        vReal[i] = raw_i[i] * window; // Parte reale
        vImag[i] = raw_q[i] * window; // Parte immaginaria
    }

    // Esegui la FFT sui numeri complessi
    FFT.compute(FFT_FORWARD);

    // Trova il picco nella fascia 0.1 - 1.0 Hz
    int startIdx =(int) (0.1 * SAMPLES) / SAMPLING_FREQ;
    int endIdx =(int) (1.0 * SAMPLES) / SAMPLING_FREQ;
    int peakIndex = startIdx;
    double maxVal = 0;

    for (int i = startIdx; i <= endIdx; i++) {
    double magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
    if (magnitude > maxVal) {
        maxVal = magnitude;
        peakIndex = i;
    }
    }


    double peakFrequency = (double)peakIndex * SAMPLING_FREQ / SAMPLES;
    double BRPM = peakFrequency * 60.0;

    double smoothedBRPM = averageBreath(BRPM);
    
    //Stampa il valore stabilizzato
    Serial.print("BRPM stabilizzato: ");
    Serial.println(smoothedBRPM);

    //Gestione LED in base al valore stabilizzato
    if (smoothedBRPM < BRPM_LOW) {
        Led.On(LED_RED);
        Led.Off(LED_GREEN);
        Led.Off(LED_BLUE);
    } 
    else if (smoothedBRPM >= BRPM_LOW && smoothedBRPM <= BRPM_HIGH) {
        Led.On(LED_GREEN);
        Led.Off(LED_RED);
        Led.Off(LED_BLUE);
    } 
    else {
        Led.On(LED_BLUE);
        Led.Off(LED_RED);
        Led.Off(LED_GREEN);
    }
}


void setup() {
    Serial.begin(500000);
    delay(1000);

    Led.Add(LED_RED);
    Led.Add(LED_GREEN);
    Led.Add(LED_BLUE);
    Led.Off(LED_RED);
    Led.Off(LED_GREEN);
    Led.Off(LED_BLUE);

    radarDev.initHW();
    radarDev.setSkipSamples(0);

    uint32_t minFramePeriod = radarDev.getMinFramePeriod();
    radarDev.end();
    radarDev.setFramePeriod(minFramePeriod);

    radarDev.registerErrorCallback(myErrorCallback);
    radarDev.registerRawDataCallback(myRawDataCallback);

    radarDev.begin();
    Serial.println("Sistema pronto");
}

void loop() {
    radarDev.run();

    if (newDataAvailable) {
        processFFT();
        newDataAvailable = false;
    }
}

