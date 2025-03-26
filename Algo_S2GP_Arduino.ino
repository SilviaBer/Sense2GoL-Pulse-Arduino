#include <IFXRadarPulsedDoppler.h>
#include <LED.h>
#include <arduinoFFT.h>

#define SAMPLES 256
#define SAMPLING_FREQ 20
#define BRPM_LOW 15
#define BRPM_HIGH 22


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

void myErrorCallback(uint32_t error) {
    Serial.print("ERROR: 0x");
    Serial.println(error, HEX);
    while(1);
}

void myRawDataCallback(raw_data_context_t context) {
    radarDev.getRawData(context, raw_i, raw_q, SAMPLES);
    
    // Stampa grezza dei dati
    Serial.println("Dati grezzi:");
    for(int i = 0; i < SAMPLES; i++) {
        Serial.print(raw_i[i]);
        Serial.print("\t");
        Serial.println(raw_q[i]);
    }
    
    newDataAvailable = true;
}


void processFFT() {
    //Serial.println("Eseguo FFT...");

    // Stampa i dati grezzi I e Q
    //Serial.println("I-Signal\tQ-Signal");
    for(int i = 0; i < SAMPLES; i++) {
       // Serial.print(raw_i[i]);
       // Serial.print("\t");
       // Serial.println(raw_q[i]);

        // Calcola il modulo (intensità) per la FFT
        vReal[i] = raw_i[i];   // Usa direttamente i valori grezzi
        vImag[i] = raw_q[i];   // Usa direttamente i valori grezzi
    }

    // Esegui la FFT senza finestra
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    // Trova il picco principale della FFT (maggiore intensità)
    int peakIndex = FFT.majorPeak();
    double peakFrequency = ((double)peakIndex * SAMPLING_FREQ) / SAMPLES;
    double BRPM = peakFrequency * 60; // Convertilo in BRPM (battiti per minuto)

    Serial.print("BRPM: ");
    Serial.println(BRPM);

    // Controllo della velocità di rotazione e gestione dei LED
    if(BRPM > 0) { // Scarta valori nulli
        if(BRPM < BRPM_LOW) {
            Led.On(LED_RED);
            Led.Off(LED_GREEN);
            Led.Off(LED_BLUE);
        } 
        else if(BRPM >= BRPM_LOW && BRPM <= BRPM_HIGH) {
            Led.Off(LED_RED);
            Led.On(LED_GREEN);
            Led.Off(LED_BLUE);
        } 
        else {
            Led.Off(LED_RED);
            Led.Off(LED_GREEN);
            Led.On(LED_BLUE);
        }
    }
}

void setup() {
    Serial.begin(500000); // Imposta baudrate alto per flusso dati più fluido
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

    // Analizza i dati solo una volta al secondo
    if(millis() - lastAnalysisTime >= 400) {
        if(newDataAvailable) {
            processFFT();
            newDataAvailable = false;
        }
        lastAnalysisTime = millis();
    }
}
