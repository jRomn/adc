#ifndef ADC_H
#define ADC_H

// =============================
// Header Files 
// =============================

    /* --- General --- */
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"     // For QueueHandle_t

    /* --- ADC --- */
    #include "esp_adc/adc_oneshot.h"    // For ADC HW interation
    #include "esp_adc/adc_cali.h"       // For voltage calibration

// =============================
// Application Log Tag
// =============================
   
    #define ADC_TAG "ADC"   // ESP_LOGI (Info)

// =============================
// Public Globals (Handles)
// =============================
extern adc_oneshot_unit_handle_t adc_handle;        // ADC Driver Handle
extern adc_cali_handle_t         adc_cali_handle;   // ADC Calibration Driver Handle

// =============================
// Publich ADC Configuration (Exposed for ADC.c )
// =============================
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_6   // GPIO34
#define BUFFER_SIZE             256             // Circular buffer length // <--- STEP 2.1
#define ADC_SAMPLE_PERIOD_MS    2.0             // Sampling period (ms)
#define SAMPLE_RATE_HZ          (1000 / ADC_SAMPLE_PERIOD_MS)  // Derived rate

// =============================
// Public Cognitive State Type Definition
// =============================
typedef enum {
    COGNITIVE_STATE_UNKNOWN = 0,
    COGNITIVE_STATE_DELTA,      // 0.5 –  4 Hz  — deep sleep
    COGNITIVE_STATE_THETA,      //   4 –  8 Hz  — drowsiness, meditation
    COGNITIVE_STATE_ALPHA,      //   8 – 12 Hz  — relaxed focus
    COGNITIVE_STATE_BETA,       //  12 – 30 Hz  — active thinking, stress
} cognitive_state_t;


typedef struct {    
    int16_t           filtered_value;   // Bandpass-filtered EEG sample (µV scaled)
    uint32_t          blink_count;      // Running blink event counter
    cognitive_state_t cognitive_state;  // Current dominant EEG band classification
} eeg_frame_t;


// =============================
// Unity Helper Function : Push New Sample into ADC Buffer
// =============================
void adc_push_sample(int16_t sample);


// =============================
// Main Functions: Public APIs
// =============================

    /* ---> STEP 1.1 : Initialize ADC Unit-> adc.c */
    /* ADC Unit Initialization + Channel Configuration + Calibration */
    adc_oneshot_unit_handle_t adc_init(void);

    /* ---> STEP 4.1 : ADC Sampling -> adc.c */
    // For FreeRTOS Task: ADC Sampling
    void adc_sampling(void *arg);

    /* ---> STEP 4.2 : ADC Filtering -> adc.c */
    // For FreeRTOS Task: Filtering + Detection
    void adc_filtering(void *arg);

    // =============================
    // Network Decoupling Interface (Transition Point)
    // =============================
    /**
    * @brief 
    * 
    * 
    * 
    * 
    */

    void adc_set_output_queue(QueueHandle_t queue);

#endif // ADC_H


// =============================
// Test Support API
// =============================
#ifdef UNIT_TEST

void reset_filter_state(void);
void reset_adc_state(void);

#endif
