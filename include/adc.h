#ifndef ADC_H
#define ADC_H

/**
 * @file adc.h
 * @brief ADC Module Header File
 */

// =============================
// Header Files 
// =============================

    /* --- General --- */
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"

    /* --- Decoupling Queue --- */
    #include "freertos/queue.h"

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

    /* ADC Initialization */
    /** 
    * @brief Function to initialize the ADC unit and start sampling.
    * @return ESP_OK on success.
    * @return ESP_FAIL if the ADC could not be initialized.
    */
    adc_oneshot_unit_handle_t adc_init_start(void);


    /* FreeRTOS Task: Sampling */
    /** 
    * @brief Function to continuously sample raw EEG data from the ADC 
    * at a fixed rate and push it into a FreeRTOS queue for processing 
    * by the filtering task.
    * @return None
    */
    void adc_sampling(void *arg);


    /* ADC-to-WebSocket Decoupling Interface (Transition Point) */
    /** 
    * @brief Function to pass the FreeRTOS queue handle from main.c to adc.c, 
    * enabling the ADC module to push eeg_frame_t data to the WS Server module through this 
    * shared queue without a direct dependency on the WS Server implementation.
    * @return None
    */
    void adc_set_in_queue(QueueHandle_t queue);


    /* FreeRTOS Task: Filtering */
    /** 
    * @brief Function to continuously filter the sampled EEG data and 
    * detect cognitive states. 
    * @return None
    */
    void adc_filtering(void *arg);



#endif // ADC_H


// =============================
// Test Support API
// =============================
#ifdef UNIT_TEST

void reset_filter_state(void);
void reset_adc_state(void);

#endif
