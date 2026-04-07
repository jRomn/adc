// =============================
// Header Files 
// =============================

    /* --- General --- */
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_log.h"
    #include "esp_err.h"

    /* --- ADC --- */
    #include "adc.h"
    #include <math.h>  // For Goertzel (sin/cos)

    /* --- Decoupling Queue --- */
    #include "freertos/semphr.h"
    #include <string.h>

// =============================
// Private (File-scope) Globals 
// =============================

    adc_oneshot_unit_handle_t adc_handle =  NULL;   // ADC Driver Handle
    adc_cali_handle_t adc_cali_handle =     NULL;   // ADC Calibration Driver Handle

    static SemaphoreHandle_t adc_mutex =    NULL;   // Mutex to protect ADC buffer access (File-scope)

    static int16_t adc_buffer[BUFFER_SIZE];                // Circular Buffer for ADC samples // <--- STEP 2.2
    static volatile size_t buffer_index = 0;               // Index for Circular Buffer (shared between tasks)
    
    static volatile uint32_t blink_count = 0;              // Blink counter (volatile for cross-task access)
    volatile cognitive_state_t cognitive_state = COGNITIVE_STATE_UNKNOWN;

    /* Adaptive thresholds (will be calculated at runtime) */
    volatile int16_t blink_rise_threshold = 0; // The minimum absolute amplitude the filtered signal must reach to consider “a blink has started”
    volatile int16_t blink_fall_threshold = 0;// The lower amplitude the signal must drop below to consider “the blink has finished”.

    static QueueHandle_t external_queue =   NULL;   // The "Bridge"

// =============================
// Blink Adaptive Calibration Constants
// =============================
    #define CALIBRATION_SAMPLES   1500    /* 1500 = 3 seconds at 500 Hz */
    #define RISE_MARGIN_FACTOR    1.4f   /* threshold = resting_ceiling * 1.4    */
    #define FALL_FACTOR           0.4f   /* fall = rise * 0.4                    */
    #define REFRACTORY_SAMPLES     250    /* 500 ms lockout at 500 Hz            */

// =============================
// IIR Bandpass Globals (2nd-order Butterworth, 0.5-30Hz @500Hz sample)
// =============================
    float bp_a[3] = {1.0f, -1.68218424f, 0.68420169f};   // Denominator (a0=1, a1, a2)
    float bp_b[3] = {0.15789915f, 0.0f, -0.15789915f};   // Numerator (b0, b1, b2)
    float bp_x[2] = {0};  // Input history
    float bp_y[2] = {0};  // Output history

// =============================
// Main Functions:
// =============================

    /* ---> STEP 1.1 : Initialize ADC */

    // =============================
    // ADC Unit Initialization + Channel Configuration + Calibration
    // =============================
    // Function to initialize the ADC unit, configure the channel, and set up calibration.
    // Returns a handle to the initialized ADC unit.
    adc_oneshot_unit_handle_t adc_init(void){

        esp_err_t ret;  // Error Status Code Type to stores the status returned by the Driver API Initialization Call.

        // ==============================
        // 1. ADC Unit Initializing (Identify ADC Block + Create Driver Handle)
        // ==============================

        /* STEP 1A : Declare the ADC Unit Handle ( already define at the top of this file)    */
        
        /* STEP 1B : ADC Unit Configuration Instance */
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT,   // Use ADC1 block
        };
    
        /* STEP 1C : ADC Unit Initialization */
        ret = adc_oneshot_new_unit(&init_config, &adc_handle);
        if (ret == ESP_OK) {
            ESP_LOGI(ADC_TAG, "ADC Unit initialized successfully!");
        } else {
            ESP_LOGE(ADC_TAG, "Failed to initialize ADC unit! Error code: %d", ret);
            return NULL; // Stop if initialization failed
        }
        


        // ==============================
        // 2. ADC Unit Configuration (Identify Channel + Set Parameters)
        // ==============================

        /* STEP 2A : Identify the ADC Channel (pin) to Configure
        *
        *      ADC Unit  : ADC1
        *      ADC Input : Channel 6 (GPIO34)
        *
        */

        /* STEP 2B : Define the conversion parameters: resolution and attenuation
        *
        *  .bitwidth : 12-bit resolution
        *      Resolution of the ADC conversion.
        *
        *  .atten : ADC_ATTEN_DB_12   →  ~3.3V full-scale range
        *
        */

        /* STEP 2C: Configuration Instance Initialization */
        adc_oneshot_chan_cfg_t chan_config = {
            .bitwidth = ADC_BITWIDTH_DEFAULT,  // Default 12-bit resolution
            .atten = ADC_ATTEN_DB_12           // ~3.3V full-scale voltage range
        };

        /* STEP 2D: Hardware Initialization call */
        ret = adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_config);
        if (ret == ESP_OK) {
            ESP_LOGI(ADC_TAG, "ADC channel configured successfully!");
        } else {
            ESP_LOGE(ADC_TAG, "Failed to configure ADC channel! Error code: %d", ret);
            return NULL;
        }

        // ==============================
        // 3. ADC Calibration Setup
        // ==============================

        /* STEP 3A : Declare the Calibration Handle ( already define at the top of this file)    */

        /* STEP 3B : Define the Calibration Configuration Instance */
        adc_cali_line_fitting_config_t cali_cfg = {
            .unit_id = ADC_UNIT,               // Must match the ADC unit used earlier
            .atten = ADC_ATTEN_DB_12,          // Must match channel attenuation
            .bitwidth = ADC_BITWIDTH_DEFAULT   // Must match channel resolution
        };

        /* STEP 3C : Initialize the Calibration Driver and Handle */
        if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) {
            ESP_LOGI(ADC_TAG, "ADC calibration ready.");
        } else {
            ESP_LOGW(ADC_TAG, "ADC calibration not available. Using raw ADC values.");
            adc_cali_handle = NULL;          // Use raw values if calibration fails
        }

        if(adc_mutex == NULL){
            adc_mutex = xSemaphoreCreateMutex();
            if (adc_mutex == NULL) {
                ESP_LOGE(ADC_TAG, "Failed to create ADC mutex!");
                return NULL;
            }
        }
        
        // ==============================
        // --- End of setup ---
        // ==============================
        ESP_LOGI(ADC_TAG, "ADC is now initialized and ready for sampling.");

        // Return the ADC driver handle in case the caller wants it
        return adc_handle;

    }


    /* ---> STEP 3.1 : ADC Sampling */

    // =============================
    // Function for FreeRTOS Task: ADC Sampling
    // =============================
    void adc_sampling(void *arg){

        ESP_LOGI(ADC_TAG, "ADC sampling task started!");

        while (1) {

            int raw = 0;
            int voltage = 0; // Calibrated voltage in mV

            // --- 1. Read raw ADC value and hold the result at &raw ---
            adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw); // ESP-IDF API

            // --- 2. Convert raw to calibrated voltage (mV) ---
            if (adc_cali_handle) {
                adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage); // ESP-IDF API
            } else {
                // Fallback if calibration unavailable
                voltage = raw;
            }

            // --- 3. Store calibrated voltage in circular buffer ---
            // Note: 1 unit = 0.1 mV scaling for EEG µV interpretation (e.g., 200 threshold = 20µV actual)
            xSemaphoreTake(adc_mutex, portMAX_DELAY);
            adc_buffer[buffer_index] = (int16_t)(voltage * 10);
            buffer_index = (buffer_index + 1) % BUFFER_SIZE; // Wrap around
            xSemaphoreGive(adc_mutex);

            // --- 5. Sampling Period - define when the next sample happens: Scheduled Wakeup (2 ms) ---
            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_PERIOD_MS));

        }
    }


    /* ---> STEP 3.2 : ADC Filtering -> adc.c */

    /* ---> STEP 3.2. :  */
    // =============================
    // 
    // =============================
    /* Function to receive the queue handle from main.c */
    void adc_set_output_queue(QueueHandle_t queue) {
        external_queue = queue;
    }

    /* ---> STEP 3.2.5 :  */
    // =============================
    // Simple Goertzel for Alpha Power (8-12 Hz; For Focus)
    // =============================
    static float goertzel_power(const int16_t *window, size_t len, float target_freq) {

        float coeff = 2.0f * cosf(2.0f * M_PI * target_freq / SAMPLE_RATE_HZ);
        float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;

        for (size_t i = 0; i < len; i++) {
            q0 = coeff * q1 - q2 + (float)window[i];
            q2 = q1;
            q1 = q0;
        }

        // Return power — sqrt omitted for speed (relative comparison only)
        return q1 * q1 + q2 * q2 - q1 * q2 * coeff;
    }

    /* ---> STEP 3.2.4 :  */
    // =============================
    // Cognitive State Detection — Dominant Band Classification
    // =============================
    // Runs four Goertzel passes, one per EEG band, and returns the band
    // with the highest power as the current dominant cognitive state.
    cognitive_state_t detect_cognitive_state(const int16_t *window, size_t len) {

        float power_delta = goertzel_power(window, len,  2.0f);
        float power_theta = goertzel_power(window, len,  6.0f);
        float power_alpha = goertzel_power(window, len, 10.0f);
        float power_beta  = goertzel_power(window, len, 20.0f);

        // Find the dominant band by comparing all four powers
        float max_power        = power_delta;
        cognitive_state_t state = COGNITIVE_STATE_DELTA;

        if (power_theta > max_power) { max_power = power_theta; state = COGNITIVE_STATE_THETA; }
        if (power_alpha > max_power) { max_power = power_alpha; state = COGNITIVE_STATE_ALPHA; }
        if (power_beta  > max_power) { max_power = power_beta;  state = COGNITIVE_STATE_BETA;  }

        return state;
    }

    /* ---> STEP 3.2.3 :  */
    // =============================
    // Event Detection (Blinks & Focus)
    // =============================
    void detect_events(int16_t filtered_current) {  // Changed: Param for filtered
        
        static int16_t  prev_sample = 0;
        static uint8_t  refractory  = 0;
        static int16_t  peak_value  = 0;
        static bool     in_blink    = false;

        int16_t derivative = filtered_current - prev_sample;

        /* --- Blink detection state machine --- */
        if (refractory > 0) {
            refractory--;
        } else if (!in_blink) {
            if (abs(filtered_current) > blink_rise_threshold &&
                abs(derivative) > 250) {
                in_blink   = true;
                peak_value = filtered_current;
            }
        } else {
            if (abs(filtered_current) > abs(peak_value)) {
                peak_value = filtered_current;
            }
            if (abs(filtered_current) < blink_fall_threshold) {
                blink_count++;
                ESP_LOGI(ADC_TAG, "Blink detected! Count: %lu | Peak: %d (%.1f mV)",
                        blink_count, peak_value, peak_value / 10.0f);
                in_blink   = false;
                peak_value = 0;
                refractory = REFRACTORY_SAMPLES;
            }
        }

        prev_sample = filtered_current;

        /* --- Cognitive state classification — always runs, every 50 samples --- */
        static size_t sample_counter = 0;
        if (++sample_counter >= 50) {
            cognitive_state = detect_cognitive_state(adc_buffer, BUFFER_SIZE);
            const char *state_label[] = { "UNKNOWN", "DELTA", "THETA", "ALPHA", "BETA" };
            ESP_LOGI(ADC_TAG, "Cognitive state: %s", state_label[cognitive_state]);
            sample_counter = 0;
        }

    }

    /* ---> STEP 3.2.2 :  */
    // =============================
    // IIR Bandpass Filter ( 0.5-30 Hz )
    // =============================
    int16_t apply_bandpass_iir(int16_t input) {
        
        float x = (float)input;
        
        float y = bp_b[0] * x + bp_b[1] * bp_x[0] + bp_b[2] * bp_x[1] -
                bp_a[1] * bp_y[0] - bp_a[2] * bp_y[1];
        
        bp_x[1] = bp_x[0]; bp_x[0] = x;
        bp_y[1] = bp_y[0]; bp_y[0] = y;
        
        return (int16_t)y;

    }

    /* ---> STEP 3.2.4 :  */
    // Simple Baseline Calibration Window (first 3 seconds after connection) (call once after filtering task starts)
    void calibrate_blink_thresholds(void){

        ESP_LOGI(ADC_TAG, "Blink calibration starting — remain still for 3 seconds...");

        int16_t cal_peak_abs = 0;

        for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {

            /* Read the latest sample from the circular buffer */
            xSemaphoreTake(adc_mutex, portMAX_DELAY);
            size_t idx = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
            int16_t raw = adc_buffer[idx];
            xSemaphoreGive(adc_mutex);

            /* Filter it through the same IIR pipeline as the main loop */
            int16_t filtered = apply_bandpass_iir(raw);

            /* Track peak absolute amplitude */
            int16_t abs_val = (int16_t)abs(filtered);
            if (abs_val > cal_peak_abs) cal_peak_abs = abs_val;

            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_PERIOD_MS));

        }

        /* Compute thresholds from measured resting ceiling */
        blink_rise_threshold = (int16_t)(cal_peak_abs * RISE_MARGIN_FACTOR);
        blink_fall_threshold  = (int16_t)(blink_rise_threshold * FALL_FACTOR);

        ESP_LOGI(ADC_TAG,
            "Blink calibration complete. "
            "Resting ceiling: %d | RISE: %d | FALL: %d",
            cal_peak_abs, blink_rise_threshold, blink_fall_threshold);

    }


    /* ---> STEP 3.2.1 :  */
    // =============================
    // FreeRTOS Task: Filtering + Detection 
    // =============================
    void adc_filtering(void *arg) {

        ESP_LOGI(ADC_TAG, "ADC filtering task started!");

        /* ================================================
        * CALIBRATION: Runs ONLY ONCE at task startup
        * ================================================ */
        calibrate_blink_thresholds();

        /* ================================================
        * Main 500 Hz filtering loop
        * ================================================ */
        
        while (1) {

            // --- 1. Take the "key" and read the latest sample from the  Buffer.
            xSemaphoreTake(adc_mutex, portMAX_DELAY);
            size_t latest_idx = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
            int16_t current_sample = adc_buffer[latest_idx];
            xSemaphoreGive(adc_mutex);

            // --- 2. Passes the buffer sample through a digital Bandpass Filter
            int16_t filtered = apply_bandpass_iir(current_sample);  // Compute once

            // --- 3. Passes the filtered sample to the Event Detection Engine for blinks and focus level detection. 
            detect_events(filtered);  // Pass to avoid double filter

            // --- 4. === NETWORK TRANSITION POINT === 
            // Package all ADC-derived state into a Single Frame and push to Queue ---
            if (external_queue != NULL) {
                eeg_frame_t frame = {
                    .filtered_value  = filtered,
                    .blink_count     = blink_count,
                    .cognitive_state = cognitive_state,
                };

                // We use a 0 timeout because 500Hz is fast; if the queue is full, 
                // it's better to drop a frame than to lag the filter.
                xQueueSend(external_queue, &frame, 0); 

            }

            // --- 5. Filtering Sample Rate: 2 ms ( 500 Hz - 500 samples per second) ---
            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_PERIOD_MS));

        }
    }


// =============================
// Unit Test Helper Functions (Exposed for test_adc.c):
// =============================

    // =============================
    // Helper: Push New Sample into ADC Buffer
    // =============================
    void adc_push_sample(int16_t sample) {
        adc_buffer[buffer_index] = sample;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;
    }


    // =============================
    // Test Helper Fn: Internal State Reset
    // =============================
    void reset_filter_state(void) {
        bp_x[0] = bp_x[1] = 0.0f;
        bp_y[0] = bp_y[1] = 0.0f;
    }

    void reset_adc_state(void) {
        memset(adc_buffer, 0, sizeof(adc_buffer));
        buffer_index = 0;
        blink_count = 0;
        cognitive_state = COGNITIVE_STATE_UNKNOWN;
        reset_filter_state();
    }




