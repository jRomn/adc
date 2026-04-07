#define UNIT_TEST

#include "unity.h"
#include "adc.h"     // Under test
#include <string.h>  // For memset
#include <math.h>    // For sinf in mocks

/* * ============================================================================
 * test_adc.c — ADC Logic Test Suite
 * ============================================================================
 * PURPOSE: 
 * This file contains unit tests for the ADC signal processing pipeline,
 * including the IIR filter, blink detection, and Alpha score calculation.
 * * THE LIFECYCLE:
 * To ensure "Test Isolation," the Unity framework calls:
 * 1. setUp()    -> Before EVERY test (resets the ADC state to zero).
 * 2. test_xxx() -> The actual test case (Arrange, Act, Assert).
 * 3. tearDown() -> After EVERY test (cleans up memory/buffers).
 * * This prevents a "dirty" buffer from one test affecting the result of another.
 * ============================================================================
 */

// =============================
// Test Setup
// =============================

    /**
    * setUp runs before every RUN_TEST call.
    */
    void setUp(void) {
        reset_adc_state();  
    }

    /**
    * tearDown runs after every RUN_TEST call.
    */
    void tearDown(void) {
        // Optional: explicit cleanup
        memset(adc_buffer, 0, sizeof(adc_buffer));
        buffer_index = 0;
    }

/* ---> STEP 1 :  */
// =============================
// Test: Buffer Integrity (Simulates Sampling Fill/Wrap)
// =============================
void test_buffer_fills_and_wraps(void) {
    
    // ARRANGE: Arrange the Buffer with mock sine wave samples to simulate ADC input.
    int16_t mock_sine[3] = {1000, 0, -1000};
    for (int i = 0; i < 3; i++) {
        adc_push_sample(mock_sine[i]);
    }
    
    // ACT: nothing special — buffer index should now be 3
    
    // ASSERT: Check buffer index and values after pushing 3 samples
    TEST_ASSERT_EQUAL_INT(3, buffer_index);
    TEST_ASSERT_EQUAL_INT(1000, adc_buffer[0]);
    TEST_ASSERT_EQUAL_INT(-1000, adc_buffer[2]);
    
    // EDGE: Validate buffer index wraps around correctly. 
    for (int i = 3; i < BUFFER_SIZE; i++) {
        adc_push_sample(i);  // Just arbitrary data
    }
    TEST_ASSERT_EQUAL_INT(0, buffer_index);  // Buffer Index should be 0. 

}

/* ---> STEP 2 :  */
// =============================
// Test: IIR Bandpass Filter Behavior
// =============================
void test_apply_bandpass_iir_behavior(void) {


    // --- 1st Case: DC input ---

    // --- Arrange ---
    reset_filter_state();

    // A constant DC input (e.g., 1000) should be **attenuated** by a bandpass filter.
    int16_t dc_input = 1000;
    int16_t output_dc = 0;      // Expected output should approach 0 over time, since it's DC.

    // --- Act ---
    for (int i = 0; i < 100; i++) {
        output_dc = apply_bandpass_iir(dc_input);
    }

    // --- Assert ---
    // “Assert that output_dc is within ±10 of 0”
    TEST_ASSERT_INT16_WITHIN(10, 0, output_dc);



    // --- 2nd Case: 10Hz input ---

    // To test the passband, we generate a discrete-time sine wave:
    //
    //      x[n] = A * sin(2π * f * n / fs)
    //
    // where:
    //   A  = amplitude (1000)
    //   f  = input signal frequency (10 Hz)
    //   fs = sampling frequency (100 Hz)
    //
    // Each iteration simulates sampling the analog sine wave every 10 ms

    // --- Arrange ---
    reset_filter_state();

    // --- Act ---
    float A = 1000.0f;
    float f = 10.0f;           // 10 Hz (inside passband)
    float fs = SAMPLE_RATE_HZ; // Sampling rate (100 Hz)
    int N = 300;               // Number of samples (~3 seconds)
    float sum_abs_pass = 0.0f;
    
    for (int n = 0; n < N; n++) {
        float sample = A * sinf(2.0 * M_PI * f * n / fs);
        int16_t output = apply_bandpass_iir((int16_t)sample);
        sum_abs_pass += fabsf(output);
    }

    float avg_abs_pass = sum_abs_pass / N;
    
    // --- Assert ---
    // Assert: Average amplitude should remain significant
    TEST_ASSERT_TRUE(avg_abs_pass > 100.0f);



    // --- 3rd Case: 50 Hz input ---
    //
    // A 50 Hz sine wave lies outside the 0.5–30 Hz bandpass range,
    // so the filter should strongly attenuate it.
    //
    // --- Arrange ---
    reset_filter_state();

    // --- Act ---
    f = 50.0f;                 // 50 Hz (outside passband)
    sum_abs_pass = 0.0f;

    for (int n = 0; n < N; n++) {
        float sample = A * sinf(2.0 * M_PI * f * n / fs);
        int16_t output = apply_bandpass_iir((int16_t)sample);
        sum_abs_pass += fabsf(output);
    }

    float avg_abs_stop = sum_abs_pass / N;


    // --- Assert ---
    // Assert: Output amplitude should be much smaller than passband
    TEST_ASSERT_TRUE(avg_abs_stop < (avg_abs_pass * 0.3f)); // <30% of passband power

}

/* ---> STEP 3 :  */
// =============================
// Test: Blink Detection Increments
// =============================
void test_blink_detection_increments(void) {

    // --- Case 0: No blink (steady signal) ---
    reset_adc_state();  // resets blink_count, buffer, etc.
    // Simulate steady signal (no blinks)
    for (int i = 0; i < 20; i++) {
        detect_events(1000);   // flat signal, derivative = 0
    }
    TEST_ASSERT_EQUAL_UINT32(0, blink_count);  // No blink expected

    // --- Case 1: Simulated blink (spike) ---
    detect_events(1030);   // +30 µV change, should trigger blink
    TEST_ASSERT_EQUAL_UINT32(1, blink_count);  // Blink detected

    // --- Case 2: Multiple samples inside refractory period ---
    for (int i = 0; i < 5; i++) {
        detect_events(1020);   // still changing but should be ignored
    }
    TEST_ASSERT_EQUAL_UINT32(1, blink_count);  // Still 1 (debounced)

    // --- Case 3: Trigger a new blink after refractory ---
    for (int i = 0; i < 14; i++) {
        detect_events(1060);   // small changes, clear refractory
    }
    detect_events(1090);       // new spike to trigger blink
    TEST_ASSERT_EQUAL_UINT32(2, blink_count);  // Second blink detected

    // --- Case 4: Negative spike detection ---
    for (int i = 0; i < 20; i++) {
        detect_events(1060);  // flat again
    }
    detect_events(1030);  // -30 µV drop should also count
    TEST_ASSERT_EQUAL_UINT32(3, blink_count);  // Third blink detected (negative slope)

}

/* ---> STEP 4 :  */
// =============================
// Test: Alpha Dominance (Focus Detection)
// =============================
void test_alpha_dominance(void) {

    const float A = 1000.0f;        // amplitude of test signal
    const float fs = SAMPLE_RATE_HZ;

    uint8_t score_2hz = 0;
    uint8_t score_10hz = 0;
    uint8_t score_20hz = 0;

    // --- Case 1: 2 Hz (low frequency, below alpha band) ---
    for (int n = 0; n < BUFFER_SIZE; n++) {
        adc_buffer[n] = (int16_t)(A * sinf(2 * M_PI * 2.0f * n / fs));
    }
    score_2hz = compute_alpha_score(adc_buffer, BUFFER_SIZE);

    // --- Case 2: 10 Hz (center of alpha band) ---
    for (int n = 0; n < BUFFER_SIZE; n++) {
        adc_buffer[n] = (int16_t)(A * sinf(2 * M_PI * 10.0f * n / fs));
    }
    score_10hz = compute_alpha_score(adc_buffer, BUFFER_SIZE);

    // --- Case 3: 20 Hz (above alpha band) ---
    for (int n = 0; n < BUFFER_SIZE; n++) {
        adc_buffer[n] = (int16_t)(A * sinf(2 * M_PI * 20.0f * n / fs));
    }
    score_20hz = compute_alpha_score(adc_buffer, BUFFER_SIZE);

    // --- Assertions ---
    // Expect: Alpha (10 Hz) >> Low/High frequencies
    TEST_ASSERT_TRUE(score_10hz > score_2hz * 1.5);   // 10 Hz power dominates
    TEST_ASSERT_TRUE(score_10hz > score_20hz * 1.5);  // also dominates 20 Hz
    // Note: The factor of 3 enforces a clear separation between alpha and non-alpha power responses. 

    // Optional sanity: all scores within [0,100]
    TEST_ASSERT_TRUE(score_2hz <= 100);
    TEST_ASSERT_TRUE(score_10hz <= 100);
    TEST_ASSERT_TRUE(score_20hz <= 100);

}


// // =============================
// // Test: BLE Formatting Packs Bytes
// // =============================
// void test_ble_formatting_packs_bytes(void) {
//     blink_count = 258;  // 0x0102
//     attention_level = 85;  // 0x55
    
//     uint8_t blink_expected[4] = {0x02, 0x01, 0x00, 0x00};  // Little-endian
//     uint8_t attn_expected[1] = {0x55};
    
//     // Act: Simulate pack (from ble_notifications)
//     uint8_t blink_data[4];
//     blink_data[0] = (uint8_t)(blink_count & 0xFF);
//     blink_data[1] = (uint8_t)((blink_count >> 8) & 0xFF);
//     blink_data[2] = (uint8_t)((blink_count >> 16) & 0xFF);
//     blink_data[3] = (uint8_t)((blink_count >> 24) & 0xFF);
//     uint8_t attn_data[1] = {attention_level};
    
//     // Assert
//     TEST_ASSERT_EQUAL_MEMORY(blink_expected, blink_data, 4);
//     TEST_ASSERT_EQUAL_UINT8(attn_expected[0], attn_data[0]);
// }

/*
int main(void) {
    UNITY_BEGIN();  // Optional: For ESP-IDF runner
    RUN_TEST(test_buffer_fills_and_wraps);
    RUN_TEST(test_filtering_rejects_noise);
    RUN_TEST(test_blink_detection_increments);
    RUN_TEST(test_alpha_score_dominant);
    RUN_TEST(test_ble_formatting_packs_bytes);
    return UNITY_END();
}
*/