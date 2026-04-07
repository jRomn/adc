# EEG Headband Firmware Setup — ADC Module Subsystem 

*Development Environment*

This project was developed and tested using:

- IDE: Visual Studio Code

- C/C++ IntelliSense & Linting: clangd

- SDK / Framework: ESP-IDF (Espressif IoT Development Framework)

- Target Platform: ESP32

> | Note: Clangd integration ensures accurate code completion, error highlighting, and faster navigation while working with ESP-IDF projects in VS Code.


## Overview: What This ADC Module Does

The ADC subsystem ( Analog-to-Digital Converter ) is responsible for translating the tiny analog EEG voltages coming from the electrodes into digital samples that the ESP32 can process in software.

EEG signals are extremely weak — typically in the 10–100 µV range — and cannot be directly read by the ESP32.

To make them measurable, an analog front-end amplifier (e.g. AD8232) is use  to boosts these signals into a 0–3.3 V range, which the ADC can then safely sample. 

Here we show how to set up the ADC subsystem for the Data Acquisition Task —once configured, the ADC subsystem becomes the foundation of continuously sampling EEG voltages and placing them into a buffer for real-time processing.

The setup flow can be summarized as :  

> | → ADC Unit Initialization → Channel Configuration → Calibration


**STEP 1 : ADC Unit Initialization** 

*Declare what is called a Handles :*

Handles are like references ( essentially a "pointer” ) that will later point to software objects ( blueprints ) of the ADC hardware and/or calibration engine inside ESP-IDF.

Think of these handles as “remote controls” that allow firmware to communicate with the ADC hardware through an abstraction layer.
```c	
extern adc_oneshot_unit_handle_t adc_handle;  // ADC driver handle
extern adc_cali_handle_t adc_cali_handle;  // ADC Calibration handle
```	
	
This handle is Global, meaning it can be used for all future ADC calls.

> | Note :  At this point, adc_handle is just a NULL pointer.


*Define the ADC Unit configuration structure :*

Here we describe which ADC hardware block (ADC1 or ADC2) we want to use and any global settings associated with it.

```c
adc_oneshot_unit_init_cfg_t init_config = {
    .unit_id = ADC_UNIT,   // Use ADC1 block
};
```

> | Note :  This structure doesn’t actually configure hardware yet — it’s just a blueprint that describes our intended setup.

*Initialize the ADC Unit using ESP-IDF API :*

```c
ret = adc_oneshot_new_unit(&init_config, &adc_handle);
```

By “initialize” we mean : 

- Allocates memory for the ADC driver object we have just created.

- Programs ADC hardware registers according to init_config

- And most importantly, updates adc_handle to point to this driver object. 

- At this point we have taken our blueprint and turned it into an operational ADC driver object. 


**STEP 2 : Channel Configuration**

Once we have initialized our ADC Unit, we must define which specific physical pin (channel) we’re going to sample ( read ) from, and under what electrical scaling conditions. Those are referred to as “_resolution_” and “_attenuation_”. 

*Define the channel configuration structure :*

```c
adc_oneshot_chan_cfg_t chan_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,  // Default 12-bit resolution
    .atten = ADC_ATTEN_DB_12           // ~3.3V full-scale voltage range
};
```



*Link this configuration to our ADC handle :*

```c
ret = adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_config)
```

At this point, any call from ESP-IDF to the ADC handle will inform that the ADC Unit instance is configured to read from GPIO34, using 12-bit precision and full 3.3 V input range.


**STEP 3 : ADC Calibration Initialization** 

The ESP32’s ADC hardware is not perfectly linear — meaning the relationship between input voltage and digital output is not exact.

To improve accuracy to actual voltages in millivolts ( mV ) , request ESP-IDF to create a calibration object to perform this correction automatically each time we read a raw ADC value.

*Define the calibration configuration structure :*

Use the same parameters used for our channel. 

```c
// This structure describes how the calibration should be performed.
// - unit_id: Which ADC unit (must match the one used before)
// - atten: Must match the attenuation used in channel config
// - bitwidth: Must match the bitwidth used in channel config
adc_cali_line_fitting_config_t cali_cfg = {
    .unit_id = ADC_UNIT,           // Same ADC unit as before
    .atten = ADC_ATTEN_DB_12,      // Same attenuation as channel config
    .bitwidth = ADC_BITWIDTH_DEFAULT  // Same bitwidth as channel config
};
```

  
*Link this configuration to our Calibration handle :* 

```c
if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) 
{
	ESP_LOGI(ADC_TAG, "ADC calibration ready.");
} else {
	ESP_LOGW(ADC_TAG, "ADC calibration not available. Using raw ADC values.");
	adc_cali_handle = NULL;          // Use raw values if calibration fails
}
```

From now on, any future readings will be converted to millivolts for improved accuracy.


**STEP 4 System Ready → FreeRTOS Acquisition Task**

Up to this point the ADC subsystem is now fully initialized, calibrated and ready to start sampling. 

To finalize this ADC Module set up, we will involve creating a FreeRTOS Acquisition Task that periodically :

*Read raw ADC value:* 

	```c
	adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
	```

*Convert raw to calibrated voltage (mV):*  

	```c
	adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage);
	```

*Store calibrated voltage in circular buffer:*

	```c
	adc_buffer[buffer_index] = (int16_t)(voltage * 10);
	buffer_index = (buffer_index + 1) % BUFFER_SIZE; // Wrap around
	```

This task will run at a fixed interval (e.g., every 10 ms → 100 Hz sampling), feeding the signal processing layer with continuous, real-time EEG data.

	“_The ADC reads one sample at a rate of 100 Hz, meaning every 10 ms_.”





**STEP 5 FreeRTOS Filtering ( & Detection ) Task**

After the ADC task fills the circular buffer with digitized EEG samples, the next step is to clean the signal ( _filtering_ ) and identify the frequencies of interest ( _detection_ ). 

Those can be accomplished by a dedicated FreeRTOS Filtering ( & Detection ) Task, which periodically performs the following three main operations for continuous real-time signal processing :

*Read latest ADC sample from the circular buffer*

```c
size_t latest_idx = (buffer_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
int16_t current_sample = adc_buffer[latest_idx];
```

This retrieves the most recent ADC sample stored by the ADC acquisition task, using modular arithmetic to safely wrap around when the buffer index reaches the end.

*Apply a digital IIR bandpass filter*

```c
int16_t filtered = apply_bandpass_iir(current_sample);
```
This line applies a 2nd-order IIR Band-pass filter to isolate 0.5–30 Hz band frequencies.


*Pass the Cleaned Value to the Next Processing Stage ( Detection )*

Once the signal is cleaned, it’s passed to the event detection and alpha-score routines to identify physiological events such as blinks or dominant alpha-band activity.

```c
detect_events(filtered);
```

This task “xTaskCreate(adc_filtering)” function runs periodically ( same as ADC acquisition task, every 10 ms → 100 Hz sampling ) but remains with a lower priority ( 4 ) than the ADC acquisition task,  to prevent any risk of timing interference with the sampling process.


----------------------------------------------------------------------------------------------------






## System Diagram — ADC Subsystem Data Flow

```
        ┌────────────────────────┐
        │  Analog EEG Signal     │
        │ (0–3.3 V after AFE)    │
        └──────────┬─────────────┘
                   │
                   ▼
        ┌────────────────────────┐
        │ ADC Unit (ADC1)        │
        │ - adc_oneshot_read()   │
        │ - 12-bit @ 12 dB atten │
        └──────────┬─────────────┘
                   │ Raw Counts (0–4095)
                   ▼
        ┌────────────────────────┐
        │ ADC Calibration         │
        │ - Line Fitting Scheme   │
        │ - Converts to mV        │
        └──────────┬─────────────┘
                   │ Calibrated mV
                   ▼
        ┌────────────────────────┐
        │ FreeRTOS Acquisition   │
        │ Task (100 Hz)          │
        │ - Periodic sampling     │
        │ - Stores to buffer      │
        └──────────┬─────────────┘
                   │
                   ▼
        ┌────────────────────────┐
        │ Circular Buffer         │
        │ - Rolling data storage  │
        └──────────┬─────────────┘
                   │
                   ▼
        ┌────────────────────────┐
        │ Signal Processing       │
        │ Module (Filtering, FFT) │
        └────────────────────────┘
```

----------------------------------------------------------------------------------------------------


## Folder Structure

For “modular implementation” here is the expected general folder structure: 
	
	eeg/          — Root project directory
	├── .vscode/            — VS Code configs (for debugging bliss)
	├── components/
	│   └── adc/            — Our ADC module (reusable, testable)
	│       ├── include/
	│       │   └── adc.h   — Declarations, configs, globals
	│       ├── adc.c       — Implementations (init, future tasks/filters)
	│       ├── CMakeLists.txt — Builds the component
	│       └── test/       — Unit tests (mock ADC for filter validation)
	│           ├── CMakeLists.txt
	│           └── test_adc.c
	├── main/
	│   ├── main.c          — App entry (init everything, create tasks)
	│   └── CMakeLists.txt  — Main component build
	├── test/               — Full-system tests
	│   ├── CMakeLists.txt
	│   ├── sdkconfig.defaults
	│   └── main/
	│       ├── CMakeLists.txt
	│       └── test_main.c
	├── CMakeLists.txt      — Top-level project (boilerplate magic)
	├── pytest_unittest.py  — Test runner (optional)
	└── README.md           — This doc (or expand it!)
	

> | Note :  We recommend to follow this structure since it would be easy to add unit tests in the future. 

The most important parts right now are the main.c, adc.h and adc.c.
