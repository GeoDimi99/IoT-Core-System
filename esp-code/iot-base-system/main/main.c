#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "esp_timer.h"
#include <rom/ets_sys.h>
#include "esp_system.h"
#include "nvs_flash.h"

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"

#include "esp_tls.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mqtt_client.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <math.h>
#include "esp_dsp.h"



/* Communication Configuration */
#define SSID "Redmi Note 12 5G"
#define PASS "da0a0tt0"
#define MQTT_URI "mqtts://test.mosquitto.org:8883"
#define MQTT_SAMPLE_TOPIC "sample_1890039"

/* Sampling Configuration */
#define N_SAMPLES 4096   
#define MAX_NUMBER_OF_SAMPLES 25000 * 3						// Samples = 25000 Hz * 3s
#define MAX_SAMPLE_FREQUENCY 25000.0f						// Maximum frequency 25000 Hz
#define SAMPLING_AVG_DELAY_us 40							// Sampling Delay in microseconds
#define ZSCORE_THRESHOLD 3.5f

/* Stream Buffer Configuration */
#define LENGHT_OF_BUFFER  sizeof(float)
#define TRIGGER_LEVEL     sizeof(float)

/* Audio ADC Configuration */
#define AUDIO_CHANNEL ADC_CHANNEL_1
#define AUDIO_ATTEN ADC_ATTEN_DB_12

/* Certificate TLS/SSL */ 
extern const uint8_t mqtt_eclipseprojects_io_pem_start[]   asm("_binary_mqtt_eclipseprojects_io_pem_start");
extern const uint8_t mqtt_eclipseprojects_io_pem_end[]   asm("_binary_mqtt_eclipseprojects_io_pem_end");
	
/* Debugging Tags */ 
static const char *ESP_TAG = "ESP32s3";
static const char *WIFI_TAG = "WIFI";
static const char *MQTT_TAG = "MQTT";
static const char *FTT_TAG = "FTT";
static const char *AUDIO_TAG = "AUDIO";
static const char *BUFFER_TAG = "STREAM BUFFER"; 



int N = N_SAMPLES;

/* ADC input variables */
static int adc_raw;
static int voltage;

/* Stream Buffer */
float data;
StreamBufferHandle_t buffer_handler = NULL;

/* Signal array */
__attribute__((aligned(16)))
float x1[N_SAMPLES];
__attribute__((aligned(16)))
float x2[N_SAMPLES];


/* Noise array */
__attribute__((aligned(16)))
float noise[N_SAMPLES];

/* Signal sum array : original signal + noise */
__attribute__((aligned(16)))
float signal_tot[N_SAMPLES];


/* Window coefficients */
__attribute__((aligned(16)))								
float wind[N_SAMPLES];

/* Working complex array */
__attribute__((aligned(16)))								
float y_cf[N_SAMPLES* 2];							

/* Pointer to result arrays */
float *y1_cf = &y_cf[0];										 
float *y2_cf = &y_cf[N_SAMPLES];


/* ADC calibration function */
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(AUDIO_TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(AUDIO_TAG , "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(AUDIO_TAG , "Invalid arg or no memory");
    }

    return calibrated;
}

static void adc_calibration_deinit(adc_cali_handle_t handle)
{
    ESP_LOGI(AUDIO_TAG, "Deregister calibration scheme");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
} 

/* Handler functions */

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(MQTT_TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
		case MQTT_EVENT_CONNECTED:
			ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
			msg_id = esp_mqtt_client_subscribe(client, MQTT_SAMPLE_TOPIC , 0);
			msg_id = esp_mqtt_client_publish(client, MQTT_SAMPLE_TOPIC , "ESP32s3(1890039) CONNECTED", 0, 1, 0);
			break;
		case MQTT_EVENT_DISCONNECTED:
			ESP_LOGW(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
			break;

		case MQTT_EVENT_SUBSCRIBED:
			ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
			msg_id = esp_mqtt_client_publish(client, MQTT_SAMPLE_TOPIC , "data", 0, 0, 0);
			ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
			break;
		case MQTT_EVENT_UNSUBSCRIBED:
			ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_PUBLISHED:
			ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
			break;
		case MQTT_EVENT_DATA:
			ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
			printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
			printf("DATA=%.*s\r\n", event->data_len, event->data);
			break;
		case MQTT_EVENT_ERROR:
			ESP_LOGE(MQTT_TAG, "MQTT_EVENT_ERROR");
			break;
		default:
			ESP_LOGI(MQTT_TAG, "Other event id:%d", event->event_id);
			break;
    }
}


static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
		ESP_LOGI(WIFI_TAG, "WIFI_START");
        break;
        
    case WIFI_EVENT_STA_CONNECTED:
		ESP_LOGI(WIFI_TAG, "WIFI_CONNECTED");
        break;
        
    case WIFI_EVENT_STA_DISCONNECTED:
		ESP_LOGW(WIFI_TAG, "WIFI_DISCONNECTED");
		/* If the device lost the connection, retry the connection */ 
		ESP_LOGW(WIFI_TAG, "WIFI_RETRY_CONNECTION");
		esp_wifi_connect();
        break;
        
    case IP_EVENT_STA_GOT_IP:
		ESP_LOGI(WIFI_TAG, "WIFI_GOT_IP");
        break;
        
    default:
        break;
    }
}

esp_mqtt_client_handle_t mqtt_app_start(void)
{
	/* MQTT Initial Configuration */
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        .broker.verification.certificate = (const char *)mqtt_eclipseprojects_io_pem_start,
    };

	/* MQTT Init */
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    
    /* MQTT Start Client */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    
    return client;
    
}


/* MQTT Communication */


void mqtt_communication_task(void *args){
	
	/*
	 * Communication Task: connection through Wi-Fi to MQTT broker and
	 * 					   comunicate the avarage sample signal 
	 */ 
	
	
	
	/* Initialize the default NVS partition */
    ESP_ERROR_CHECK(nvs_flash_init());
    
	/* Wi-Fi Connection: */ 
	
	/* Wi-Fi Init Phase */
	ESP_ERROR_CHECK(esp_netif_init());                   					// TCP/IP initiation 					
    
    ESP_ERROR_CHECK(esp_event_loop_create_default());     					// event loop 			                
    
    esp_netif_create_default_wifi_sta(); 									// WiFi station 	                    
    
    
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();		// Wifi initialization 
    esp_wifi_init(&wifi_initiation); 																				
    
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));						// Wifi Station mode (connetct to AP)  				
	  
	/* Wi-Fi Configuration Phase */
	esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS
            }
           };
            
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    
    /* Wi-Fi Start Phase */
    ESP_ERROR_CHECK(esp_wifi_start());
    
    /* Wi-Fi Connect Phase */
    esp_wifi_connect();
    
    /* MQTT connection */
    esp_mqtt_client_handle_t client = mqtt_app_start();
    
    size_t stream_buffer_result = 0;
	float signal_avr;
    while(true){		
		stream_buffer_result = xStreamBufferReceive(buffer_handler, (void *) &signal_avr, LENGHT_OF_BUFFER, 100);
		ESP_LOGI(AUDIO_TAG, "Reciver avg signal: %f", signal_avr);
		
		char * msg_avr = (char*) malloc( 50 * sizeof(char));
		sprintf(msg_avr, "AVG AUDIO: %f\n", signal_avr);
		int msg_id = esp_mqtt_client_publish(client, MQTT_SAMPLE_TOPIC , msg_avr, 0, 1, 0);
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
	
	
}


/* Audio Sampler */


float array_mean(float * buffer, size_t buffer_size){
	
	float sum = 0; 
	for(int i = 0; i < buffer_size; i++ ){
		sum += buffer[i]; 
	}
	return sum / buffer_size;
	
}

float array_sqr_mean(float * buffer, size_t buffer_size){
	
	float sqr_sum = 0;
	for(int i = 0; i < buffer_size; i++){
		sqr_sum += buffer[i]*buffer[i];
	}
	return sqr_sum/buffer_size; 
}

float array_std_dev(float * buffer, float mean, size_t buffer_size){
	
	float var = 0;
	for(int i = 0; i < buffer_size; i++){
			var += (buffer[i] - mean) * (buffer[i] - mean);
	}
	return sqrt( var / buffer_size );
	
} 





void audio_sampler_task(void *args){
	
	/*
	 * Audio Sampler Task: Sampling and Analyze Task
	 * 					     
	 */
	
	
	/* 
	 * Compute the sampling interval 
	 * 
	 * Fs - Sampling Frequency
	 * T - Delay for sleep
	 * d - Computing delay for each iteration
	 * 
	 * Fs = 1 / (T + d) 
	 * T = 1/Fs - d 
	 * 
	 */  
	

	/* Initialize FFT */
	
	
	/* Audio ADC Init */
	adc_oneshot_unit_handle_t audio_adc_handle;
    adc_oneshot_unit_init_cfg_t audio_adc_init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&audio_adc_init_config, &audio_adc_handle));
	
	/* Audio ADC Config */
	adc_oneshot_chan_cfg_t audio_adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT ,
        .atten = AUDIO_ATTEN,
    };
     ESP_ERROR_CHECK(adc_oneshot_config_channel(audio_adc_handle, AUDIO_CHANNEL, &audio_adc_config));
     
    /* Audio ADC Calibration */
    adc_cali_handle_t audio_adc_cali_handle = NULL;
    bool do_calibration_audio_adc_chan = adc_calibration_init(ADC_UNIT_1, AUDIO_CHANNEL, AUDIO_ATTEN, &audio_adc_cali_handle);
    
    
    
    
    /* Audio ADC Oversample */
    int64_t start_time_sampling;
    int64_t finish_time_sampling;
    float max_sample_frequency;
    
	ESP_LOGI(AUDIO_TAG, "AUDIO_START_SAMPLE");
	ESP_LOGI(ESP_TAG, "TIMER_START");
	start_time_sampling = esp_timer_get_time();
	for(int i = 0; i < N; i++) {
		ESP_ERROR_CHECK(adc_oneshot_read(audio_adc_handle, AUDIO_CHANNEL, &adc_raw));
		if (do_calibration_audio_adc_chan) {
			ESP_ERROR_CHECK(adc_cali_raw_to_voltage(audio_adc_cali_handle, adc_raw, &voltage));
			
			signal_tot[i] = voltage * 0.001f;
		}
		
	}
	finish_time_sampling = esp_timer_get_time();
	ESP_LOGI(AUDIO_TAG, "AUDIO_STOP_SAMPLE");
	float tot_sample_time_s = (finish_time_sampling - start_time_sampling) * 0.000001f;
	float iter_sample_time_s = ((finish_time_sampling - start_time_sampling) * 0.000001f) / N;
	max_sample_frequency = N / ((finish_time_sampling - start_time_sampling) * 0.000001f);
	ESP_LOGI(ESP_TAG, "TIMER_STOP delay: %f s, delay for iter %f s, frequency %f Hz", tot_sample_time_s, iter_sample_time_s,  max_sample_frequency) ;
	
	
	
	
	
	/* Initialize FFT */
    ESP_LOGI(FTT_TAG, "FTT_START");
    ESP_ERROR_CHECK(dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE));
    
    
    /* Generate hann window */
	dsps_wind_hann_f32(wind, N);
	
	
	
	/* Convert two input vectors to one complex vector */
	 for (int i = 0 ; i < N ; i++) {
        y_cf[i * 2 + 0] = signal_tot[i] * wind[i];		// Real part
        y_cf[i * 2 + 1] = 0;							// Imaginary part
    }
    
    /* FFT */
    unsigned int start_b = dsp_get_cpu_cycle_count();
    dsps_fft2r_fc32(y_cf, N);
    unsigned int end_b = dsp_get_cpu_cycle_count();
    
    /* Bit reverse */
    dsps_bit_rev_fc32(y_cf, N);
    
    
    /* Convert one complex vector to two complex vectors */
    dsps_cplx2reC_fc32(y_cf, N);
    
    for (int i = 0 ; i < N / 2 ; i++) {
        y1_cf[i] = 10 * log10f((y1_cf[i * 2 + 0] * y1_cf[i * 2 + 0] + y1_cf[i * 2 + 1] * y1_cf[i * 2 + 1]) / N);
        y2_cf[i] = 10 * log10f((y2_cf[i * 2 + 0] * y2_cf[i * 2 + 0] + y2_cf[i * 2 + 1] * y2_cf[i * 2 + 1]) / N);
    }
    
      ESP_LOGI(FTT_TAG, "FTT_STOP");
    
    /* Show power spectrum in 64x10 window from -100 to 0 dB from 0..N/4 samples */
    ESP_LOGI(FTT_TAG, "Signal ");
    dsps_view(y1_cf, N / 2, 64, 10,  -60, 40, '|');
    ESP_LOGI(FTT_TAG, "FFT for %i complex points take %i cycles", N, end_b - start_b);
    
    
    float mean = array_mean(y1_cf, N/2);
    float std_dev = array_std_dev(y1_cf, mean, N/2);
    
    
    /* Compute Z-Score and compare it with threshold */
    float z_score;
    float min_sample_freq = max_sample_frequency;
    for(int i = 0; i < N/2; i++){
		z_score = (y1_cf[i] - mean) / std_dev;
		
		if(z_score > ZSCORE_THRESHOLD){
				/* Nyquist Theorem */
				min_sample_freq = 2 * ((i * MAX_SAMPLE_FREQUENCY) / N);
		}
		
	}
	
	
	/* Audio ADC Adapted Sample */
	int sample_interval_us = (1 / (min_sample_freq)) * 1000000; 
	ESP_LOGI("SAMPLE", "Frequency: %f Interval: %d", min_sample_freq, sample_interval_us);
	
	while(true) {
		ESP_LOGI(AUDIO_TAG, "AUDIO_START_SAMPLE");
		ESP_LOGI(ESP_TAG, "TIMER_START");
		start_time_sampling = esp_timer_get_time();
		for(int i = 0; i < N; i++) {
			ESP_ERROR_CHECK(adc_oneshot_read(audio_adc_handle, AUDIO_CHANNEL, &adc_raw));
			if (do_calibration_audio_adc_chan) {
				ESP_ERROR_CHECK(adc_cali_raw_to_voltage(audio_adc_cali_handle, adc_raw, &voltage));
				signal_tot[i] = voltage * 0.001f;
			}
		
			ets_delay_us(sample_interval_us - SAMPLING_AVG_DELAY_us);
		 }
		finish_time_sampling = esp_timer_get_time();
		ESP_LOGI(AUDIO_TAG, "AUDIO_STOP_SAMPLE");
		float tot_time_s = (finish_time_sampling - start_time_sampling) * 0.000001f;
		float iter_time_s = ((finish_time_sampling - start_time_sampling) * 0.000001f) / N;
		float sample_frequency = N / ((finish_time_sampling - start_time_sampling) * 0.000001f); 
		ESP_LOGI(ESP_TAG, "TIMER_STOP sample_interval: %d delay: %f s, delay for iter %f s, frequency %f Hz", sample_interval_us, tot_time_s, iter_time_s * 1000000, sample_frequency);
		
		
		
		/* Compute average of the signal */
		float signal_avr = array_sqr_mean(signal_tot, N);
		ESP_LOGI(AUDIO_TAG, "Trasmitter avg signal: %f", signal_avr);
		
		/* Send the avarage of the signal to the mqtt task */
		size_t stream_buffer_result = 0;
		stream_buffer_result = xStreamBufferSend(buffer_handler, (void*) &signal_avr, LENGHT_OF_BUFFER, 100);
		
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
	

}

void app_main(void)
{
	/* Info Logs */
	ESP_LOGI(ESP_TAG, "Startup..");
    ESP_LOGI(ESP_TAG, "Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(ESP_TAG, "IDF version: %s", esp_get_idf_version());
    
    
    /* Initialize StreamBuffer that allow sharing memory between task */
    buffer_handler = xStreamBufferCreate(LENGHT_OF_BUFFER, TRIGGER_LEVEL);
    if( buffer_handler != NULL){
		ESP_LOGI(BUFFER_TAG, "STREAM_BUFFER_CREATED_SUCCESFULL");
	}
	else {
		ESP_LOGE(BUFFER_TAG, "STREAM_BUFFER_NOT_CREATED");
		return;
	}
    
    
    /* Tasks */
	
    xTaskCreatePinnedToCore(mqtt_communication_task , "mqtt_communication_task", configMINIMAL_STACK_SIZE*3 , NULL, 5, NULL,0);
    xTaskCreatePinnedToCore(audio_sampler_task , "audio_sampler_task", configMINIMAL_STACK_SIZE*3 , NULL, 5, NULL, 1);
    
	
}
