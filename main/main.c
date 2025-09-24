#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "soc/spi_struct.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "esp_task_wdt.h"

// Configuration parameters
#define PWM_FREQUENCY_HZ    10000   // 10 kHz
#define PWM_GPIO_A          21       // First PWM output
#define PWM_GPIO_B          12       // Second PWM output
#define PWM_DUTY_CYCLE      50       // 50% duty cycle for both signals
#define PWM_RESOLUTION_HZ   80000000 // 80 MHz resolution for high precision

static const char *TAG = "Analog-LIA Controller";
static float current_phase_degrees = 0.0;
static int32_t current_phase_ticks = 0;

// fast digital write
#define GPIO_SET_HIGH(pin)   (GPIO.out_w1ts = (1 << pin))
#define GPIO_SET_LOW(pin)    (GPIO.out_w1tc = (1 << pin))

// SPI pins
#define PIN_NUM_CLK   18
#define PIN_NUM_MOSI  23  
#define PIN_NUM_MISO  19
#define PIN_NUM_CS     5

// buffer size
#define SAMPLES_PER_BATCH 32  // Number of ADC samples to collect before processing
#define FILTER_LENGTH 8  // Size of moving average length

// Number of outputs
#define NUM_OUTPUTS 5

// 12 bit mask for 32-bit integers
#define MASK_A 0xFFFFF000
#define MASK_B 0x00000FFF

// prototype functions
void init_buffers(void);
void init_spi_adc(BaseType_t coreID);
void spi_sampling_task(void *param);
void app_main(void);

// Global buffers
static int32_t ring_buffer[NUM_OUTPUTS][FILTER_LENGTH]; // moving average
static int32_t filter_out[NUM_OUTPUTS];
static int32_t pesos[FILTER_LENGTH];

// Function prototypes
esp_err_t set_phase_difference_degrees(float phase_degrees);
esp_err_t set_phase_difference_ticks(int32_t ticks);
void keyboard_monitor_task(void *pvParameters);
esp_err_t mcpwm_initialize(void);

// Initalization of ADC and filter buffers
void init_buffers(void){
  for(size_t o=0; o<NUM_OUTPUTS; o++){
    for(size_t i=0; i<FILTER_LENGTH; i++)
      ring_buffer[o][i] = 0;
    filter_out[o] = 0;
  }
}

void init_spi_adc(BaseType_t coreID){

  // Initialize SPI bus
  spi_bus_config_t bus_cfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };
  spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_DISABLED);
  
  // Configure CS pin manually
  gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_NUM_CS, 0);

  // Clear all SPI3 data buffers
  for (int i = 0; i < 16; i++) {
    SPI3.data_buf[i] = 0;
  }
  
  // Clock base 80MHz
  // Actual clk speed: 80MHz / clk_div;
  int clk_div = 8; // 8 for 10 MHz, 10 for 8 MHz
  SPI3.clock.clk_equ_sysclk = 0;
  SPI3.clock.clkdiv_pre = 0;
  SPI3.clock.clkcnt_n = clk_div-1; // n+1 = 10, so 80MHz/10 = 8MHz
  SPI3.clock.clkcnt_h = clk_div/2 - 1; // (n+1)/2 - 1 for ~50% duty cycle
  SPI3.clock.clkcnt_l = clk_div-1;

  // *** SPI MODE 1 CONFIGURATION (CPOL=0, CPHA=1) ***
  SPI3.pin.ck_idle_edge = 0;    // CPOL=0: Clock idles LOW
  SPI3.user.ck_out_edge = 0;    // CPHA=1: Master changes data on rising edge,
                                //         samples MISO on falling edge
                                // Note: ck_out_edge controls MASTER timing, not just MOSI
  
  // No command at the begining
  SPI3.user.usr_command = 0; // otherwise is sends an initial 8-bit sequence (the SPI command)
  
  // Set as 16-bit transfers
  uint32_t bitLength = 16;
  SPI3.mosi_dlen.usr_mosi_dbitlen = bitLength - 1; // mosi length
  SPI3.miso_dlen.usr_miso_dbitlen = bitLength - 1; // miso length
  
  // Full-duplex or half-duplex (1:enable 0:disable)
  SPI3.user.usr_miso = 1; // miso
  SPI3.user.usr_mosi = 1; // mosi
  SPI3.user.doutdin = 1;  // full duplex
  
  // Create SPI sampling task pinned to CoreID
  xTaskCreatePinnedToCore(
        spi_sampling_task,      // Task function
        "spi_sampling",         // Task name
        4096,                   // Stack size (bytes)
        NULL,//tx_buffer,              // Parameter (TX buffer)
        tskIDLE_PRIORITY + 2,   // High priority (use tskIDLE_PRIORITY instead of configMAX_PRIORITIES)
        NULL,                   // Task handle (optional)
        coreID                  // Core ID (0 = Core0, 1 = Core1)
        );
}

// SPI sampling task
// Should run on Core 1 (dedicated to SPI sampling)
// Note: Input parameter is not used
void spi_sampling_task(void *param){
  size_t index = 0;
  while (1) {
    for(int o = 0; o < NUM_OUTPUTS; o++){
      // === ADC ACCUMULATION PHASE ===
      int32_t acc = 0; // accumulator
      for (int i = 0; i < SAMPLES_PER_BATCH; i++) {
        
        // Fast CS pulse (~60 ns)
        GPIO_SET_HIGH(PIN_NUM_CS);
        GPIO_SET_LOW(PIN_NUM_CS);
        
        // Load TX data (for monitoring)
        SPI3.data_buf[0] = 0xAAAA;
        
        // Start SPI transaction and wait for completion
        SPI3.cmd.usr = 1;
        while (SPI3.cmd.usr);
        
        // Get RX data and extract value
        uint32_t word = SPI3.data_buf[0];
        word = word >> 2; // Select the right number of bits to drop here!
        
        // sign extension
        if( (word >> 11)&1 ) word |= MASK_A;
        else word &= MASK_B;
        
        // accumulate
        acc += (int32_t)word;
      }
      
      // === FILTERING PHASE ===
      filter_out[o] = 0;
      for(int i=0; i<FILTER_LENGTH; i++){
	filter_out[o] += ring_buffer[o][i]*pesos[i];
      }
      //filter_out[o] -= ring_buffer[o][index];
      //filter_out[o] += acc;
      ring_buffer[o][index] = acc;
     }
    index = (index+1) % FILTER_LENGTH;
  }
}

// Task to monitor if a key is pressed
void keyboard_monitor_task(void *pvParameters)
{
  static const char *KeyboardTAG = "Phase updater";
  int c;
  float desired_phase_degrees;
  int32_t desired_phase_ticks;
  while (1) {
    c = getchar();
    if (c != EOF && c != 0xFF) { // Check for a valid character
      ESP_LOGI(KeyboardTAG, "Key pressed: '%c' (ASCII: %d)", c, c);
      
      // React to the character
      switch (c) {
      case 'w':
	desired_phase_ticks = current_phase_ticks + 1;
	ESP_LOGI(KeyboardTAG, "Increasing phase (fine)...");
	set_phase_difference_ticks(desired_phase_ticks);
	ESP_LOGI(KeyboardTAG, "Current phase: %.3f degrees (%ld ticks)",
		 current_phase_degrees, current_phase_ticks);
	break;
      case 's':
	desired_phase_ticks = current_phase_ticks - 1;
	ESP_LOGI(KeyboardTAG, "Decreasing phase (fine)...");
	set_phase_difference_ticks(desired_phase_ticks);
	ESP_LOGI(KeyboardTAG, "Current phase: %.3f degrees (%ld ticks)",
		 current_phase_degrees, current_phase_ticks);
	break;
      case 'd':
      case 'D':
	desired_phase_degrees = current_phase_degrees + 90.0;
	ESP_LOGI(KeyboardTAG, "+90 degree phase...");
	set_phase_difference_degrees(desired_phase_degrees);
	ESP_LOGI(KeyboardTAG, "Current phase: %.3f degrees (%ld ticks)",
		 current_phase_degrees, current_phase_ticks);
        break;
      case 'a':
      case 'A':
	desired_phase_degrees = current_phase_degrees - 90.0;
	ESP_LOGI(KeyboardTAG, "-90 degree phase...");
	set_phase_difference_degrees(desired_phase_degrees);
	ESP_LOGI(KeyboardTAG, "Current phase: %.3f degrees (%ld ticks)",
		 current_phase_degrees, current_phase_ticks);
        break;
      case 'W':
	desired_phase_ticks = current_phase_ticks + 100;
	ESP_LOGI(KeyboardTAG, "Increasing phase (coarse)...");
	set_phase_difference_ticks(desired_phase_ticks);
	ESP_LOGI(KeyboardTAG, "Current phase: %.3f degrees (%ld ticks)",
		 current_phase_degrees, current_phase_ticks);
	break;
      case 'S':
	desired_phase_ticks = current_phase_ticks - 100;
	ESP_LOGI(KeyboardTAG, "Decreasing phase (coarse)...");
	set_phase_difference_ticks(desired_phase_ticks);
	ESP_LOGI(KeyboardTAG, "Current phase: %.3f degrees (%ld ticks)",
		 current_phase_degrees, current_phase_ticks);
	break;
	/*case 'q':
	ESP_LOGI(TAG, "Exiting monitor task...");
	// Delete the task if needed
	vTaskDelete(NULL);
	break;*/
      default:
	ESP_LOGI(KeyboardTAG, "Unknown command.");
	break;
      }
    }
    // Add a small delay to yield CPU time to other tasks
    vTaskDelay(pdMS_TO_TICKS(10)); // Adjust delay as needed
  }
}

// MCPWM handles - using separate timers for true independence
mcpwm_timer_handle_t timer_a_handle = NULL;
mcpwm_timer_handle_t timer_b_handle = NULL;
mcpwm_oper_handle_t operator_a_handle = NULL;
mcpwm_oper_handle_t operator_b_handle = NULL;
mcpwm_cmpr_handle_t comparator_a_handle = NULL;
mcpwm_cmpr_handle_t comparator_b_handle = NULL;
mcpwm_gen_handle_t generator_a_handle = NULL;
mcpwm_gen_handle_t generator_b_handle = NULL;
mcpwm_sync_handle_t sync_handle = NULL;   // Global sync handle (created once)

// Timer period for phase calculations
uint32_t timer_period_ticks = 0;

esp_err_t mcpwm_initialize(void)
{
  esp_err_t ret = ESP_OK;
  
  // Calculate the period in ticks
  timer_period_ticks = PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ;
  
  // Create two independent MCPWM timers
  mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = PWM_RESOLUTION_HZ,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    .period_ticks = timer_period_ticks,
  };
  
  // Create timer A
  ret = mcpwm_new_timer(&timer_config, &timer_a_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM timer A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Create timer B (independent)
  ret = mcpwm_new_timer(&timer_config, &timer_b_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create MCPWM timer B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ESP_LOGI(TAG, "Timers created - Frequency: %d Hz, Period: %lu ticks", 
	   PWM_FREQUENCY_HZ, timer_period_ticks);
  
  // Create MCPWM operators
  mcpwm_operator_config_t operator_config = {
    .group_id = 0,
  };
  
  ret = mcpwm_new_operator(&operator_config, &operator_a_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create operator A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ret = mcpwm_new_operator(&operator_config, &operator_b_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create operator B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Connect each operator to its own timer
  ret = mcpwm_operator_connect_timer(operator_a_handle, timer_a_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect timer A to operator A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ret = mcpwm_operator_connect_timer(operator_b_handle, timer_b_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to connect timer B to operator B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Create comparators
  mcpwm_comparator_config_t comparator_config = {
    .flags.update_cmp_on_tez = true,
  };
  
  ret = mcpwm_new_comparator(operator_a_handle, &comparator_config, &comparator_a_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create comparator A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ret = mcpwm_new_comparator(operator_b_handle, &comparator_config, &comparator_b_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create comparator B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Create generators
  mcpwm_generator_config_t generator_config = {
    .gen_gpio_num = PWM_GPIO_A,
  };
  
  ret = mcpwm_new_generator(operator_a_handle, &generator_config, &generator_a_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create generator A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  generator_config.gen_gpio_num = PWM_GPIO_B;
  ret = mcpwm_new_generator(operator_b_handle, &generator_config, &generator_b_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create generator B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Set duty cycle for both comparators (exactly 50%)
  uint32_t duty_ticks = timer_period_ticks / 2;  // 50% duty cycle
  
  ret = mcpwm_comparator_set_compare_value(comparator_a_handle, duty_ticks);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set comparator A value: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ret = mcpwm_comparator_set_compare_value(comparator_b_handle, duty_ticks);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set comparator B value: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ESP_LOGI(TAG, "Duty cycle set to %lu ticks (exactly 50%%)", duty_ticks);
  
  // Configure PWM A actions (reference signal)
  ret = mcpwm_generator_set_action_on_timer_event(generator_a_handle,
						  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
  if (ret != ESP_OK) return ret;
  
  ret = mcpwm_generator_set_action_on_compare_event(generator_a_handle,
						    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_a_handle, MCPWM_GEN_ACTION_LOW));
  if (ret != ESP_OK) return ret;
  
  // Configure PWM B actions (same pattern as A)
  ret = mcpwm_generator_set_action_on_timer_event(generator_b_handle,
						  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
  if (ret != ESP_OK) return ret;
  
  ret = mcpwm_generator_set_action_on_compare_event(generator_b_handle,
						    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_b_handle, MCPWM_GEN_ACTION_LOW));
  if (ret != ESP_OK) return ret;
  
  // Enable both timers
  ret = mcpwm_timer_enable(timer_a_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ret = mcpwm_timer_enable(timer_b_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable timer B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Create sync source ONCE here: use timer A EMPTY event
  mcpwm_timer_sync_src_config_t sync_config = {
    .timer_event = MCPWM_TIMER_EVENT_EMPTY,
    .flags.propagate_input_sync = false,
  };
  ret = mcpwm_new_timer_sync_src(timer_a_handle, &sync_config, &sync_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create sync source: %s", esp_err_to_name(ret));
    return ret;
  }
  
  // Start both timers
  ret = mcpwm_timer_start_stop(timer_a_handle, MCPWM_TIMER_START_NO_STOP);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer A: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ret = mcpwm_timer_start_stop(timer_b_handle, MCPWM_TIMER_START_NO_STOP);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start timer B: %s", esp_err_to_name(ret));
    return ret;
  }
  
  ESP_LOGI(TAG, "MCPWM initialized successfully - Both independent timers running");
  return ESP_OK;
}

esp_err_t set_phase_difference_ticks(int32_t ticks)
{
  // Ensure phase is within range
  while (ticks < 0) ticks += timer_period_ticks;
  while (ticks >= timer_period_ticks) ticks -= timer_period_ticks;
  
  // Calculate phase offset in timer ticks
  uint32_t phase_offset_ticks = (uint32_t)ticks;
  uint32_t count_at_sync = phase_offset_ticks;
  
  // Program the phase update to take effect on the NEXT A EMPTY event.
  // This is jitter-free and supported in ESP-IDF v5.2 (no soft-trigger API).
  mcpwm_timer_sync_phase_config_t phase_config = {
    .sync_src = sync_handle,   // reuse global sync handle
    .count_value = count_at_sync,
    .direction = MCPWM_TIMER_DIRECTION_UP,
  };
  
  esp_err_t ret = mcpwm_timer_set_phase_on_sync(timer_b_handle, &phase_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set phase on timer B: %s", esp_err_to_name(ret));
    return ret;
  }

  // Update was posible, so update the current phase
  current_phase_ticks = ticks;

  // Update phase in degrees as well
  current_phase_degrees = (current_phase_ticks * 360.0) / timer_period_ticks;

  ESP_LOGI(TAG, "New phase update accepted (%0.3f degrees, %ld ticks).", current_phase_degrees, current_phase_ticks);
  
  return ESP_OK;
}

esp_err_t set_phase_difference_degrees(float phase_degrees)
{
  // Ensure phase is within 0-360 range
  while (phase_degrees < 0) phase_degrees += 360.0;
  while (phase_degrees >= 360.0) phase_degrees -= 360.0;
  
  // Calculate phase offset in timer ticks
  uint32_t phase_offset_ticks = (uint32_t)((phase_degrees / 360.0) * timer_period_ticks);
  
  uint32_t count_at_sync = phase_offset_ticks;
  
  // Program the phase update to take effect on the NEXT A EMPTY event.
  // This is jitter-free and supported in ESP-IDF v5.2 (no soft-trigger API).
  mcpwm_timer_sync_phase_config_t phase_config = {
    .sync_src = sync_handle,   // reuse global sync handle
    .count_value = count_at_sync,
    .direction = MCPWM_TIMER_DIRECTION_UP,
  };
  
  esp_err_t ret = mcpwm_timer_set_phase_on_sync(timer_b_handle, &phase_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set phase on timer B: %s", esp_err_to_name(ret));
    return ret;
  }

  // Update was posible, so update the current phase
  current_phase_degrees = phase_degrees;

  // Update phase in degrees as well
  current_phase_ticks = phase_offset_ticks;

  ESP_LOGI(TAG, "New phase update accepted (%0.3f degrees, %ld ticks).", current_phase_degrees, current_phase_ticks);
  
  return ESP_OK;
}

void print_filtered_task(void *param){
    while(1){
        for(int o = 0; o < NUM_OUTPUTS; o++)
            printf("%ld ", filter_out[o]);
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera 1 segundo
    }
}

void app_main(void)
{
    init_buffers();                   // Inicializa buffers del filtro y ADC
    init_spi_adc(1);                  // Sampling SPI en Core 1
    esp_err_t ret = mcpwm_initialize();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MCPWM: %s", esp_err_to_name(ret));
        return;
    }
    set_phase_difference_degrees(0);  // Fase inicial en 0 (se puede ajustar después)
    xTaskCreate(keyboard_monitor_task, "Keyboard Monitor Task", 2048, NULL, 5, NULL); // Tarea teclado en Core 0
    xTaskCreate(print_filtered_task, "Print Filtered Task", 2048, NULL, 4, NULL);     // Nueva tarea impresión

    // Bucle vacío principal: las tareas manejan todo por separado
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10000)); // Espera larga para evitar que termine
    }
}
