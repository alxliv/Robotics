#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

// For Pi PICO 2W

// USED in L298N H-Bridge for MOTOR rotation direction and Start/Stop
const int IN3_GPIO = 2;          // pin 4
const int IN4_GPIO = 3;          // pin 5
const int GPIO_IRQ_B = 10;       // Encoder of Motor B (phase A) will fire interrupt on GPIO 10 (physical pin 14)
const int GPIO_M_B_PHASE_B = 11; // Encoder of Motor B (phase B) (physical pin 15)

// Interrupt callback function
volatile int _posB = 0;
static void gpio_irq_callback(uint gpio, uint32_t events)
{
    //    printf("Interrupt on GPIO %d! Events: 0x%08x\n", gpio, events);
    if (gpio != GPIO_IRQ_B)
    {
        printf("WHAT?? IRQ gpio=%d\n", gpio);
        return;
    }
    int phaseB = gpio_get(GPIO_M_B_PHASE_B);
    if (phaseB)
        _posB++;
    else
        _posB--;
    // printf("IRQ %d, phaseB=%d\n", gpio, phaseB);
}

int main()
{
    static int nloop = 0;
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init())
    {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Example to turn on the Pico W LED
    bool led_state = true;

    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    // Select ADC input 0 (GPIO26)
    adc_select_input(0);

    printf("ALEX!!! Started adc on pin %d\n", 26);

    gpio_init(IN3_GPIO);
    gpio_set_dir(IN3_GPIO, GPIO_OUT);
    gpio_init(IN4_GPIO);
    gpio_set_dir(IN4_GPIO, GPIO_OUT);

    // turn on motor B:
    gpio_put(IN3_GPIO, 1);
    gpio_put(IN4_GPIO, 0);

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(0, GPIO_FUNC_PWM); // pin 1 on pico
    gpio_set_function(1, GPIO_FUNC_PWM); // pin 2 on pico

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(0);

    // Set period of 10000 cycles (0 to 9999 inclusive)
    pwm_set_wrap(slice_num, 9999);
    pwm_set_gpio_level(0, 0); // 0% duty cycle

    pwm_set_enabled(slice_num, true);

    // Initialize GPIO 10 as input
    gpio_init(GPIO_M_B_PHASE_B); // pin 15
    gpio_set_dir(GPIO_M_B_PHASE_B, GPIO_IN);
    gpio_pull_down(GPIO_M_B_PHASE_B); // Enable pull-down resistor

    gpio_init(GPIO_IRQ_B); // pin 14
    gpio_set_dir(GPIO_IRQ_B, GPIO_IN);
    gpio_pull_down(GPIO_IRQ_B); // Enable pull-down resistor
    // Enable interrupt on rising edge for GPIO 10
    gpio_set_irq_enabled_with_callback(10, GPIO_IRQ_EDGE_RISE, true, &gpio_irq_callback);

    sleep_ms(1000);
    printf("START!!!\n");
    uint32_t t_start0 = time_us_32();
    uint old_duty = 0;
    int posB = 0;

    while (true)
    {
        nloop++;
        if (posB != _posB)
        {
           uint32_t irq_status = save_and_disable_interrupts();
           posB = _posB;
           restore_interrupts(irq_status);
           printf("posB: %d\n", posB);
        }

        uint32_t t_curr0 = time_us_32();
        if (t_curr0 - t_start0 > 300 * 1000)
        {
            t_start0 = t_curr0;
            // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
            const float conversion_factor = 3.3f / (1 << 12);
            uint16_t result = adc_read();
            //   printf(">Raw:%u, Voltage:%f\n", result, result * conversion_factor);

            uint duty_cycle = (result * 10000) / (1 << 12);
            if (duty_cycle != old_duty)
            {

                pwm_set_gpio_level(0, duty_cycle); // 50% duty cycle
                old_duty = duty_cycle;
            }

            // printf("nloop=%d, duty=%d\n", nloop, duty_cycle);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            led_state = !led_state;
        }
    }
}
