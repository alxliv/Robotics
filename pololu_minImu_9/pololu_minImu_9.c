#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define GYRO_ACCEL (0x6B)
#define MAGNETO_METER (0x1E)

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#define LED_DELAY_MS (600)

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void i2c_scan(i2c_inst_t *i2c)
{
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 0; addr <= 127; addr++)
    {
        uint8_t dummy = 0;
        if (!reserved_addr(addr))
        {
            int ret = i2c_read_blocking(i2c, addr, &dummy, 1, false);
            if (ret >= 0)
            {
                char *devname="UNKNOWN";
                if (addr == GYRO_ACCEL)
                    devname="GYRO_ACCEL";
                else if (addr == MAGNETO_METER)
                    devname = "MAGNETO_METER";
                printf("I2C Device (%s) found at 0x%02X\n", devname, addr);
            }
        }
    }
    printf("Scan complete.\n");
}

int main() {

    stdio_init_all();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);


    // Initialize I2C0 at 400 kHz on the default pins (GP4 = SDA, GP5 = SCL)
    i2c_init(i2c_default, 400 * 1000);

    // Set up I2C pins (example for Pico)
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C); // SDA GP4 which is physical pin 6
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C); // SCL GP5 - physical pin 7
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    sleep_ms(5000);
    i2c_scan(i2c_default);
    printf("ALX!! minIMU-9 started OK\n");
    int nloop = 0;
    while (true) {
        nloop++;
        printf("[%d]\n", nloop);
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}
