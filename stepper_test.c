#include <pico/stdlib.h>
#include <stdio.h>

int main()
{
    const uint LED_PIN = 7;

    // Initialize the GPIO to blink the LED
    //
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    // Initialize the serial port.
    //
    stdio_init_all();

    // Loop to blink the LED and print the message.
    //
    while (true)
    {
        printf("Wax on!\n");
        gpio_put(LED_PIN, 1);
        sleep_ms(100);

        printf("Wax off!\n");
        gpio_put(LED_PIN, 0);
        sleep_ms(100);
    }

    return 0;
}