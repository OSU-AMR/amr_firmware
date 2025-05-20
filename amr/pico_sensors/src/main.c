#include "driver/mfrc522.h"
#include "driver/quad_encoder.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "titan/version.h"

#include <math.h>
#include <stdio.h>

#define MISO_PIN 16
#define MOSI_PIN 19
#define SCK_PIN 18
#define CSN_PIN 17

#define HALL_PIN 0

// const uint LED_PIN = 25;
#define LED_PIN 5

static void hall_get(uint gpio, uint32_t events) {
    gpio_put(LED_PIN, !(events & 0x08));
}

int main() {
    watchdog_enable(5000, true);
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    // gpio_put(LED_PIN, 1);

    printf("%s\n", FULL_BUILD_TAG);

    // Hall
    // gpio_set_irq_enabled_with_callback(HALL_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &hall_get);
    gpio_init(HALL_PIN);

    // RFID
    // uint8_t tag1[] = { 0x04, 0x88, 0x66, 0x1A, 0x58, 0x17, 0x91 };

    // MFRC522Ptr_t mfrc = MFRC522_Init(CSN_PIN);
    // PCD_Init(mfrc, spi0, MISO_PIN, MOSI_PIN, SCK_PIN, CSN_PIN);
    // PCD_Init(mfrc, spi0);

    while (true) {
        watchdog_update();

        // printf("Waiting for card\n\r");
        // while (!PICC_IsNewCardPresent(mfrc)) {
        //     watchdog_update();
        // }
        // // Select the card
        // printf("Selecting card\n\r");
        // PICC_ReadCardSerial(mfrc);

        // // Show UID on serial monitor
        // printf("PICC dump: \n\r");
        // PICC_DumpToSerial(mfrc, &(mfrc->uid));

        // // Authorization with uid
        // printf("Uid is: ");
        // for (int i = 0; i < 4; i++) {
        //     printf("%x ", mfrc->uid.uidByte[i]);
        // }
        // printf("\n\r");

        // if (memcmp(mfrc->uid.uidByte, tag1, 4) == 0) {
        //     printf("Authentication Success\n\r");
        // }
        // else {
        //     printf("Authentication Failed\n\r");
        // }

        // printf("%u\n", gpio_get(HALL_PIN));

        gpio_put(LED_PIN, !gpio_get(HALL_PIN));

        sleep_ms(50);
    }
    return 0;
}
