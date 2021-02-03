/* Flash multiple partitions example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include "esp_loader.h"
#include "example_common.h"
#include "raspberry_port.h"

#define TARGET_RST_Pin 2
#define TARGET_IO0_Pin 3

#define DEFAULT_BAUD_RATE 115200
#define HIGHER_BAUD_RATE  460800


//#define SERIAL_DEVICE     "/dev/ttyUSB0"
#define SERIAL_DEVICE     "\\\\.\\COM88"

#define BINARY_PATH       "./Firmware.bin"

#define BOOTLOADER_ADDRESS  0x1000
#define PARTITION_ADDRESS   0x8000
#define APPLICATION_ADDRESS 0x10000

static int get_buffer(const char* path,char** buffer)
{
    FILE *image = fopen(path, "rb");
    if (image == NULL) {
        printf("Error:Failed to open file %s\n", path);
        return;
    }

    fseek(image, 0L, SEEK_END);
    size_t size = ftell(image);
    rewind(image);
    fseek(image, 0, SEEK_SET);
    printf("File %s opened. Size: %u bytes\n", path, size);

    *buffer = (char *)malloc(size);
    if (*buffer == NULL) {
        printf("Error: Failed allocate memory\n");
        return -1;
    }
    uint32_t bytes_read = fread(*buffer, 1, size, image);
    return bytes_read;
}

static void upload_file(loader_raspberry_config_t* config)
{
    char *buffer = NULL;
    int size = 0;
#if 0
    FILE *image = fopen(config->boot_path, "rb");
    if (image == NULL) {
        printf("Error:Failed to open file %s\n", path);
        return;
    }

    fseek(image, 0L, SEEK_END);
    size_t size = ftell(image);
    rewind(image);
    fseek(image, 0, SEEK_SET);
    printf("File %s opened. Size: %u bytes\n", path, size);

    buffer = (char *)malloc(size);
    if (buffer == NULL) {
        printf("Error: Failed allocate memory\n");
        goto cleanup;
    }

    // copy file content to buffer
    //size_t bytes_read = fread(buffer, 1, size, image);
    uint32_t bytes_read = fread(buffer, 1, size, image);
#endif
    size = get_buffer(config->boot_path, &buffer);
    flash_binary(buffer, size, config->bootloader_address);
    free(buffer);

    size = get_buffer(config->app_path, &buffer);
    flash_binary(buffer, size, config->application_address);
    free(buffer);

    size = get_buffer(config->part_path, &buffer);
    flash_binary(buffer, size, config->partition_address);
    free(buffer);

/*
    flash_binary(buffer, size, config->partition_address);
    flash_binary(buffer, size, config->application_address);
*/
cleanup:
    free(buffer);
}
#define TP_UART_FILENAME_PREFIX     "\\\\.\\"
int main(int argc,int* argv[])
{
    char com_set[50];
    if(argc > 1)
    {
        strcpy(com_set, TP_UART_FILENAME_PREFIX);
        strcpy(&com_set[strlen(TP_UART_FILENAME_PREFIX)],argv[1]);
        printf("com = %s\r\n",com_set);
    }
    else
    {
        strcpy(com_set,SERIAL_DEVICE);
    }
    loader_raspberry_config_t config = {
        .device = SERIAL_DEVICE,
        .baudrate = DEFAULT_BAUD_RATE,
        .reset_trigger_pin = TARGET_RST_Pin,
        .gpio0_trigger_pin = TARGET_IO0_Pin,
        .partition_address = PARTITION_ADDRESS,
        .bootloader_address = BOOTLOADER_ADDRESS,
        .application_address = APPLICATION_ADDRESS,
        .app_path = "./app.bin",
        .boot_path = "./boot.bin",
        .part_path = "./part.bin"
    };
    config.device = com_set;
    loader_port_raspberry_init(&config);

    if (connect_to_target(HIGHER_BAUD_RATE) == ESP_LOADER_SUCCESS) {
        upload_file(&config);
    }

    loader_port_reset_target();
}
