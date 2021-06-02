
#include <tm4c123gh6pm.h>
#include "stdint.h"
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#define Red 0x02
#define Blue 0x04
#define Green 0x08
#define Clock 16000000

#define PI 3.141592654
#define RequiredDistance 100
#define PortF_switches 0x11
#define Delay 2000 //ms

// functions proto type
int read_current_coordinates(double *longitude, double *latitude);
void seven_segments_display(uint32_t number, uint32_t digit);
void double_to_three_digits(double number, uint32_t *d1, uint32_t *d2, uint32_t *d3);
bool is_final_destination(double distance);
double distance_between_points(double longitude1, double latitude1, double longitude2, double latitude2);
void light_up(uint32_t led);
void light_down(uint32_t led);
void lights_off(void);
void systick_wait_free_ms(uint32_t delay);
void switch_led(uint32_t led);
char get_next_char(void);
void serial_send(char data);
void print_it(char s[], int length);
bool is_gps_ready(void);
double angel_to_decimal(double angel);

void portF_init();
void portE_init();
void portD_init();
void portA_init();
void systick_init();
void initialize_ports();

void SystemInit(void) {}

void initialize_ports()
{
    portF_init();
    portE_init();
    portD_init();
    portA_init();
    systick_init();
}

int main()
{
    double distance = 0;
    uint32_t digit1 = 0;
    uint32_t digit2 = 0;
    uint32_t digit3 = 0;

    initialize_ports();

    // prints 0 to all seven segments
    seven_segments_display(0, 1);
    seven_segments_display(0, 10);
    seven_segments_display(0, 100);

    // idle case
    while ((GPIO_PORTF_DATA_R & PortF_switches) != 0x10) //push switch 2 to start calculating
    {
        light_up(Blue);
    }
    light_down(Blue);

    // calculating distance
    while (1)
    {
        distance += distance_between_points(0, 0, 0, 0); // this fn returns 1 as dummy distance
        double_to_three_digits(distance, &digit1, &digit2, &digit3);

        // printing
        seven_segments_display(digit3, 1);
        seven_segments_display(digit2, 10);
        seven_segments_display(digit1, 100);

        switch_led(Green);
        if (is_final_destination(distance))
        {
            lights_off();
            light_up(Red);
            return 0;
        }
        systick_wait_free_ms(10);
    }
}

//ziad
// LEDs
void portF_init()
{
    SYSCTL_RCGCGPIO_R |= 0x20; //port f
    while ((SYSCTL_PRGPIO_R & 0x20) == 0)
    {
    } //wait for portf to be activated
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = 0x1F;
    GPIO_PORTF_AFSEL_R = 0;
    GPIO_PORTF_PCTL_R = 0;
    GPIO_PORTF_AMSEL_R = 0;
    GPIO_PORTF_DEN_R = 0x1F;
    GPIO_PORTF_DIR_R = 0x0E;
    GPIO_PORTF_PUR_R = 0x11;
}