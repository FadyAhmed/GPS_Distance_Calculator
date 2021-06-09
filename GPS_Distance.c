#include <tm4c123gh6pm.h>

#include "stdint.h"
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

#define Red 0x02
#define Blue 0x04
#define Green 0x08
#define Clock 16000000

#define PI 3.141592654
#define RequiredDistance 230
#define PortF_switches 0x11
#define Delay 4000 //ms
#define R 6371e3
#define Tolerance 2

// iterator for testing only
// int stringLength = 0;
// array and index to save coordinates to the end of each trip
int coordinatesIndex = 0;
// saves 1000 read each has 23 charachter
char final_coordinates[1000][23];

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
int parse_coor(double *longitude, double *latitude);

void portF_init(void);
void portE_init(void);
void portD_init(void);
void portA_init(void);
void systick_init(void);
void UART5_Init(void);
void UART1_Init(void);

void initialize_ports(void);


void SystemInit(void) {}

void initialize_ports()
{
    portF_init();
    portE_init();
    portD_init();
    portA_init();
    systick_init();
    UART5_Init();
    UART1_Init();
}

int main()
{
    double distance = 0;
    uint32_t digit1 = 0;
    uint32_t digit2 = 0;
    uint32_t digit3 = 0;
    double longitude1 = 0;
    double latitude1 = 0;
    double longitude2 = 0;
    double latitude2 = 0;
    char s[16] = {0};

    initialize_ports();

    // prints 0 to all seven segments
    seven_segments_display(0, 1);
    seven_segments_display(0, 10);
    seven_segments_display(0, 100);

    // wait gps to reach a fix
    while (!is_gps_ready())
    {
        light_up(Blue);
        systick_wait_free_ms(50);
        light_down(Blue);
        systick_wait_free_ms(50);
    }

    // idle case (gps is ready)
    while ((GPIO_PORTF_DATA_R & PortF_switches) != 0x10) //push switch 2 to start calculating
    {
        light_up(Blue);
    }
    light_down(Blue);

    read_current_coordinates(&longitude1, &latitude1);

		sprintf(s, "%f", distance);
		print_it(s, 6);
		serial_send('\n');
    // calculating distance
    while (1)
    {
        int print_coor_index = 0;
        read_current_coordinates(&longitude2, &latitude2);
        distance += distance_between_points(0, 0, 0, 0); // this fn returns 1 as dummy distance
        longitude1 = longitude2;
        latitude1 = latitude2;

        sprintf(s, "%f", distance);
        print_it(s, 6);
        serial_send('\n');

        double_to_three_digits(distance, &digit1, &digit2, &digit3);
        // printing
        seven_segments_display(digit3, 1);
        seven_segments_display(digit2, 10);
        seven_segments_display(digit1, 100);

        switch_led(Green);
               // when distance reached
        if (is_final_destination(distance))
        {
            light_down(Green);
            light_up(Red);
            //send data to UART5
            // wait user to push switch 1
					while(1){
            while ((GPIO_PORTF_DATA_R & PortF_switches) != 0x01)
            {
            }
            for (print_coor_index = 0; print_coor_index < coordinatesIndex; print_coor_index++)
            {
                print_it(final_coordinates[print_coor_index], 23);
								print_it(s, 6);
								serial_send('\n');
            }}
            break;
        }
        systick_wait_free_ms(Delay);
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

// ones seven segments
void portE_init()
{
  SYSCTL_RCGCGPIO_R |= 0x10; // PORTE

  while ((SYSCTL_PRGPIO_R & 0x10) == 0)
  {
  } //wait for portE to be activated

  GPIO_PORTE_DEN_R |= 0x0F;
  GPIO_PORTE_AFSEL_R |= 0;
  GPIO_PORTE_AMSEL_R |= 0;
  GPIO_PORTE_PCTL_R |= 0;
  GPIO_PORTE_DEN_R |= 0x0F;
  GPIO_PORTE_DIR_R |= 0x0F;
}
// tens seven segments
void portD_init()
{
  SYSCTL_RCGCGPIO_R |= 0x08; //port D
  while ((SYSCTL_PRGPIO_R & 0x08) == 0)
    ; //wait for portD to be activated
  GPIO_PORTD_AFSEL_R |= 0;
  GPIO_PORTD_PCTL_R |= 0;
  GPIO_PORTD_AMSEL_R |= 0;
  GPIO_PORTD_DEN_R |= 0x0F;
  GPIO_PORTD_DIR_R |= 0x0F;
}

// hundreds seven segments
void portA_init()
{
  SYSCTL_RCGCGPIO_R |= 0x01; //portA
  while ((SYSCTL_PRGPIO_R & 0x01) == 0)
    ; //wait for portA to be activated
  GPIO_PORTA_AFSEL_R |= 0;
  GPIO_PORTA_PCTL_R |= 0;
  GPIO_PORTA_AMSEL_R |= 0;
  GPIO_PORTA_DEN_R |= 0xF0;
  GPIO_PORTA_DIR_R |= 0xF0;
}

//ziad
void light_up(uint32_t led)
{
    GPIO_PORTF_DATA_R |= led;
}

void light_down(uint32_t led)
{
    GPIO_PORTF_DATA_R &= !led;
}

void lights_off()
{
    GPIO_PORTF_DATA_R &= (!Blue && !Green && !Red);
}

void switch_led(uint32_t led)
{
    if ((GPIO_PORTF_DATA_R &= led) == led)
    {
        light_down(led);
    }
    else
    {
        light_up(led);
    }
}

// systick init
void systick_init(void)
{
    NVIC_ST_CTRL_R = 0;
    NVIC_ST_RELOAD_R = 0x00FFFFF;
    NVIC_ST_CURRENT_R = 0;
    NVIC_ST_CTRL_R = 0x5; // enable & clock source (101)
}
void systick_wait_ms(uint32_t delay)
{
    delay = delay * Clock / 100;
    NVIC_ST_RELOAD_R = delay - 1; // sub 1 because count flag raise after reaching zero by 1 clock
    NVIC_ST_CURRENT_R = 0;        // this line clears: count flag and current value
    while ((NVIC_ST_CTRL_R & 0x10000) == 0)
        ; // read count flag
}
// this function deals with any delay ms value
void systick_wait_free_ms(uint32_t delay)
{
    uint32_t max = 0xFFFFFF / Clock * 1000;

    while (delay > max)
    {
        systick_wait_ms(max);
        delay -= max;
    }
    systick_wait_ms(delay);
}
//this function checks if final destination reached or not
bool is_final_destination(double distance)
{
    return distance >= RequiredDistance;
}
//this function calculates the distance between two points
double distance_between_points(double longitude1, double latitude1, double longitude2, double latitude2)
{
    double latitudeRadian1; // to radians
    double latitudeRadian2;
    double deltaLatitude;
    double deltaLongitude;

    double a;
    double c;
    double d;

    longitude1 = angel_to_decimal(longitude1);
    longitude2 = angel_to_decimal(longitude2);
    latitude1 = angel_to_decimal(latitude1);
    latitude2 = angel_to_decimal(latitude2);

    latitudeRadian1 = latitude1 * PI / 180; // to radians
    latitudeRadian2 = latitude2 * PI / 180;
    deltaLatitude = (latitude2 - latitude1) * PI / 180;
    deltaLongitude = (longitude2 - longitude1) * PI / 180;

    a = sin(deltaLatitude / 2) * sin(deltaLatitude / 2) +
        cos(latitudeRadian1) * cos(latitudeRadian2) *
            sin(deltaLongitude / 2) * sin(deltaLongitude / 2);
    c = 2 * atan2(sqrt(a), sqrt(1 - a));

    d = R * c; // in meter

    return d;
}

void seven_segments_display(uint32_t number, uint32_t digit)
{
  // E0,1,2,3
  if (digit == 1)
  {
    GPIO_PORTE_DATA_R &= 0xF0;
    GPIO_PORTE_DATA_R |= number;
  }
  else if (digit == 10)
  {
    //D0,1,2,3
    GPIO_PORTD_DATA_R &= 0xF0;
    GPIO_PORTD_DATA_R |= number;
  }
  else if (digit == 100)
  {
    // A4,5,6,7
    GPIO_PORTA_DATA_R &= 0x0F;
    number = number << 4;
    GPIO_PORTA_DATA_R |= number;
  }
}

void double_to_three_digits(double number, uint32_t *d1, uint32_t *d2, uint32_t *d3)
{
  *d3 = (uint32_t)(number) % 10;
  *d2 = (uint32_t)(number / 10) % 10;
  *d1 = (uint32_t)(number / 100) % 10;
}

void dummy_seven_segments(int d3, int d2, int d1)
{
  seven_segments_display(d1, 1);
  seven_segments_display(d2, 10);
  seven_segments_display(d3, 100);
}

double angel_to_decimal(double nmea)
{
    int degrees = (int)(nmea / 100);
    double minutes = nmea - (degrees * 100);
    double dec = degrees + (minutes / 60);

    return dec;
}

bool is_gps_ready()
{
  char t = get_next_char();
  while (t != '$')
  {
    t = get_next_char();
  }
  if (t == '$')
  {
    t = get_next_char();
    while (t != 'G')
    {
      t = get_next_char();
    }
    if (t == 'G')
    {
      t = get_next_char();
      while (t != 'P')
      {
        t = get_next_char();
      }
      if (t == 'P')
      {
        t = get_next_char();
        while (t != 'G')
        {
          t = get_next_char();
        }
        if (t == 'G')
        {
          t = get_next_char();
          while (t != 'L')
          {
            t = get_next_char();
          }
          if (t == 'L')
          {
            t = get_next_char();
            if (t == 'L')
            {
              while (t != 'L')
              {
                t = get_next_char();
              }
              t = get_next_char();
              while (t != ',')
              {
                t = get_next_char();
              }
              if (t == ',')
              {
                t = get_next_char();
                if (t == ',')
                {
                  return false;
                }
                else
                {
                  return true;
                }
              }
            }
          }
        }
      }
    }
  }
  return false;
}

// ziad
void UART1_Init()
{
    SYSCTL_RCGCUART_R |= 0x0002;
    while ((SYSCTL_RCGCUART_R & 0x02) == 0)
    {
    } //wait for UART1 to be activated
    UART1_CTL_R &= ~0x0001; // disable UART1
    UART1_IBRD_R = 104;
    UART1_FBRD_R = 11;
    UART1_LCRH_R = 0x0070;
    UART1_CTL_R = 0x0301;

    GPIO_PORTB_LOCK_R = 0x4C4F434B;
    GPIO_PORTB_CR_R = 0xFF;
    GPIO_PORTB_AFSEL_R |= 0x03;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) + 0x00000011;
    GPIO_PORTB_AMSEL_R &= ~0x03;
    GPIO_PORTB_DEN_R = 0xFF;
}
//UART5 Initialization
void UART5_Init(){
    SYSCTL_RCGCUART_R |= 0x20; /* enable clock to UART5 */
    while ((SYSCTL_RCGCUART_R & 0x20) == 0)
    {
    } //wait for UART5 to be activated
     /* UART5 initialization */
    UART5_CTL_R = 0;// disable UART5
    UART5_IBRD_R = 104;
    UART5_FBRD_R = 11;
    UART5_CC_R = 0;
    UART5_LCRH_R = 0x60;
    UART5_CTL_R = 0x301;

    GPIO_PORTE_PCTL_R |= 0x00110000;
    GPIO_PORTE_DEN_R |= 0x30;
    GPIO_PORTE_DEN_R |= 0x30;
    GPIO_PORTE_AFSEL_R |= 0x30;
    GPIO_PORTE_AMSEL_R |= 0;
    GPIO_PORTE_PCTL_R |= 0x00110000;
}

// ziad
char get_next_char()
{
    // char string[204] = "$GPGLL,3017.32780,N,03143.54992,E,174810.00,A,A*6F\n$GPGLL,3017.32722,N,03143.54938,E,174809.00,A,A*6F";
    // if (stringLength == 203)
    // {
    //     stringLength = 0;
    // }
    // return string[stringLength++];
    char data;
    while ((UART1_FR_R & 0x10) != 0)
    {
    }
    data = UART1_DR_R;
    //	serial_send(data);

    return (unsigned char)data;
}

int read_current_coordinates(double *longitude, double *latitude)
{
  while (
      get_next_char() != '$' ||
      get_next_char() != 'G' ||
      get_next_char() != 'P' ||
      get_next_char() != 'G' ||
      get_next_char() != 'L' ||
      get_next_char() != 'L')
  {
    continue;
  }
  parse_coor(longitude, latitude);
  return 0;
}
void serial_send(char data)
{
    while ((UART5_FR_R & 0x20) != 0)
        ;              /* wait until Tx buffer not full */
    UART5_DR_R = data; /* before giving it another byte */
}

void print_it(char s[], int length)
{
    int i = 0;
    for (i; i < length; i++)
    {
        serial_send(s[i]);
    }
}

// ziad
int parse_coor(double *longitude, double *latitude)
{
    int j = 0;
    double multiblier = 1000;
    char t = get_next_char();

    while (t != ',')
    {
        t = get_next_char();
    }
    if (t == ',')
    {
        t = get_next_char();
        if (t == ',')
        {
            return 0;
        }
        *longitude = 0;
        while (t != ',')
        {
            final_coordinates[coordinatesIndex][j++] = t;
            serial_send(t);
            *longitude += ((int)(t - '0')) * multiblier;
            multiblier = multiblier / 10;
            t = get_next_char();
            if (t == '.')
            {
                final_coordinates[coordinatesIndex][j++] = t;
                serial_send(t);
                t = get_next_char();
                continue;
            }
        }
        final_coordinates[coordinatesIndex][j++] = ',';
        serial_send(',');

        multiblier = 10000;
        t = get_next_char();
        t = get_next_char();

        if (t == ',')
        {
            t = get_next_char();
            if (t == ',')
            {
                return 0;
            }
            *latitude = 0;
            serial_send(t);
            while (t != ',')
            {
                final_coordinates[coordinatesIndex][j++] = t;
                serial_send(t);
                *latitude += ((int)(t - '0')) * multiblier;
                multiblier = multiblier / 10;
                t = get_next_char();
                if (t == '.')
                {
                    serial_send(t);
                    final_coordinates[coordinatesIndex][j++] = t;
                    t = get_next_char();
                    continue;
                }
            }
            final_coordinates[coordinatesIndex][j++] = ',';
            coordinatesIndex = (coordinatesIndex + 1) % 1000;
            serial_send(',');
            return 0;
        }
    }
}