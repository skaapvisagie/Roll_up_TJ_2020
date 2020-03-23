
//Project : DCBA Roll-up Testjig 2018
//
//Device : 16F1527
//
//Date : 22/02/2018
//
//Author : Leon Damaskinos

#define PGM_Enable // comment out for programming disable
//#define Buzz_disable // comment out for buzzer disable on TJ
//#define Board_identifier // comment out for identifier disable

// <editor-fold defaultstate="collapsed" desc="Config and Includes">
#include <xc.h>
#include <stdlib.h>
#include <pic.h>
#asm
//set mcu freq for __delay_() function
#define _XTAL_FREQ  16000000
#ifndef __delay_us
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/8000000.0)))
#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/8000.0)))
#endif
#endasm

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT disabled)
#ifndef __DEBUG
#pragma config PWRTE = ON    // Power-up Timer Enable->PWRT enabled
#pragma config MCLRE = ON    // MCLR Pin Function Select->MCLR/VPP pin function is digital input
#pragma config CP = ON          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#endif
#ifdef __DEBUG
#pragma config PWRTE = OFF   // Power-up Timer Enable->PWRT enabled
#pragma config MCLRE = ON    // MCLR Pin Function Select->MCLR/VPP pin function is digital input
#pragma config CP = OFF          // Flash Program Memory Code Protection (Program memory code protection is enabled)
#endif
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = HI        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), high trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Hardware definitions">
#define Opto_motor_dir_a       RA0
#define Opto_motor_dir_b       RA1

#define lcd_reset       RA5
#define lcd_backlight   RA4
#define i2c_lcd_address      0x7C
#define i2c_pot_address         0x5C //[tpl0401a]
#define trigger         LATD2 // change up // trigger 
#define buzzer          LATC5
#define button          RE1
#define Remote_Tx       LATF4
#define encoder         LATF7
#define Sink_Output     LATE3

#define CHECK_BIT(var,pos) !!((var) & (1<<(pos))) //allows us to check the bits of an int (useful for checking registers)

#define pickit_rel_1_rx        LATC2 //k1
#define pickit_rel_2_rx        LATC7 //k5

//both of these relays are powered via k4, see bottom of board
#define k1        LATC2 //k1
#define k2        LATC6 //k2
#define k3        LATB7 //k3
#define k4        LATA7 //k4
#define k5        LATC7 //k5 

#define ch2_tx      LATG3 //black wire, channel 2
#define ch1_tx      LATG2 //blue wire, channel 1

#define pickit_button_relay LATC7 //c7 and c6 tied together on rhs of Q4 on testjig
#define pickit_LED_blue     RE4
#define pickit_LED_green      RE6
#define pickit_LED_red    RE5

//#define CH2_high_current_K13_P10 LATB5-
//#define CH3_high_current_K6_P13 LATA6

#define power_supply_k6 LATA6
#define power_supply_k7 LATD4
#define power_supply_k8 LATD5
#define power_supply_k9 LATD6
#define power_supply_k10 LATD7
#define power_supply_k11 LATB4
#define power_supply_k12 LATB6
#define power_supply_k13 LATB5


#define data_switch LATB6 

#define comms_MCLR_input RG1
#define comms_TX_in RG0
#define comms_TX LATG0
#define comms_TX_TRIS TRISG0
#define comms_RX RG3

//Inputs

#define board_detect RF5

#define mclr_reset_P10 LATD7
#define mclr_reset_P10_TRIS TRISD7

// Pi connections A2, A3 F0
#define A3 RA3 //tied to pin 31 which is uart tx1, do not use
#define A2 RA2
#define F0 RF0

#define BT LATG0
#define LIM LATG3
#define BM LATG4

//#define relay_NC_P4 RF5
#define relay_NO_P8 RF2

#define Bus_input RC1
#define Bus_fet LATC0

#define RF_button   RG0
#define RF_LED      RG3
#define RF_out     RE0

//#define debug_1 LATF4

//definitions for checking states of DUT
#define ENC 0
#define ECAN_RX 1
#define BUTTON_UP 2
#define BUTTON_DOWN 3
#define FET_SENSE 4
#define LIM_IN 5
#define BT_IN 6

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="function declarations">
void i2c_Write(unsigned char data);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_Write_string(const unsigned char *data);
void i2c_write_char(unsigned char char_data);
void init_i2c_lcd_5v(unsigned char contrast, unsigned char single1_double2);
void clear_lcd(void);
void i2c_first_line(void);
void i2c_second_line(void);
void lcd_createChar(unsigned char location, unsigned char charmap[]);
void init_i2c(void);
void init_current_sink(void);
unsigned char get_motor_direction(void);
void power_supply_set(unsigned char supply);
unsigned int get_current(void);
void Vout_set(int Volts);
void digitalpot(unsigned char value);
void debug_fast_tx(unsigned int data);
void TX_one(unsigned char data);
unsigned char RX_one(void);
unsigned char RX_one_timeout(void);
void init_ADC(void);
unsigned int ADC_to_mv(unsigned int ADC);
unsigned int ADC_to_mv_DUT(unsigned int ADC); //use this for conversions on the DUT monitor as it uses 5v instead of 4.096v reference.
unsigned int ADC_get_val(unsigned char channel);
float ADC_to_voltage(unsigned int ADC);
unsigned int board_detected(unsigned int ADC);
unsigned int board_type(unsigned int ADC);
unsigned int board_type(unsigned int ADC);
unsigned int DUT_comms(unsigned int callsign);
unsigned int DUT_voltage_check(unsigned int voltage_actual, unsigned int voltage_req);

void print_error(const unsigned char *top, const unsigned char *bottom);
void print_screen(const unsigned char *top, const unsigned char *bottom);
void lcd_print_int(unsigned int number, unsigned int pos, unsigned char decimal, unsigned char ten_thou_digit);
/**
 This function initializes the USART2 module to the following parameters: \n
 TXEN = 1, transmitter enabled \n
 BRGH = 1, High baud rate \n
 SPEN = 1, serial port enable \n
 CREN = 1, Receiver enabled \n
 BRG16 = 1, 16 bit baud rate generator\n
 Baud Rate = 117647 baud.
 @param None
 @return None
 @Calls
 */
void init_uart1(void);
void init_uart2(void);
/**
 This function disables the USART2 module to allow the pins to be used as RF inputs. \n
  TXEN = 0, transmitter disabled \n
  CREN = 0, Receiver disabled \n
  SPEN = 0, serial port disable
 @param None
 @return None
 @Calls
 */
void disable_uart(void);
/**
 This function enables the USART2 for for RF module communications. \n
  TXEN = 1, transmitter enabled \n
  SPEN = 1, serial port enable \n
 then calls uart_reset();
 @param None
 @return None
 @Calls uart_reset();
 */
void enable_uart(void);
/**
 This function waits for the USART2 tx module to be adle and then begins transmision of 1 byte over uart.
 @param uart1_data - data to be transmitted.
 @return None
 @Calls
 */
void send_uart1(unsigned char uart1_data);
void send_uart2(unsigned char uart1_data);
/**
 This function resets the USART2 and clears the buffer
 @param None
 @return None
 @Calls
 */
void uart_reset(void);
/**
 This function recieves 1 packet (4 bytes) over USART2.
 @param uart1_timer - the timeout for the receiver, decremented once per TMR0 interrupt.
 @return UART1_RESET - if a comms error occured and the uart was reset. \n
 UART1_BUFFER_FULL - if 4 bytes were received. \n
 UART1_TIMEOUT - if a time-out occured
 @Calls
 */
unsigned char uart_receive_four(unsigned int uart1_timer);
/**
 This function sends one packet (4 bytes, or 1 if opcode == 0x89) over uart and then receives 1 packet.
 @param opcode - the first byte to be sent\n
 byte_1, byte_2, byte3 - the second, third and fourth bytes to be transmitted respectively. \n
 uart2_receive_timer - the timeout passed to uart_receive_four();
 @return returns the state returned by uart_receive_four().
 @Calls
 */
unsigned char uart_send_receive_packet(unsigned char opcode, unsigned char byte_1, unsigned char byte_2, unsigned char byte_3, unsigned int uart2_receive_timer);

void set_rollup(unsigned char cmd, unsigned char data);
/*
 Commands Roll Up, see excel sheet in the S drive, in Testjig/Commands folder
 * Errors with checksum high byte, low can still be used to check
 */
void get_rollup(unsigned char cmd);
/*
 See set_rollup
 Different behaviour in that checksum will never have a high byte, see command list.
 */

void buzz_one(unsigned int time);
void testjig_done(unsigned char state, unsigned char device_type);
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="User definitions">

enum {
    current_sink_off, current_sink_on, sampling_ADC, updating_PWM
} current_sink_state;

enum {
    pickit_finish, pickit_start, pickit_push_button, pgm_start_delay,  pickit_busy, pickit_error, pickit_idle
} pickit_state;

//adc channel select
#define ADC_current_sink    0b01001101//channel 19 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_v2_test         0b01110101//channel 29 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_3v3_test        0b01100101//channel 25 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_5v_test         0b01100001//channel 24 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_24v_test        0b01011101//channel 23 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_v1_test         0b01101001//channel 26 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_Q_curr_sense    0b01010001//channel 20 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define Pi_to_TJ_trigger    0b01000001//channel 16, RF0 (channels are bit shifted up by two bits and padded with 11 to set Go_nDone and ADC_on)
#define ADC_Motor_1         0b00000101//channel 1 for checking the optocoupler for motor 1
#define ADC_Motor_2         0b00000001//channel 0 for checking the optocoupler for motor 2

#define UART1_RESET        0
#define UART1_TIMEOUT      1
#define UART1_BUFFER_FULL  2

#define NO_motor_current 0xFF
#define Direction_1 0x01
#define Direction_2 0x02

#define AC1     0
#define AC2     1
#define BAT     2
#define ACTEST  3
#define NONE    4

#define Ctest_off       0
#define Ctest_battery   1
#define Ctest_12v       2
#define Ctest_24v       3

#define one_second_timer_value 1000

#define Relay 0
#define Diodes 1

#define YES                     1
#define NO                      0

#define USART_FAULT_RESET               0
#define USART_FAULT                     1

#define DISABLE_TIMEOUT         0
#define ENABLE_TIMEOUT          1

#define ADC_reference_voltage 4.096
#define ADC_resolution 1024

#define Vtest_P5_CH23_div_ratio 11.0
#define Vtest_P5_CH23_5v (((5/Vtest_P5_CH23_div_ratio)*ADC_resolution)/ADC_reference_voltage)
#define P5_5v_test_min (unsigned int)(Vtest_P5_CH23_5v*0.92)
#define P5_5v_test_max (unsigned int)(Vtest_P5_CH23_5v*1.08)

#define Vtest_P2_CH24_div_ratio 11.0
#define Vtest_P2_CH24_12 (((12/Vtest_P2_CH24_div_ratio)*ADC_resolution)/ADC_reference_voltage)
#define P2_12v_test_min (unsigned int)(Vtest_P2_CH24_12v*0.9)
#define P2_12v_test_max (unsigned int)(Vtest_P2_CH24_12v*1.2)

#define T0_1s_float 7518.8

#define T0_100ms    (unsigned int)(T0_1s_float/10)
#define T0_250ms    (unsigned int)(T0_1s_float/4)
#define T0_400ms    (unsigned int)(T0_1s_float/2.5)
#define T0_500ms    (unsigned int)(T0_1s_float/2)
#define T0_750ms    (unsigned int)(3*(T0_1s_float/4))
#define T0_1s       (unsigned int)(T0_1s_float)
#define T0_2s       (unsigned int)((2*T0_1s_float))
#define T0_3s       (unsigned int)((3*T0_1s_float))
#define T0_4s       (unsigned int)((4*T0_1s_float))
#define T0_5s       (unsigned int)((5*T0_1s_float))


//Device Hexcodes
#define Device_Condo3       0xA0
#define Device_RX3          0xA1
#define Device_Mem_Copier   0xA2
#define TXMicro2            0xA3
#define RXMicro2            0xA4
#define DCBA_Rollup         0xA5

unsigned char usart_receive_state_main;
#define RESET_RECEIVE_PROTOCOL              0
#define RECEIVE_COMMAND_BYTE_1              1
#define RECEIVE_DATA_HIGH_BYTE_2            2
#define RECEIVE_DATA_LOW_BYTE_3             3
#define RECEIVE_DATA_CHECKSUM_HIGH_BYTE_4   4
#define RECEIVE_DATA_CHECKSUM_LOW_BYTE_5    5
#define DATA_VALIDATION                     6
#define FIND_AND_EXECUTE_COMMAND            7
#define USART_ERROR                         8

unsigned char usart_fault_state;
#define USART_FAULT_STATE_NONE          0
#define COMMS_TIMEOUT                   1
#define COMMS_DATA_ERROR                2
//#define
//unsigned char tesjig_state_mode;
//#define CHECK_FOR_TESTJIG                   0
//#define RUN_TEST_NORMALLY                   1


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Global variables">
volatile bit usart_fault_flag;
const unsigned int popup_delay = 15; //delay a popup is shown for, in 100ms
unsigned char button_beep_delay = 2 ;
unsigned char ten_thou, thou, hund, ten, unit; //0-256 GLOBALS!!!!
unsigned char uart_buff_work_recieve[5];
volatile int uart_receive_timer, motor_direction_timer;
volatile unsigned int pickit_timer, pickit_all_counter, pickit_pgm_timeout, usart_timer;
volatile unsigned int current_sink_timer, testjig_timer;
unsigned char Data_Temp_Work_char, return_data;
volatile unsigned int one_second_timer = one_second_timer_value;
volatile unsigned int buzz_timer;
volatile unsigned int ADC_cur_sink_val;
volatile unsigned int curent_sink_val, current_sink_stable;
unsigned char RelayVsDiodes = Relay;
unsigned int ADC_val = 0;
float voltage_f = 0;
unsigned int voltage_i, Data_Temp_Work_int;
unsigned int pickit_red_counter; 
unsigned int trip_current;
unsigned int tj_timer = 0;
unsigned int detect = 0;
unsigned int device = 0;
unsigned int voltage_actual;
unsigned int current_actual;
unsigned int voltage_req;
unsigned int callsign;
unsigned int lower=0;
unsigned int higher=0;
unsigned char volt_reading;
unsigned int voltage_outcome; //0=below threshold, 1=above, 2= within
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="USART LEON STUFF">
void EUSART1_Write(unsigned char txData)

{
    while (TX2STAbits.TRMT == 0); //Transmission still in progress? or exit pressed
    TX2REG = txData; //send data
//    while(0 == PIR3bits.TX1IF)
//    {
//    }
//
//    TX1REG = txData;    // Write the data byte to the USART.
}

void send_usart_packet_manage(unsigned char command_byte_1, unsigned int Data_carry)
{
    unsigned int checksum_send_work;
    unsigned char  data_high_byte_2, data_low_byte_3, checksum_high_byte_4, checksum_low_byte_5;
       
    //--extract & load data
    data_high_byte_2 = Data_carry >> 8;
    data_low_byte_3 = Data_carry & 255;        
    //--        
    //-- add checksum
    checksum_send_work = 0;
    checksum_send_work = checksum_send_work + command_byte_1 + data_high_byte_2 + data_low_byte_3;
        
    checksum_high_byte_4 = checksum_send_work >> 8;
    checksum_low_byte_5 = checksum_send_work & 255;
    
    //--
    
    //--SEND BYTES
    EUSART1_Write(command_byte_1);
    EUSART1_Write(data_high_byte_2);
    EUSART1_Write(data_low_byte_3);
    EUSART1_Write(checksum_high_byte_4);
    EUSART1_Write(checksum_low_byte_5);
    //--
}

unsigned char manage_read_byte(unsigned char timeout_enable)
{
    usart_fault_flag = USART_FAULT_RESET; 
    
    while(!RC2IF)
    {
        if(timeout_enable == DISABLE_TIMEOUT)
        {
            usart_timer = 30; //ms
        }
        
        if(usart_timer == 0)
        {
            usart_fault_flag = USART_FAULT; 
            return 0;
        }
        
//        if((testjig_search_timer == 0)&&(tesjig_state_mode == CHECK_FOR_TESTJIG)) //session to search for jig timed out?
//        {//yes
//            usart_fault_flag = USART_FAULT;
//            return 0;
//        }
    }
    
    if(1 == RC2STAbits.OERR)
    {
        // EUSART1 error - restart

        RC2STAbits.SPEN = 0; 
        RC2STAbits.SPEN = 1; 
        usart_fault_flag = USART_FAULT;
        return 0;
    }

    return RC2REG;
}

unsigned int receive_data_bytes(unsigned int time_out_first_byte)
{
    unsigned char command_byte, data_high_byte, data_low_byte, checksum_high_byte, checksum_low_byte;
    unsigned int bit_data, usart_validation_work, usart_checksum, data_int_value;
    
//    testjig_search_timer = TIMER_1000MS; //set for testjig search session 
    usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
    //tesjig_state_mode = CHECK_FOR_TESTJIG; //set this if timeout needed
    //tesjig_state_mode = RUN_TEST_NORMALLY; //disable testjig search timeout
    
    while(1)
    {
        switch(usart_receive_state_main) //state machine  
        {
            case RESET_RECEIVE_PROTOCOL:
                
                while(1 == RC2IF)
                {
                    command_byte = RC2REG; //clear buffer & flag 
                }
                
                usart_fault_state = USART_FAULT_STATE_NONE;
 
                usart_receive_state_main = RECEIVE_COMMAND_BYTE_1;
            break;    

            case RECEIVE_COMMAND_BYTE_1:
                
                usart_timer = time_out_first_byte;
                        
                command_byte = manage_read_byte(ENABLE_TIMEOUT);

                if(usart_fault_flag == USART_FAULT)
                {
                   //usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
                   usart_fault_state = COMMS_TIMEOUT; 
                   usart_receive_state_main = USART_ERROR; 
                }    
                else
                {
                   usart_receive_state_main = RECEIVE_DATA_HIGH_BYTE_2; 
                }
            break;

            case RECEIVE_DATA_HIGH_BYTE_2:
                usart_timer = 30; //ms
                
                data_high_byte = manage_read_byte(ENABLE_TIMEOUT);
                
                if(usart_fault_flag == USART_FAULT)
                {
                   //usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
                    usart_fault_state = COMMS_TIMEOUT; 
                    usart_receive_state_main = USART_ERROR; 
                }    
                else
                {
                    usart_receive_state_main = RECEIVE_DATA_LOW_BYTE_3; 
                }
            break;
            
            case RECEIVE_DATA_LOW_BYTE_3:
                usart_timer = 30; //ms
                
                data_low_byte = manage_read_byte(ENABLE_TIMEOUT);
                
                if(usart_fault_flag == USART_FAULT)
                {
                   //usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
                    usart_fault_state = COMMS_TIMEOUT; 
                    usart_receive_state_main = USART_ERROR; 
                }    
                else
                {
                   usart_receive_state_main = RECEIVE_DATA_CHECKSUM_HIGH_BYTE_4; 
                }
            break;
            
            case RECEIVE_DATA_CHECKSUM_HIGH_BYTE_4:
                usart_timer = 30; //ms
                
                checksum_high_byte = manage_read_byte(ENABLE_TIMEOUT);
                
                if(usart_fault_flag == USART_FAULT)
                {
                   //usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
                    usart_fault_state = COMMS_TIMEOUT; 
                    usart_receive_state_main = USART_ERROR; 
                }    
                else
                {
                   usart_receive_state_main = RECEIVE_DATA_CHECKSUM_LOW_BYTE_5; 
                }
            break;
            
            case RECEIVE_DATA_CHECKSUM_LOW_BYTE_5:
                usart_timer = 30; //was 30 ms
                
                checksum_low_byte = manage_read_byte(ENABLE_TIMEOUT);
    
                if(usart_fault_flag == USART_FAULT)
                {
                   //usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
                    usart_fault_state = COMMS_TIMEOUT; 
                    usart_receive_state_main = USART_ERROR; 
                }    
                else
                {
                   usart_receive_state_main = DATA_VALIDATION; 
                }
            break;
            
            case DATA_VALIDATION:
                 //beeper_set(300, 300, 1); //on time/ off time/ count
                 //while(beeper_state != BEEPER_OFF);
                 usart_checksum = (checksum_high_byte << 8) | checksum_low_byte;
                 usart_validation_work = 0;
                 usart_validation_work = usart_validation_work + command_byte + data_high_byte + data_low_byte;
                
                 if(usart_checksum == usart_validation_work)
                 {
                     //beeper_set(300, 300, 1); //on time/ off time/ count
                     usart_receive_state_main = FIND_AND_EXECUTE_COMMAND; 
                 }
                 else
                 {
                     //usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
                        usart_fault_state = COMMS_DATA_ERROR; 
                        usart_receive_state_main = USART_ERROR; 
                 }
            break;
            
            case FIND_AND_EXECUTE_COMMAND:
                
                data_int_value = (data_high_byte << 8) | data_low_byte;
                
                return data_int_value;
//                buzz_timer = 500; //ms
//                buzzer = 1;
            break;
            
            case USART_ERROR:
                if(usart_fault_state == COMMS_TIMEOUT)
                {
                    print_error("Comms Timeout", "");
                }
                else if(usart_fault_state == COMMS_DATA_ERROR)
                {
                    print_error("Comms_Data_Err", "");
                }
                
                //return 0;
                
            break;
            
//            case FIND_AND_EXECUTE_COMMAND:
//                //AD VALUE REQUEST
//                if(command_byte == 128) //VALUE MONITOR V1 - SEND
//                {
//                    send_usart_packet_manage(command_byte, ad_buffer_AC_voltage_sens); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 129) //VALUE MONITOR V2 - SEND
//                {
//                   send_usart_packet_manage(command_byte, ad_buffer_batt_voltage_sens); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 130) //VALUE MONITOR 24v - SEND
//                {
//                    send_usart_packet_manage(command_byte, ad_buffer_24_voltage_sens); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 131)
//                {
//                    //NOT APPLICABLE
//                }
//                else if(command_byte == 132) //VALUE MOTOR CURRENT SENSE - SEND
//                {
//                    send_usart_packet_manage(command_byte, ad_buffer_mot_curr_sens); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 133) //VALUE BEAM IN - SEND
//                {
//                    send_usart_packet_manage(command_byte, ad_buffer_beam_sens); //send (commanded value, data) & add checksum bytes
//                }
//                //SET OUTPUT PIN REQUEST
//                else if(command_byte == 134) //ECAN TX SET PIN
//                {
//                    if(data_low_byte == 2)
//                    {
//                        ECAN_DATA_OUT = 1;
//                    }
//                    else
//                    {
//                        ECAN_DATA_OUT = 0;
//                    }
//                    
//                    send_usart_packet_manage(134, 134); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 135) //LED B SET PIN
//                {
//                   if(data_low_byte == 2)
//                    {
//                       status_led_B_state = SYSTEM_LED_STATUS_ON;
//                        //LED_B = 1;
//                    }
//                    else
//                    {
//                       status_led_B_state = SYSTEM_LED_STATUS_OFF;
//                        //LED_B = 0;
//                    }
//                    
//                    send_usart_packet_manage(135, 135); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 136) //LED ENCODER SET PIN
//                {
//                    if(data_low_byte == 2)
//                    {
//                        LED_ENCODER = 1;
//                    }
//                    else
//                    {
//                        LED_ENCODER = 0;
//                    }
//                    
//                    send_usart_packet_manage(136, 136); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 137) //LED A SET PIN
//                {
//                    if(data_low_byte == 2)
//                    {
//                        status_led_A_state = SYSTEM_LED_STATUS_ON;
//                    }
//                    else
//                    {
//                        status_led_A_state = SYSTEM_LED_STATUS_OFF;
//                    }
//                    
//                    send_usart_packet_manage(137, 137); //send (commanded value, data) & add checksum bytes
//                    
//                }
//                else if(command_byte == 138) //LED RF SET PIN
//                {
//                    if(data_low_byte == 2)
//                    {
//                        LED_RF = 1;
//                    }
//                    else
//                    {
//                        LED_RF = 0;
//                    }
//                    
//                    send_usart_packet_manage(138, 138); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 139) //BAT DISCONECTION ENABLE/DISABLE
//                {
//                    if(data_low_byte == 2)
//                    {
//                        BATT_DISC_DISABLE = 1;
//                    }
//                    else
//                    {
//                        BATT_DISC_DISABLE = 0;
//                    }
//                    
//                    send_usart_packet_manage(139, 139); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 140) //BUZZER SET OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        buzzer_isr_flag = 1;
//                    }
//                    else
//                    {
//                        buzzer_isr_flag = 0;
//                    }
//                    
//                    send_usart_packet_manage(140, 140); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 141) //BATT CHARGE SET OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        BATT_CHARGE_ENABLE =  1;
//                    }
//                    else
//                    {
//                        BATT_CHARGE_ENABLE = 0;
//                    }
//                    
//                    send_usart_packet_manage(141, 141); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 142) //MOTOR PWM OUTPUT DUTY SET
//                {
//                     
//                    isr_target_speed  = data_low_byte; //PWM IS SET IN ISR
//                    
//                    send_usart_packet_manage(142, 142); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 143) //LIGHT SET OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        light = 1;
//                    }
//                    else
//                    {
//                        light = 0;
//                    }
//
//                    send_usart_packet_manage(143, 143); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 144) //BEAM LED SET OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        LED_BEAM = 1;
//                    }
//                    else
//                    {
//                        LED_BEAM = 0;
//                    }
//
//                    send_usart_packet_manage(144, 144); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 145) //24V AUX LED SET OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        LED_24V_AUX = 1;
//                    }
//                    else
//                    {
//                        LED_24V_AUX = 0;
//                    }
//
//                    send_usart_packet_manage(145, 145); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 146) //SET MOTOR PLUS RELAY OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        motor_plus_relay = 1;
//                    }
//                    else
//                    {
//                        motor_plus_relay = 0;
//                    }
//                    
//                    send_usart_packet_manage(146, 146); //send (commanded value, data) & add checksum bytes
//                }
//                else if(command_byte == 147) //SET MOTOR MINUS RELAY OUTPUT
//                {
//                    if(data_low_byte == 2)
//                    {
//                        motor_minus_relay = 1;
//                    }
//                    else
//                    {
//                        motor_minus_relay = 0;
//                    }
//                    
//                    send_usart_packet_manage(147, 147); //send (commanded value, data) & add checksum bytes
//                }
//                //STATUS OF PIN REQUESTED
//                else if(command_byte == 148)
//                {
//                    bit_data = 0;
//                    
//                    if(ENCODER_IN == 1)
//                    {
//                        bit_data = bit_data | 1;
//                    }
//                    if(ECAN_DATA_IN == 1)
//                    {
//                        bit_data = bit_data | 2;
//                    }
//                    if(button_2 == 1)
//                    {
//                        bit_data = bit_data | 4;
//                    } 
//                    if(button_1 == 1)
//                    {
//                        bit_data = bit_data | 8;
//                    } 
//                    if(FET_SENSE == 1)
//                    {
//                        bit_data = bit_data | 16;
//                    } 
//                    if(LIMIT_IN == 1 == 1)
//                    {
//                        bit_data = bit_data | 32;
//                    } 
//                    if(BT_IN == 1)
//                    {
//                        bit_data = bit_data | 64;
//                    } 
//                    
//                    send_usart_packet_manage(148, bit_data); //send (commanded value, data) & add checksum bytes    
//                }
// 
//                else if(command_byte == 254) //testjig found ?
//                {//yes - send echo
//                    tesjig_state_mode = RUN_TEST_NORMALLY; //disable testjig search timeout
//                    send_usart_packet_manage(254, 254); //send (commanded value, data) & add checksum bytes
//                }
//                
//                else if(command_byte == 253) //test passed ?
//                {//yes - send echo
//                    User_Mem_write_value(Testjig_status, 3);//write 3 to flash indicating a pass
//                    send_usart_packet_manage(253, 253); //send (commanded value, data) & add checksum bytes
//                }
//                
//                else if(command_byte == 252) //request valid Rf decryption
//                {//yes -
//                    Decryption_valid_flag = DECRYPTION_RESET;
//                    
//                    usart_timer = TIMER_1000MS;
//                    
//                    while(1)
//                    {
//                        if(Decryption_valid_flag == DECRYPTION_VALID)
//                        {
//                            send_usart_packet_manage(252, 2); //send (commanded value, data) & add checksum bytes
//                            break;
//                        }
//                        if(usart_timer == 0)
//                        {
//                            send_usart_packet_manage(252, 1); //send (commanded value, data) & add checksum bytes
//                            break;
//                        }
//                    }//loop
//                }
//                
//                usart_receive_state_main = RESET_RECEIVE_PROTOCOL;
//            break;
            
        }//switch
        

    }//loop
       
}
// </editor-fold>
//////////////////////////////////////...Code Starts Here...///////////////////////////////////////

// <editor-fold defaultstate="collapsed" desc="Interrupt routines">

void interrupt
isr(void) {
    if (INTCONbits.TMR0IF == 1) //timer 0 interrupt, 128us
    {
        if (uart_receive_timer)
            uart_receive_timer--;
        if (motor_direction_timer)
            motor_direction_timer--;

        INTCONbits.TMR0IF = 0;

    }
    if (PIR1bits.TMR2IF == 1)//timer 2 interrupt, 1ms
    {//used to control the current sink
        if(testjig_timer)
        {testjig_timer--;}
        
        if(usart_timer)
        {usart_timer--;}
        
        if(pickit_timer)
        {
            pickit_timer--;
        }
        else 
        {
            switch (pickit_state) 
            {
                case pickit_start:
                    //pickit_rel_1 = 1;
                    //pickit_rel_2 = 1;
                    k3 = 1;
                    k4 = 1;
                    pickit_timer = 500; //ms allow for relays to switch
                    pickit_state = pickit_push_button;
                break;
                
                case pickit_push_button:
                    pickit_timer = 500; //allow 500ms for relays to push button
                    pickit_button_relay = 1;
                    pickit_state = pgm_start_delay;
                    
                break;
                
                case pgm_start_delay:
                    pickit_button_relay = 0;
                    pickit_pgm_timeout = 0;
                    pickit_timer = 500; //allow 500ms after release of push button
                    pickit_state = pickit_busy;
                break;        
                
                case pickit_busy:
                    
                    pickit_pgm_timeout++;
                    if (pickit_pgm_timeout > 8000) //ms  
                    {
                        pickit_state = pickit_finish;;
                    }
                    //-- error?
                    if((pickit_LED_blue == 0)&&(pickit_LED_red == 1))
                    {
                        pickit_all_counter++;
                        
//                         if (pickit_all_counter > 150) //150ms
//                        {
                        pickit_state = pickit_error;
//                        }
 
                    }
                    else
                    {
                        pickit_all_counter = 0;
                    }
                    //-- pass?
                    if((pickit_LED_blue == 0)&&(pickit_LED_red == 0))
                    {
                        pickit_red_counter++;
                        if (pickit_red_counter > 400) //ms
                        {
                            pickit_state = pickit_finish;
                        }
                    }
                    else
                    {
                        pickit_red_counter = 0;
                    }
                    
//                    if (pickit_LED_red == 1) 
//                    {
//                        pickit_red_counter++;
//                        
//                        if (pickit_red_counter > 3125) //300ms
//                        {
//                            pickit_state = pickit_error;
//                        }
//                    } 
//                    else
//                    {
//                        pickit_red_counter = 0;
//                    }
//                    
//                    
//                    if (pickit_LED_green == 0)//an error occurred
//                    {
//                        pickit_state = pickit_error;
//                        pickit_timer = 1000;
//                    } 
//                    else if (pickit_LED_blue == 0)//back in standby
//                    {
//                        pickit_state = pickit_finish;
//                    }
                    pickit_timer = 1;
                    
                break;
                case pickit_error:
                    k3 = 0;
                    k4 = 0;
                    //pickit_rel_1 = 0;
                    //pickit_rel_2 = 0;
                break;
                case pickit_finish:
                    k3 = 0;
                    k4 = 0;
                    //pickit_rel_1 = 0;
                    //pickit_rel_2 = 0;
                    pickit_state = pickit_idle;
                break;
                case pickit_idle:
                break;
            }
        }

        if(RC2IF==1){
            //  Uart_read_buffer[++]==RC2REG
            //  clear flag
            //  change in function ~ if UART read buffer is full, prevent new reads and use buffer for program 
        }
        
        if (buzz_timer)
            buzz_timer--;
        else
            buzzer = 0;
        if (one_second_timer)
            one_second_timer--;
        else {
            one_second_timer = one_second_timer_value;
        }
        if (tj_timer)
            tj_timer--;
        PIR1bits.TMR2IF = 0;
    }
}
// </editor-fold>

void main(void) 
{
    // <editor-fold defaultstate="collapsed" desc="Oscilator and port setup">
    OSCCON = 0x7A;
    //    OPTION_REG = 0b01000001; //PULLUPS ENABLE/INT INSTR CLOCK;/PRE= divide by 4
    OPTION_REG = 0b11010000;
    TMR0 = 0x00;
    TMR0IF = 0;
    TMR0IE = 1;

    //--SETUP PORTS
    TRISA = 0b00001011; //
    TRISB = 0b00001111; //
    TRISC = 0b00011010; //
    TRISD = 0b10001111; //
    TRISE = 0b11110110; //
    TRISF = 0b00101011; //
    TRISG = 0b11100110; // //bit 2&1=1 for uart2 usage / G1=TX G2=RX
    //
    PORTA = 0b00000000;
    PORTB = 0b00000000;
    PORTC = 0b00000000; //
    PORTD = 0b01000000; //
    PORTE = 0b00000000; //
    PORTF = 0b00000000; //
    PORTG = 0b00000000; //
    //
    WPUB = 0b00000000; //
    WPUD = 0b00000000; //
    WPUE = 0b00000000; //
    //
    ANSELA = 0b00000000; //
    ANSELB = 0b00001100; //
    ANSELD = 0b00001111; //
    ANSELE = 0b00000110; //
    ANSELF = 0b00000001; //
    ANSELG = 0b00000000; //
    // </editor-fold>
    power_supply_set(NONE);

    init_ADC();
    init_current_sink();
    pickit_state = pickit_idle;
    //    current_sink_state = current_sink_off;
    lcd_reset = 0;
    buzz_one(200);
    lcd_reset = 1; // disable reset on LCD
    init_i2c(); //init I2C module on MCU
    init_i2c_lcd_5v(8, 0x38);
    print_screen("DCBA Roll-up", "Test-Jig|Startup");
    lcd_backlight = 1;
    GIE = 1;
    __delay_ms(100);
    debug_fast_tx(0xAAAA);
    __delay_us(100);
    debug_fast_tx(0x5555);
    __delay_us(100);
    
    power_supply_set(NONE);
    
    buzz_timer = 500; //ms
    buzzer = 1;
    //__delay_ms(500);
    //buzzer = 0;
    
    print_screen("DCBA Roll-up TJ", "-> Insert PCB & Start");  //waiting for pi to power up and run script 
    
    if(board_detect == 0) 
    { // if board not detected change this function. 2.6v on pin when no board, 1.8v when board present
        print_screen("Board","Detected");
    }
    else
    {
        print_screen("DCBA Roll-up TJ", "-> Insert PCB & Start");  //waiting for pi to power up and run script 
        while(1)
        {}
    }
    
//1. Power Board
    print_screen("Powering Board", "On AC 2");
    
    power_supply_k8 = 1; //switch AC2 input on 
    
    __delay_ms(150); //contact protection delay (crappy relays)
    
    power_supply_k7 = 1; //power board

    __delay_ms(2000); //power up delay
    
    power_supply_set(AC1); //switch back to AC1   
    
    __delay_ms(500); //contact protection delay (crappy relays)
    
//1b. Measure for motor short
    if((Opto_motor_dir_a == 0) || (Opto_motor_dir_b == 0))
    {
        print_error("Error", "Motor Short");
    }
    
//2. Measure Q Current 
    print_screen("Measuring", "Q Current");  

    ADC_val = get_current(); //Get mA (normal around 64mA) 
    
    if(ADC_val > 90) //> 90mA
    {
        print_error("Q Current", "Too High");
    }

//-- Measure Voltages
//3. Mesure V1
    print_screen("Measuring V1", "");  
    
    ADC_val = ADC_get_val(ADC_v1_test); //Get Voltage (calibrated 31.3v @ 719) factor = 04353v / inc 
    
    if(ADC_val < 693) //< 30.2v
    {//yes
        print_error("V1 Voltage-Jig", "Too Low");
    }
//    while(1)
//    {
//        ADC_val = ADC_get_val(ADC_v1_test); //Get Vol
//        lcd_print_int(ADC_val, 1, 0b00001000, 1);
//        __delay_ms(100);
//    }
    
//4. Mesure 24v
    print_screen("Measuring 24V", "");
    
    ADC_val = ADC_get_val(ADC_24v_test); //Get Voltage (calibrated 31.3v @ 719) factor = 04353v / inc 
    
    if(ADC_val < 533) //< 23.2v
    {//yes
        print_error("24V Voltage", "Too Low");
    }
    if(ADC_val > 572) //> 24.9v
    {//yes
        print_error("24V Voltage", "Too High");
    }
//    while(1)
//    {
//        ADC_val = ADC_get_val(ADC_24v_test); //Get Vol
//        lcd_print_int(ADC_val, 1, 0b00001000, 1);
//        __delay_ms(100);
//    }
    
//5. Mesure 5v
    print_screen("Measuring 5v", "");
    
    ADC_val = ADC_get_val(ADC_5v_test); //Get Voltage (calibrated 31.3v @ 719) factor = 04353v / inc 
    
    if(ADC_val < 110) //< 4.8v
    {//yes
        print_error("5V Voltage", "Too Low");
    }
    if(ADC_val > 133) //> 5.8v
    {//yes
        print_error("5V Voltage", "Too High");
    }
//    while(1)
//    {
//        ADC_val = ADC_get_val(ADC_5v_test); //Get Vol
//        lcd_print_int(ADC_val, 1, 0b00001000, 1);
//        __delay_ms(100);
//    }
    
//6. Mesure 3.3v
    print_screen("Measuring 3.3v", "");
    
    ADC_val = ADC_get_val(ADC_3v3_test); //Get Voltage (calibrated 3.32v @ 84) factor = v / inc 
    
    if(ADC_val < 73) //< 2.9v
    {//yes
        print_error("3.3V Voltage", "Too Low");
    }
    if(ADC_val > 94) //> 3.7v 93
    {//yes
        print_error("3.3V Voltage", "Too High");
    }
//    while(1)
//    {
//        ADC_val = ADC_get_val(ADC_3v3_test); //Get Vol
//        lcd_print_int(ADC_val, 1, 0b00001000, 1);
//        __delay_ms(200);
//    }
 
//7. Program Device 
    print_screen("Programming", "Bypassed");
     //__delay_ms(1000);
#ifdef PGM_Enable    
    print_screen("Programming..", "");
        
        pickit_state = pickit_start;
        
        while(1)
        {
            if(pickit_state == pickit_error)
            {
                print_error("Programming", "ERROR");
            } 
            
            if( pickit_state == pickit_idle)
            {
                break;
            }
        }
        
        print_screen("Programming", "Done");
        
#endif        
    __delay_ms(1000);   
        
//8. Get voltages from pcb   
    init_uart2(); //pi
    init_uart1(); //DUT
    //print_screen("UART Comms", "DUT Pinging");
    __delay_ms(300);
    
   //8.1  Test V1 voltage
    send_usart_packet_manage(128, 128); //get V1 voltage (574)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    //lcd_print_int(Data_Temp_Work, 1, 0b00001000, 1);
    //__delay_ms(2000);
    
    if(Data_Temp_Work_int > 622)  //>34V
    {
        print_error("V1 Voltage b", "Too High");
    }
    if(Data_Temp_Work_int < 553) //30.2
    {
        print_error("V1 Voltage b", "Too Low");
    }
    
//8.2  Test V2 voltage (Battery)
    
    //-- Link battery charger to sink
    power_supply_k12 = 1; 
    power_supply_k11 = 1;
    //--
    
    __delay_ms(200);
    
//    while(1)
//    {
//         ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage (factor is 627 @ 27.35v)
//         lcd_print_int(ADC_val, 1, 0b00001000, 1);
//         __delay_ms(1000);
//    }
    
    ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage (factor is 627 @ 27.35v)
    
    if(ADC_val > 646) //> 28.2v
    {
        print_error("Charge Err", "Voltage to High");
    }
      
    if(ADC_val < 618) //< 27v (was 623 27.2v )
    {
        print_error("Charge Err", "Voltage to Low");


//        while(1)
//        {
//             ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage (factor is 627 @ 27.35v)
//             lcd_print_int(ADC_val, 1, 0b00001000, 1);
//             __delay_ms(1000);
//        }
        
    }
    
    //-- Disconnect battery charger to sink
    power_supply_k12 = 0; 
    power_supply_k11 = 0;
    //--
    
    __delay_ms(200);
    
//    while(1)
//    {
//        send_usart_packet_manage(129, 129); //get V2 voltage (506)
//        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
//    
//        lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//        __delay_ms(1000);
//    }       
//        
        
    send_usart_packet_manage(129, 129); //get V2 voltage (499 @ 27.35V) 
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    if(Data_Temp_Work_int > 516) //28.2v
    {
        print_error("V2 SENSE", "Too High");
    }
    if(Data_Temp_Work_int < 495) //27.13v 
    {
        print_error("V2 SENSE", "Too Low");
    }
    //lcd_print_int(Data_Temp_Work, 1, 0b00001000, 1);
    //__delay_ms(2000);
    
//8.3  Test 24V voltage
    
//    while(1)
//    {
//        //23.1 - 25
//        send_usart_packet_manage(130, 130); //get 24V voltage (443)
//        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
//        lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//        __delay_ms(2000);
//    }
    
    
    send_usart_packet_manage(130, 130); //get 24V voltage (factor - 430 @ 23.827v)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    //lcd_print_int(Data_Temp_Work, 1, 0b00001000, 1);
    //__delay_ms(2000);
    
    if(Data_Temp_Work_int > 451) //25v 
    {
        print_error("24V SENSE", "Too High");
    }
    if(Data_Temp_Work_int < 416)//23.1
    {
        print_error("24V SENSE", "Too Low");
    }
    
//8.4  Test Beam voltage
    
    BM = 0;
    
    __delay_ms(300);
    
    send_usart_packet_manage(133, 133); //get beam in voltage no collector (1023)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //lcd_print_int(Data_Temp_Work, 1, 0b00001000, 1);
    //__delay_ms(2000);
    
    if(Data_Temp_Work_int < 1010)
    {
        print_error("Beam Input Err", "Not released");
    }
       
    BM = 1;
    
    __delay_ms(300);
    
    send_usart_packet_manage(133, 133); //get beam in voltage with collector (22)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    if(Data_Temp_Work_int > 28)
    {
        print_error("Beam Input Err", "V High");
    }
    if(Data_Temp_Work_int < 18)
    {
        print_error("Beam Input Err", "V Low");
    }
    
    BM = 0;
    
//9. Test I/o's  
    print_screen("Testing", "All io's high");
    
    send_usart_packet_manage(148, 148); //get all io inputs data bits
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //Data_Temp_Work_char = Data_Temp_Work_int && 127;// load char lower 7bits
//    
//    lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//    __delay_ms(2000);
    
    if(CHECK_BIT(Data_Temp_Work_int,0) != 1) //encoder high?
    {//no
        print_error("Error", "Encoder Input");
    }
    
    if(CHECK_BIT(Data_Temp_Work_int,1) != 1) //ecan rx input high?
    {//no
        print_error("Error", "Ecan Rx Input");
    }
        
    if(CHECK_BIT(Data_Temp_Work_int,2) != 1) // button up high?
    {//no
        print_error("Error", "Butt Up Input");
    }
    
    if(CHECK_BIT(Data_Temp_Work_int,3) != 1) // button down high?
    {//no
        print_error("Error", "Butt Down Input");
    }
// parts not populated    
//    if(CHECK_BIT(Data_Temp_Work_int,4) != 1) // fet sense input high?
//    {//no
//        print_error("Error", "Fet Sense Input");
//    }
    
    if(CHECK_BIT(Data_Temp_Work_int,5) != 1) // limit input high?
    {//no
        print_error("Error", "Limit Input");
    }
    
    if(CHECK_BIT(Data_Temp_Work_int,6) != 1) // bt input high?
    {//no
        print_error("Error", "Bt Input");
    }

//9.1 Test Bt Input
    //print_screen("Testing", "Bt Input");
    
    BT=1;
    
    __delay_ms(10);
    
    send_usart_packet_manage(148, 148); ////get all io inputs data bits
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
//    lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//    __delay_ms(3000);
    
    if(CHECK_BIT(Data_Temp_Work_int,6) != 0) // bt input high?
    {//no
        print_error("Error", "Bt Input");
    }
   
    BT=0; 
    
//9.2 Test Limit Input
    LIM = 1;
    
    __delay_ms(10);
    
    send_usart_packet_manage(148, 148); ////get all io inputs data bits
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
//    lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//    __delay_ms(3000);
    
    if(CHECK_BIT(Data_Temp_Work_int,5) != 0) // limit input high?
    {//no
        print_error("Error", "Limit Input");
    }
   
    LIM=0; 
    
//9.3 Test Encoder Input
    encoder = 1;
    
    __delay_ms(10);
    
    send_usart_packet_manage(148, 148); ////get all io inputs data bits
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
//    lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//    __delay_ms(3000);
    
    if(CHECK_BIT(Data_Temp_Work_int,0) != 0) //encoder high?
    {//no
        print_error("Error", "Encoder Input");
    }
   
    encoder = 0;    
    
//9.4 Test Ecan Rx Input
    __delay_ms(10);
    
    send_usart_packet_manage(134, 2); //set ecan low
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //Ecan = 1;
    
    __delay_ms(10);
    
    send_usart_packet_manage(148, 148); ////get all io inputs data bits
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
//    lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
//    __delay_ms(3000);

    if(CHECK_BIT(Data_Temp_Work_int,1) != 0) //ecan rx input high?
    {//no
        print_error("Error", "Ecan Rx Input");
    }
   
    send_usart_packet_manage(134, 1); //set ecan high
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    
//10 Test pcb Up button 
      
    print_screen("Press", "Up Button");
    
//WAIT FOR UP BUTTON PUSH
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,2) == 0) //UP button pushed?
        {//yes
            break;
        }
    }
    
    //WAIT FOR UP BUTTON RELEASE
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
   
        if(CHECK_BIT(Data_Temp_Work_int,2) == 1) //UP button released?
        {//yes
            break;
        }
    }
    
//11. Test pcb Down button 
    print_screen("Press", "Down Butt");
    
  //WAIT FOR DOWN BUTTON PUSH
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,3) == 0) //down button pushed?
        {//yes
            break;
        }
    }
    
    //WAIT FOR DOWN BUTTON RELEASE
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
   
        if(CHECK_BIT(Data_Temp_Work_int,3) == 1) //down button released?
        {//yes
            break;
        }
    }
    
//12. Check all leds
    print_screen("All Leds On?", "Press Down Butt");
    
    __delay_ms(10);
    send_usart_packet_manage(135, 2); //LED B ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(136, 2); //LED ENCODER ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(137, 2); //LED A ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(138, 2); //LED RF ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(143, 2); //LED MAIN ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(144, 2); //LED BEAM ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(145, 2); //LED 24V ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
     //__delay_ms(1000);
    
  //WAIT FOR DOWN BUTTON PUSH
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,3) == 0) //down button pushed?
        {//yes
            break;
        }
    }
  //  
  //WAIT FOR DOWN BUTTON RELEASE
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,3) == 1) //down button released?
        {//yes
            break;
        }
    }
  //  
    print_screen("All Leds Off?", "Press Down Butt");
    
    __delay_ms(10);
    send_usart_packet_manage(135, 1); //LED B OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(136, 1); //LED ENCODER OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(137, 1); //LED A OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(138, 1); //LED RF OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(143, 1); //LED MAIN OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(144, 1); //LED BEAM OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(10);
    send_usart_packet_manage(145, 1); //LED 24V OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //WAIT FOR DOWN BUTTON PUSH
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,3) == 0) //down button pushed?
        {//yes
            break;
        }
    }
    
    //WAIT FOR DOWN BUTTON RELEASE
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
  
        if(CHECK_BIT(Data_Temp_Work_int,3) == 1) //down button released?
        {//yes
            break;
        }
    }
    
//13. CHECK BUZZER
    print_screen("Buzz On?", "Press Down Butt");
    
    __delay_ms(10);
    send_usart_packet_manage(140, 2); //BUZZER ON
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
     //WAIT FOR DOWN BUTTON PUSH
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,3) == 0) //down button pushed?
        {//yes
            break;
        }
    }
    
    //WAIT FOR DOWN BUTTON RELEASE
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
  
        if(CHECK_BIT(Data_Temp_Work_int,3) == 1) //down button released?
        {//yes
            break;
        }
    }
    
     __delay_ms(10);
    send_usart_packet_manage(140, 1); //BUZZER OFF
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    print_screen("Buzz Off?", "Press Down Butt");
    
    //WAIT FOR DOWN BUTTON PUSH
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
        
        if(CHECK_BIT(Data_Temp_Work_int,3) == 0) //down button pushed?
        {//yes
            break;
        }
    }
    
    //WAIT FOR DOWN BUTTON RELEASE
    while(1)
    {
        __delay_ms(10);
        send_usart_packet_manage(148, 148); ////get all io inputs data bits
        Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
  
        if(CHECK_BIT(Data_Temp_Work_int,3) == 1) //down button released?
        {//yes
            break;
        }
    }

//13. CHECK BATTERY DISCONECT
    print_screen("Testing", "Battery Disc");
    __delay_ms(10);
    send_usart_packet_manage(141, 2); //SHUT DOWN CHARGER
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(2000);
    
    //TEST V2 VOLTAGE FOR DROP
    send_usart_packet_manage(129, 129); //get V2 voltage (506)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //lcd_print_int(Data_Temp_Work, 1, 0b00001000, 1);
    //__delay_ms(2000);
    
    if(Data_Temp_Work_int > 350)
    {
        print_error("Error", "Charge Disable");
    }
    
    __delay_ms(10);
    send_usart_packet_manage(141, 1); //ENABLE CHARGER
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
//13.CHECK MOTOR OUTPUTS
    print_screen("Testing", "Motor Circuit");
    
    __delay_ms(10);
    
//13.1 Get motor current sense voltage
    send_usart_packet_manage(132, 132); //Get Motor Current Sense Voltage No load (0)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
           
    if(Data_Temp_Work_int > 3)//is there per mature current flow?
    {//yes
        print_error("Error Motor", "Circuit Short");
    }
    
    //13.2 Switch On Motor + Relay
    __delay_ms(10);
    send_usart_packet_manage(146, 2); //Switch On Motor + Relay
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(500);
    
    //13.3 Get Motor Current Sense Voltage (0)
    send_usart_packet_manage(132, 132); //Get Motor Current Sense Voltage With Load (0)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    if(Data_Temp_Work_int > 2)//is there pre mature current flow?
    {//yes
        print_error("Error Motor", "Circuit Short");
    }
    
    //13.4 Switch On Pwm for Motor FET
    __delay_ms(10);
    send_usart_packet_manage(142, 254); //Switch On Pwm for FET (0-254max)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //print_screen("PWM ON", "");
    __delay_ms(1000);
    
    //13.5 Get Motor Current Sense Voltage With Load 
    send_usart_packet_manage(132, 132); //Get Motor Current Sense Voltage With Load (4 @ 110 Ohm)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    //lcd_print_int(Data_Temp_Work_int, 1, 0b00001000, 1);
    
    if(Data_Temp_Work_int < 2)//Is there motor current flow ?
    {//no
        print_error("Error Motor", "Circuit Open");
    }
    
//13.6 Check direction opto 1
    
    if((Opto_motor_dir_a == 0) && (Opto_motor_dir_b == 0))
    {
        print_error("Error Motor", "Circuit Err 1");
    }
    
    if((Opto_motor_dir_a == 0) && (Opto_motor_dir_b == 1))
    {
        print_error("Error Motor", "Circuit Err 2");
    }
    
    if((Opto_motor_dir_a == 1) && (Opto_motor_dir_b == 1))
    {
        print_error("Error Motor", "Circuit Err 3");
    }
    
//13.7 Switch relay+ Off & relay- On
    __delay_ms(10);
    send_usart_packet_manage(146, 1); //Switch Off Motor+ Relay
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    __delay_ms(100);
    send_usart_packet_manage(147, 2); //Switch On Motor- Relay
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    __delay_ms(250);
    
//13.8 Check direction opto 2
    
    if((Opto_motor_dir_a == 0) && (Opto_motor_dir_b == 0))
    {
        print_error("Error Motor", "Circuit Err 1");
    }
    
    if((Opto_motor_dir_a == 1) && (Opto_motor_dir_b == 0))
    {
        print_error("Error Motor", "Circuit Err 2");
    }
    
    if((Opto_motor_dir_a == 1) && (Opto_motor_dir_b == 1))
    {
        print_error("Error Motor", "Circuit Err 3");
    }
    
//13.9 Switch Off Pwm for Motor FET
    __delay_ms(10);
    send_usart_packet_manage(142, 0); //Switch Off Pwm for FET (0-254max)
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
     //13.10 //Switch Off Motor + Relay
    __delay_ms(100);
    send_usart_packet_manage(146, 1); //Switch Off Motor + Relay
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
      
//14. Test RF 
    print_screen("Testing", "Rx System");
    
    Remote_Tx = 1;
    __delay_ms(500);
    //Remote_Tx = 0;
    
    //__delay_ms(10);
    send_usart_packet_manage(252, 252); //Look for valid Et decryption of any remote.
    Data_Temp_Work_int = receive_data_bytes(2000); //ms - set timeout for first received byte + print error on timeout or data corruption
    
    if(Data_Temp_Work_int == 1)//Valid decryption ?
    {//no
        print_error("Error RF", "Timeout");
    }
         
    Remote_Tx = 0;
    
//15 Test Battery Charge Current
    print_screen("Testing Charger", "");
    
    //ADC_val = ADC_get_val(ADC_current_sink); //Get sink Voltage
    
    //ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage
    
    //Sink_Output = 0; //MAKE SURE SINK IS OFF!
    //-- Link battery charger to sink
    power_supply_k12 = 1; 
    power_supply_k11 = 1;
    //--
    
    __delay_ms(150);
    
    ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage (630 @ 27.4v factor = 0.04349mv / inc )
    
    if(ADC_val > 641) //> 27.9v
    {
        print_error("Charge Err", "Voltage to High");
    }
      
    if(ADC_val < 618) //< 27v
    {
        print_error("Charge Err", "Voltage to Low");
        
        
//        while(1)
//        {
//             ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage (factor is 627 @ 27.35v)
//             lcd_print_int(ADC_val, 1, 0b00001000, 1);
//             __delay_ms(1000);
//        }
        
        
    }
    //        ADC_val = ADC_get_val(ADC_v2_test); //Get Battery Voltage (630 @ 27.4v factor = 0.04349mv / inc )
    //        lcd_print_int(ADC_val, 1, 0b00001000, 1);
    CCPR9L = 254; //SINK PWM ON
    
    testjig_timer = 3000; //MS
    
    while(testjig_timer)
    {
        ADC_val = get_current(); //Get mA  
        
        if(ADC_val > 500) //> mA
        {
            CCPR9L = 0; //SINK PWM OFF
            print_error("Charger Err", "Curr too high");
        }
        
        lcd_print_int(ADC_val, 1, 0b00001000, 1);
    
        __delay_ms(100);
    }//LOOP
    
    if(ADC_val < 200) //> mA
    {
            CCPR9L = 0; //SINK PWM OFF
            print_error("Charger Err", "Curr too low");
    }
    
    CCPR9L = 0; //SINK PWM OFF
    __delay_ms(100); //was 100ms
    power_supply_k11 = 0;
    power_supply_k12 = 1;
    __delay_ms(400); //was 100ms
    
//16 Send testjig passed
    __delay_ms(10);
    //-- Reset local usart (Needed to do this after checking charger)
    RC2STAbits.SPEN = 0; 
    __delay_ms(1);
    RC2STAbits.SPEN = 1;
    //--
    send_usart_packet_manage(253, 253); //Send testjig pass to unit 
    Data_Temp_Work_int = receive_data_bytes(1000); //ms - set timeout for first received byte + print error on timeout or data corruption
    __delay_ms(500);
    
    print_screen("Test Pass", "");
    power_supply_set(NONE);
    
    buzz_timer = 500 ; buzzer = 1;
    while(buzzer == 1);
    __delay_ms(200);
    buzz_timer = 500 ; buzzer = 1;
    while(buzzer == 1);
    __delay_ms(200);
    buzz_timer = 1000 ; buzzer = 1; 
    
    while(1) //Wait for MCLR reset
    {}
    
    
    

//<editor-fold defaultstate="collapsed" desc="Testing Murray old">
//   
//    //8.1 init comms
//    init_uart2(); //pi
//    init_uart1(); //DUT
//    print_screen("UART Comms", "DUT Pinging");
//    DUT_comms(254);
//    
////    while(1){
////    unsigned int ab=0;
////      for (ab = 0; ab<5;ab++){
////          int data = uart_buff_work_recieve[ab];
////         lcd_print_int(ab,6,0,0);
////          lcd_print_int(data,12,0,0);
////          __delay_ms(500);
////      }
////    }
////Motor Testing//////////////////////////////////////////////////////////// 
//    /*
//    //set dirn via relays (146 and 147)
//    uart_reset();
//    set_rollup(146,1); //m+ relay off
//    
//    uart_reset();
//    set_rollup(147,2); //m- relay on
//
//    //no current no voltage
//    //get FET_sense
//    testjig_timer=200;
//    DUT_comms(148);
//    if (CHECK_BIT(uart_buff_work_recieve[2], FET_SENSE)==0) //(int to be checked, bit position in number)  //gives 1 when fet is not on, default. //give 0 when it is closed in error
//    {
//        print_error("Q10 - Motor Fet"," or F1 Error");
//    }
//    
//    //get motor current 132, must be 0
//    testjig_timer=200;
//    current_actual=DUT_comms(132);
//    
//    if (current_actual>100)//
//    {   print_error("Motor Current","Detected");
//    }
//    
//    //set different speeds (pwm val0-129 w/ cmd 142)
//    set_rollup(142,128);
//    
//    //check motor current 132 ~ what is the acceptable range?
//    testjig_timer=200;
//    current_actual=DUT_comms(132);
//    lcd_print_int(voltage_outcome, 10, 0, 0);
//    __delay_ms(5000);
//    if (current_actual<0)//
//    {   print_error("Below Req","Motor Current");
//    }
//    if (current_actual>1)//
//    {   print_error("Above Req","Motor Current");
//    }
//    
//    //pwm to 0
//    set_rollup(142,0);
//    
//    //reverse dirn via relays 50ms
//    set_rollup(146,2); //m+ relay on
//    set_rollup(147,1); //m- relay off
//    
//    //set different speeds (pwm val0-129 w/ cmd 142)
//    set_rollup(142,128);
//            
//    //check current 132 ~ what is the acceptable range??
//    if (current_actual<0)
//    {   print_error("Below Req","Motor Current");
//    }
//    if (current_actual>1)//
//    {   print_error("Above Req","Motor Current");
//    }   
//    
//    //pwm to 0
//    set_rollup(142,0);
//    
//    //etc
//
//    //print_error("End of Test","");
//      */
//        
//////////////////////////////////////////////////////////////////////////////////
////////////see commands_1 in S drive testjig folder to clarify callsigns/////////   
//////////////////////////////////////////////////////////////////////////////////    
//    
////8.2 Ping Voltage Monitors
//    //V1 monitor - +-31v
//    testjig_timer=200;
//    voltage_actual=DUT_comms(128);
//    voltage_outcome=DUT_voltage_check(voltage_actual,31);
////    lcd_print_int(voltage_outcome, 1, 0, 0);   
////    __delay_ms(1000);
//
//    if (voltage_outcome==0)
//    {   print_error("Below Req","31v");
//    }
//    if (voltage_outcome==1)
//    {   print_error("Above Req","31v");
//    }
//
//    //V2 monitor - +-27v
//    testjig_timer=200;
//    voltage_actual=DUT_comms(129); 
//    voltage_outcome=DUT_voltage_check(voltage_actual,27);
//    if (voltage_outcome==0)
//    {   print_error("Below Req","27v");
//    }
//    if (voltage_outcome==1)
//    {   print_error("Above Req","27v");
//    }
//
//    //24v monitor - +-24v
//    testjig_timer=200;
//    voltage_actual=DUT_comms(130); 
//    voltage_outcome=DUT_voltage_check(voltage_actual,24);
//    if (voltage_outcome==0)
//    {   print_error("Below Req","24");
//    }
//    if (voltage_outcome==1)
//    {   print_error("Above Req","24");
//    }
//    
//    //get_rollup(131);
//    //voltage_f = ADC_to_voltage(uart_buff_work_recieve[1]*256+uart_buff_work_recieve[2]);
//    //lcd_print_int(voltage_i, 10, 0b00000100, 1);
//////////////////////////////////////////////////////////////////////////////////
//
////8.3 check analog beam in val
////beam in and 1k across it to gnd
//    BM=0;
//    __delay_ms(200);
//    testjig_timer=200;
//    voltage_actual=DUT_comms(133);
//    if (voltage_actual<4000)
//    {
//       print_error("Beam In","Components missing"); 
//    }
//    
//    BM=1;
//    __delay_ms(200);
//    testjig_timer=200;
//    voltage_actual=DUT_comms(133);
//    if (voltage_actual>4000)
//    {
//       print_error("Beam In","Not Detected"); 
//    } 
//    
////8.4 Battery/ Charger testing before v2 check ~ confusion with the two different inputs /////////// ////////////////////////
//    //disable the charger via bat charge disable
//    uart_reset();
//    set_rollup(141, 2); //turn on Q7 on DUT to gnd U3's ADJ pin
//    
//    //check voltage drops v2
//    testjig_timer=200;
//    voltage_actual=DUT_comms(129);
//    if (voltage_actual>1000){
//        print_error("Battery charger","Disable Error");
//    }
//
//    //enable charger
//    uart_reset();
//    set_rollup(141, 1);
//    
//    //check voltage again resumes hi
//    testjig_timer=200;
//    voltage_actual=DUT_comms(129);
//
//    if (voltage_actual<20000){
//        print_error("Battery charger","Enable Error");
//    }
//    
//    //bat disconnect
//    uart_reset();
//    set_rollup(139,2);
//    
//    //v2 (129) vs 24v (130) check
//    testjig_timer=200;
//    voltage_actual=DUT_comms(129);
//    //what other steps are required to completely check the battery charger etc////////////////////////////////////////////////////////////
//    //tmp digital potentiometer and resistor change necessary to drop v1 below 18v // p12?
//    //switching the 
//    //lcd_print_int(voltage_actual, 9, 0, 0);
//    __delay_ms(200);
//    
////8.5 STATUS CHECK -- send 148(all pin status, give 2byte response, each bit representing a pin) to check status of pins/ inputs
//    //Button UP and LED A & B check
//    testjig_timer=5000;
//    print_screen("A & B LEDs on?","Press 'A'");
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], BUTTON_UP)==0) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("No 'A' Button","Push Detected");
//    }
//
//    //Button DOWN and Buzzer & Lights check
//    uart_reset();
//    set_rollup(140,2);
//    uart_reset();
//    set_rollup(143,2);
//
//    print_screen("Lights & Buzzer on?","Press 'B'");
//    testjig_timer=5000;
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //no timer because in loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], BUTTON_DOWN)==0) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("No 'B' Button","Push Detected");
//    }
//
//    //turn off lights and buzzer
//    uart_reset();
//    set_rollup(140,1);
//    uart_reset();
//    set_rollup(143,1);
//
////8.6 Check can tx rx//////////////////////////////////////////////////////////////
//    //can tx off 0
//    uart_reset();
//    set_rollup(134,1);
//    testjig_timer=2000;
//    //pol ecan rx must = 1
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], ECAN_RX)==1) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("Q8 not operating","as expected (1)");
//    }
//
//    //check rx on ecan by toggling collector on data con
//    //this must be added to make the test jig complete, I have left out as I ran out of collectors
//    //this part of the test only checks if the connector and R32 are present
//    
//    //switch ecan tx to 1 via q8
//    uart_reset();
//    set_rollup(134,2);
//    
//    //ecan rx should be 0
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], ECAN_RX)==0) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("Q8 not operating","as expected (2)");
//    }
//    
////8.7 encoder check
//
//    //enc should be high
//    testjig_timer=200;
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], ENC)==1) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("Encoder Not High","Error");
//    }
//    
//    //enc should be low when pulled low by collector
//    encoder=1;
//    __delay_ms(200);
//    testjig_timer=200;
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], ENC)==0) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("Encoder not low","Error ");
//    }
//   
//    
////8.8 check lim and bt in
//    testjig_timer=5000;
//    //print_screen("Lim","Test");
//    
//    testjig_timer=5000;
//    print_screen("BT","Check");
//    BT=0;
//    __delay_ms(200);
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], BT_IN)==1) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//    
//    if (testjig_timer==0)
//    {
//        print_error("BT Not","Held High");
//    }
//
//    testjig_timer=5000;
//    BT=1;
//    __delay_ms(200);
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], BT_IN)==0) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//    
//    if (testjig_timer==0)
//    {
//        print_error("BT Not","Held Low");
//    }
//    
//    
//    testjig_timer=5000;
//    print_screen("LIM","Test");
//    BT=0;
//    LIM=0;
//    __delay_ms(200);
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], LIM_IN)==1) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("Lim not","Held High");
//    }
//    
//    LIM=1;
//    while(testjig_timer)
//    {
//        __delay_ms(50);
//        DUT_comms(148); //not given testjig timer because inside loop
//        if (CHECK_BIT(uart_buff_work_recieve[2], LIM_IN)==0) //(int to be checked, bit position in number)
//        {
//            break;
//        }
//    }
//
//    if (testjig_timer==0)
//    {
//        print_error("Lim not","Held Low");
//    }
//    
////8.9 RF Testing/////////////////////////////////////////////////////////////// put this at the end of the test
////    testjig_timer=4000;
////    print_screen("RF","Check");
////    while(testjig_timer){
////        remote=1;
////        DUT_comms(252); //no timer in loop
////
////        if (uart_buff_work_recieve[2]==2){
////            break;
////        }
////    }
////    
////    if (testjig_timer==0){
////        print_error("RF Valid Decryption","Test Failed");
////    }
////    
////    remote=0;
//    
//
////8.10 takes jig out of test mode
//    DUT_comms(253);
//    
//
////check what is coming out of P27 which is next to fuse by 24v aux  
////send test passed 253 and check echo that board is programmed and tested
    
// </editor-fold>     
}

void testjig_done(unsigned char state, unsigned char device_type) // must edit code for Micro pass
{
    while (1) {
        //power_supply_set(AC1);
        //CH2_high_current_K13_P10 = 0;
        //CH3_high_current_K6_P13 = 0;
        debug_fast_tx(11110);
        debug_fast_tx(state);
        switch (state) {
            case 1://pass
                if (device_type == Device_Condo3)
                    print_screen("Condo3 Pass", " Remove board");
                else if (device_type == Device_RX3)
                    print_screen("RX3 Pass", " Remove board");
                else if (device_type == Device_Mem_Copier)
                    print_screen("Mem_Copier Pass", " Remove board");
                else if (device_type == DCBA_Rollup)
                    print_screen("DCBA Roll-up Pass", " Remove board");
                else 
                    print_screen("Test Pass", " Remove board");
                debug_fast_tx(11111);
                buzz_one(100);
                __delay_ms(200);
                buzz_one(100);
                
                testjig_timer=10000;
                while (testjig_timer>0);
                print_error("Insert New Board","Then Press Start");
                
          
                //detects board removal
                while(1){
                    __delay_ms(50);
                    if(board_detect==1) {
                        print_screen("Insert New Board","Then Press Start");
                        power_supply_set(NONE);
                    }
                }
                

                __delay_ms(100);
                state = 0xFF;
                break;
            case 2://start up
                debug_fast_tx(11112);

                if (device_type == Device_Condo3)
                    print_screen("Condo3 ", " Testjig");
                else if (device_type == Device_RX3)
                    print_screen("RX3 ", " Testjig");
                else if (device_type == Device_Mem_Copier)
                    print_screen("Mem_Copier Pass", " Testjig");
                else
                    print_screen("DCBA Roll-up", "Testjig");
                //__delay_ms(2000);
                //print_screen("Testjig Power-up", "wait...");
                //__delay_ms(500);
                state = 0xFF;
                break;
        }
        debug_fast_tx(11113);
        if (device_type == Device_Condo3)
            print_screen("Condo3 Testjig", "-> Insert PCB");
        else if (device_type == Device_RX3)
            print_screen("RX3 Testjig", "-> Insert PCB");
        else if (device_type == Device_Mem_Copier)
            print_screen("Mem_Copier Pass", "-> Insert PCB");
        else
            print_screen("DCBA Roll-up TJ", "-> Insert PCB");

         break;
    }
}

void buzz_one(unsigned int time) 
{
#ifdef Buzz_disable
    buzz_timer = time;
    buzzer = 1;
#endif
}

// <editor-fold defaultstate="collapsed" desc="LCD Functions : Functions that print things to the LCD">
#define test_bit(var,bitno) (var & (1UL << bitno))
#define bitset(var, bitno) ((var) |= 1UL << (bitno))

void i2c_Start(void) {
    //SEN = 1;         /* Start condition enabled */
    SSP1CON2bits.SEN = 1;
    // while(SEN);      /* automatically cleared by hardware */
    while (SSP1CON2bits.SEN == 1);
    /* wait for start condition to finish */
}

void i2c_Write(unsigned char data) {
    SSP1BUF = data; /* Move data to SSPBUF */
    while (SSP1STATbits.BF == 1); /* wait till complete data is sent from buffer */
    while ((SSP1CON2 & 0x1F) || (SSP1STAT & 0x04)); /* wait for any transfer */
}

void i2c_Stop(void) {
    SSP1CON2bits.PEN = 1;
    while (SSP1CON2bits.PEN == 1);
    /* PEN automatically cleared by hardware */
}

void i2c_Write_string(const unsigned char *data) {
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x40); // data write to display RAM follows...

    while (*data != NULL) //NULL('\0')
    {
        i2c_Write(*data);
        data++;
    }
    i2c_Stop();
}

void i2c_write_char(unsigned char char_data) {
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x40); // data write to display RAM follows...
    i2c_Write(char_data);
    i2c_Stop();
    //    __delay_us(27);
}

void init_i2c_lcd_5v(unsigned char contrast, unsigned char single1_double2) //contrast between 0 and 15.
{
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00); //command mode...
    i2c_Write(0x39); //0, 0, function set, 8-bit, 2-line, normal height, extended instruction set
    i2c_Write(0x1C); //0, 0, 0, int osc frequency/bias select, bias=1/4, Oscilator = 192Hz
    i2c_Write(0x70 | contrast); //0111 (contrast set), Contrast low bits=1000
    i2c_Write(0x50); //0101 (power/icon/contrast), Icon display off, booster circuit off, contrast high bits=00
    i2c_Write(0x6C); //0110 (follower control), internal follower on, follower bits = 100
    if (single1_double2 == 1)
        i2c_Write(0x38); //0, 0, function set, 8-bit, 2-line, normal height, normal instruction set
    if (single1_double2 == 2)
        i2c_Write(0x34); //0, 0, function set, 8-bit, 2-line, normal height, normal instruction set
    i2c_Write(0x0C); //0,0,0,0,display on/off, Display on, curser off, blink off
    i2c_Write(0x06); //0,0,0,0,0, Entry Mode Set, Increment (print right), Don't shift Entire display
    i2c_Write(0x01); //clear display
    __delay_ms(2); //clear display delay
    i2c_Stop();
}

//void init_i2c_lcd_3v3(unsigned char contrast, unsigned char single0x38_double0x34) //contrast between 0 and 15. //in bootloader page, not updateable
//{
//        i2c_Start();
//    i2c_Write(i2c_device); //0x7C slave address
//    i2c_Write(0x00); //command mode...
//    i2c_Write(0x39); //0, 0, function set, 8-bit, 2-line, normal height, extended instruction set
//    i2c_Write(0x1C); //0, 0, 0, int osc frequency/bias select, bias=1/4, Oscilator = 277Hz
//    i2c_Write(0x70 | contrast); //0111 (contrast set), Contrast low bits=1000
//    char cont_high = 0x54 | ((contrast & 0b00110000) >> 4);
//    i2c_Write(cont_high); //0101 (power/icon/contrast), Icon display off, booster circuit off, contrast high bits=00
//    i2c_Write(0x6C); //0110 (follower control), internal follower on, follower bits = 100
//    //    if (single0x38_double0x34 == 1)
//    i2c_Write(single0x38_double0x34); //0, 0, function set, 8-bit, 2-line, normal height, normal instruction set
//    //    if (single0x38_double0x34 == 2)
//    //        i2c_Write(0x34); //0, 0, function set, 8-bit, 2-line, normal height, normal instruction set
//    i2c_Write(0x0C); //0,0,0,0,display on/off, Display on, curser off, blink off
//    i2c_Write(0x06); //0,0,0,0,0, Entry Mode Set, Increment (print right), Don't shift Entire display
//    i2c_Write(0x01); //clear display
//    unsigned int i;
//    for(i = 1116; i; i--); //500us delay
//    i2c_Stop();
//}

void clear_lcd(void) {
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00);
    i2c_Write(0x01); //clear display
    __delay_ms(2);
    i2c_Stop();
}

void i2c_first_line(void) {
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00);
    i2c_Write(0x80); //80 = first line, C0 = second line
    __delay_us(27);
    i2c_Stop();
}

void i2c_second_line(void) {
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00);
    i2c_Write(0xC0); //80 = first line, C0 = second line
    __delay_us(27);
    i2c_Stop();
}

void lcd_set_cursor(unsigned char row, unsigned char col) //Set the cursor to x(row) y(col) position
{
    unsigned char LCD_position = 1;
    col--;
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00); //command mode...
    LCD_position = col & 0b11001111;
    if (row == 1)
        LCD_position |= 0b10000000;
    else
        LCD_position |= 0b11000000;
    i2c_Write(LCD_position); //80 = first line, C0 = second line  (DDRAM Adress)
    i2c_Stop();
}


// <editor-fold defaultstate="collapsed" desc="Unused LCD function">

void lcd_blink_cursor(void) //Make the LCD cursor blink
{
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00); //command mode...
    i2c_Write(0x0F); //Blink cursor
    i2c_Stop();
}

//void lcd_cursor_OFF(void)                    //Make the LCD cursor blink
//{
//    i2c_Start();
//    i2c_Write(i2c_device);      //0x7C slave address
//    i2c_Write(0x00);            //command mode...
//    i2c_Write(0x0C);            //Blink cursor
//    i2c_Stop();
//}
//
//void lcd_cursor_ON(void)                    //Make the LCD cursor blink
//{
//    i2c_Start();
//    i2c_Write(i2c_device);      //0x7C slave address
//    i2c_Write(0x00);            //command mode...
//    i2c_Write(0x0E);            //Blink cursor
//    i2c_Stop();
//}
//
//void lcd_home (void)
//{
//                i2c_Start();
//                i2c_Write(i2c_device);      //0x7C slave address
//                i2c_Write(0x00);            //command mode...
//                i2c_Write(0x02);
//                i2c_Stop();
//                __delay_us(1100);           //wait for LCD to complete instruction
//}
//
//void lcd_scroll_left (void)
//{
//    i2c_Start();
//    i2c_Write(i2c_device);      //0x7C slave address
//    i2c_Write(0x00);            //command mode...
//    i2c_Write(0x18);
//    i2c_Stop();
//    __delay_us(27);
//}
//
//void lcd_scroll_right (void)
//{
//                i2c_Start();
//                i2c_Write(i2c_device);      //0x7C slave address
//                i2c_Write(0x00);            //command mode...
//                i2c_Write(0x1C);
//                i2c_Stop();
//                __delay_us(27);
//}
//
//void binary_print(unsigned char data)
//{
//    if (data&0b10000000)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b01000000)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b00100000)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b00010000)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b00001000)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b00000100)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b00000010)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//    if (data&0b00000001)
//        i2c_write_char (49);
//    else
//        i2c_write_char (48);
//}
//void debug_flash(unsigned char start_ptr)       //a function that prints a flash page to the LCD. for debugging only
//{
//    while (any_button_pressed());
//    unsigned char data;
//    lcd_set_cursor(1, 1);
//    for (char x = 0; x < 16; x++)
//    {
//        if (x == 8)
//            lcd_set_cursor(2, 1);
//        data = read_flash_1(start_ptr + x);
//        data = (data & 0xF0) >> 4;
//        //        i2c_write_char (data + 48);
//        if (data < 10)
//            i2c_write_char (data + 48);
//        else i2c_write_char (data + 55);
//
//        data = read_flash_1(start_ptr + x);
//        data = (data & 0x0F);
//        if (data < 10)
//            i2c_write_char (data + 48);
//        else i2c_write_char (data + 55);
//    }
//    while (any_button_pressed());
//    while (right_button == 1 && exit_button == 1);
//    while (exit_button == 1)
//    {
//        lcd_set_cursor(1, 1);
//        for (char y = 0; y < 16; y++)
//        {
//            if (y == 8)
//                lcd_set_cursor(2, 1);
//            data = read_flash_1(start_ptr + y + 16);
//            data = (data & 0xF0) >> 4;
//            if (data < 10)
//                i2c_write_char (data + 48);
//            else i2c_write_char (data + 55);
//
//            data = read_flash_1(start_ptr + y + 16);
//            data = (data & 0x0F);
//            if (data < 10)
//                i2c_write_char (data + 48);
//            else i2c_write_char (data + 55);
//        }
//    }
//}
//
//
//void lcd_scroll_double(unsigned char end, unsigned char dir)           //a routine to scroll the display 'end' positions, dir=0 -> Left | dir=1 ->right
//{
//    for (unsigned char i = 0; i !=100; i++)
//    {
//        if (left_button == 1 && right_button == 1 && exit_button == 1 && set_button == 1)
//            __delay_ms(4);                  //This should be less than 10ms for sensitivity
//    }
//    if (dir == 0)
//    {
//        lcd_scroll_left();
//        lcd_scroll_left();
//    } else
//    {
//        lcd_scroll_right();
//        lcd_scroll_right();
//    }
//    scroll_count++;
//    if (scroll_count == end||scroll_count==0)
//    {
//        for (unsigned char i = 0; 1!=200; i++)
//        {
//            if (left_button == 1 && right_button == 1 && exit_button == 1 && set_button == 1)
//                __delay_ms(4);                  //This should be less than 10ms for sensitivity
//        }
//        lcd_home();
//        scroll_count = 0;
//    }
//}
//
//void lcd_scroll(unsigned char end, unsigned char dir)           //a routine to scroll the display 'end' positions, dir=0 -> Left | dir=1 ->right
//{
//        for (unsigned char i = 0; i != 100; i++)
//    {
//        if (left_button == 1 && right_button == 1 && exit_button == 1 && set_button == 1)
//            __delay_ms(4);
//    }
//    if (dir == 0)
//        lcd_scroll_left();
//    else
//        lcd_scroll_right();
//    scroll_count++;
//    if (scroll_count == end)
//    {
//        lcd_home();
//        scroll_count = 0;
//    }
//    for (unsigned char i = 0; i !=100; i++)
//    {
//        if (left_button == 1 && right_button == 1 && exit_button == 1 && set_button == 1)
//            __delay_ms(4);
//    }
//}


// </editor-fold>

//void contrast_adj(void)                         // a function to adjust the contrast by calling contrast_set().
//{
//    unsigned char contrast = (0x80 & 0b11110000) >> 4;
//    print_screen("Set Contrast", "<              >");               // less memory to print spaces than print on new line
//    button_beep(button_beep_delay);
//    while (set_button == 0);                                    //wait for button release
//    while (set_button == 1 && exit_button == 1)
//    {
//        contrast = button_wrap_around(contrast, 0, 15, 1);      //count, min, max, wrap
//        lcd_set_cursor(2, 8);
//        ten_thou_conv(contrast);         //This can be out by as much as 11% due to resistor tolerance.
//        i2c_write_char (ten + 48);
//        i2c_write_char (unit + 48);
//        contrast_set(contrast);
//        flash_write_1(contrast << 4, MEM_contrast_setting); //load - (data, location in flash frame)
//    }
//    if (exit_button == 0)
//    {
//        button_beep(button_beep_delay * 64);
//        contrast_set(0x08);                                     //set default contrast
//        popup("Constrast set", "Exited");
//    } else save_beep();
//    while (set_button == 0);                                    //wait for button release
//}

void contrast_set(unsigned char contrast) //a function to set the contrast to the parsed char
{
    char cont_low = 0x70;
    char cont_high = 0x50;
    cont_low |= contrast;
    cont_high |= (contrast & 0b00110000) >> 4;
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00); //command mode...
    i2c_Write(0x39); //0, 0, function set, 8-bit, 2-line, normal height, extended instruction set
    i2c_Write(cont_low); //0111 (contrast set), Contrast low bits=1000
    i2c_Write(cont_high); //0101 (power/icon/contrast), Icon display off, booster circuit off, contrast high bits=00
    i2c_Write(0x38); //0, 0, function set, 8-bit, 2-line, normal height, normal instruction set
    i2c_Stop();
}

void ten_thou_conv(int n) //a function to calculate 5 digits quickly
{//about 200us slower that 'hund_conv', much more efficient than 'decimal_conv'
    unit = n & 0x1F;
    ten = (n >> 5) & 0xF;
    hund = (n >> 9) & 0xF;
    thou = (n >> 13) & 0x7;
    //calculate unit
    unit = 2 * (thou + hund + ten) + unit;
    ten_thou = (unit * 0x67) >> 10;
    unit = unit - 10 * ten_thou;
    //calculate ten
    ten = 9 * thou + hund + 3 * ten + ten_thou;
    ten_thou = (ten * 0x67) >> 10;
    ten = ten - 10 * ten_thou;
    //calculate hund
    hund = thou + 5 * hund + ten_thou;
    ten_thou = (hund * 0x67) >> 10;
    hund = hund - 10 * ten_thou;
    //calculate thou
    thou = 8 * thou + ten_thou;
    ten_thou = (thou * 0x1A) >> 8;
    thou = thou - 10 * ten_thou;
}

void hund_conv(int n) //a function to calculate 3 digits quickly
{
    unit = n & 0x1F;
    ten = (n >> 5) & 0xF;
    hund = (n >> 9) & 0xF;
    thou = (n >> 13) & 0x7;
    //calculate unit
    unit = 2 * (thou + hund + ten) + unit;
    ten_thou = (unit * 0x67) >> 10;
    unit = unit - 10 * ten_thou;
    //calculate ten
    ten = 9 * thou + hund + 3 * ten + ten_thou;
    ten_thou = (ten * 0x67) >> 10;
    ten = ten - 10 * ten_thou;
    //calculate hund
    hund = thou + 5 * hund + ten_thou;
    ten_thou = (hund * 0x67) >> 10;
    hund = hund - 10 * ten_thou;
}

//void popup(const unsigned char *top, const unsigned char *bottom)               //A function to display 'top' and 'bottom' on the LCD for 'pop_up delay' ms
//{
//    // if print_screen() is called here
//    clear_lcd();
//    i2c_first_line();
//    i2c_Write_string(top);
//    i2c_second_line();
//    i2c_Write_string(bottom);
//    while (any_button_pressed());               // wait for all buttons to be released.
//    for (char x = 100; x != 0; x--)             //Loop that only delays if a button is not pressed
//        if (!any_button_pressed())              // if no button is pressed.
//            delay_ms(popup_delay);
//    if (any_button_pressed())
//        button_beep(button_beep_delay);
//    while (any_button_pressed());               // wait for all buttons to be released.
//    clear_lcd();
//}

//void popup_elipses(const unsigned char *top, const unsigned char *bottom, unsigned char elipses1)       // this is a cool feature to pretend that the micro is busy. but the Elipses should rather be implimented in the funtion that it is 'pretending' to do.
//{
//    // if print_screen() is called here
//    clear_lcd();
//    i2c_first_line();
//    i2c_Write_string(top);
//    i2c_second_line();
//    i2c_Write_string(bottom);
//    while (any_button_pressed());               // wait for all buttons to be released.
//    for (char x = 100; x != 0; x--)             //Loop that only delays if a button is not pressed
//    {
//        if (!any_button_pressed())              // if no button is pressed.
//        {
//            delay_ms(popup_delay);
//            //            if (elipses1)
//            if (elipses1 && (x == 25 || x == 50 || x == 75 ))
//                i2c_write_char ('.');
//        }
//    }
//    if (any_button_pressed())
//        button_beep(button_beep_delay);
//    while (any_button_pressed());               // wait for all buttons to be released.
//    clear_lcd();
//}

void print_error(const unsigned char *top, const unsigned char *bottom) //A function to display 'top' and 'bottom' on the LCD perminanently
{
    CCPR9L = 0; //SINK PWM OFF
    power_supply_set(NONE);
//    CH2_high_current_K13_P10 = 0;
//    CH3_high_current_K6_P13 = 0;
    RF_out = 0;
    debug_fast_tx(12345);
    buzz_one(1000);
    while (1) 
    {
        clear_lcd();
        i2c_first_line();
        i2c_Write_string(top);
        i2c_second_line();
        i2c_Write_string(bottom);
        __delay_ms(2000);
        print_screen("Press START", " to restart test");
        //        lcd_print_int(trip_current, 1, 0, 0);
        __delay_ms(1200);
    }
}

void print_screen(const unsigned char *top, const unsigned char *bottom) //A function to display 'top' and 'bottom' on the LCD perminanently
{
    clear_lcd();
    i2c_first_line();
    i2c_Write_string(top);
    i2c_second_line();
    i2c_Write_string(bottom);
}

void print_second_line(const unsigned char *bottom) //A function to display 'top' and 'bottom' on the LCD perminanently
{
    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x00); //command mode...
    i2c_Write(0b11000000); //80 = first line, C0 = second line  (DDRAM Adress)
    i2c_Stop();

    i2c_Start();
    i2c_Write(i2c_lcd_address); //0x7C slave address
    i2c_Write(0x40); // data write to display RAM follows...
    for (unsigned char x = 0; x != 16; x++) {
        if (*bottom != NULL) //NULL('\0')
        {
            i2c_Write(*bottom);
            bottom++;
        } else
            i2c_Write(0x20); //print blank
    }
    i2c_Stop();
}

void lcd_bar_create(void) //function to create characters with number of lines equal to position+1. (for bar)
{
    unsigned char bar_one[8] = {
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b11111
    };
    lcd_createChar(0, bar_one);

    unsigned char bar_two[8] = {
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b11111,
        0b11111
    };
    lcd_createChar(1, bar_two);

    unsigned char bar_three[8] = {
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b11111,
        0b11111,
        0b11111
    };
    lcd_createChar(2, bar_three);

    unsigned char bar_four[8] = {
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
    lcd_createChar(3, bar_four);

    unsigned char bar_five[8] = {
        0b00000,
        0b00000,
        0b00000,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
    lcd_createChar(4, bar_five);

    unsigned char bar_six[8] = {
        0b00000,
        0b00000,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
    lcd_createChar(5, bar_six);

    unsigned char bar_seven[8] = {
        0b00000,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
    lcd_createChar(6, bar_seven);

    unsigned char bar_eight[8] = {
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };
    lcd_createChar(7, bar_eight);
}

void lcd_print_bar(unsigned char num) // a function to print a bar of 'num' value. 'size' sets the width, either 8 or 16 blocks
{
    for (int i = 0; i != num + 1; i++) {
        i2c_write_char(i);
    }
}

void lcd_createChar(unsigned char location, unsigned char charmap[]) //a function to create a character at a specific location given a character map
{
    location &= 0x07; // we only have 8 locations 0-7
    i2c_Start();
    i2c_Write(i2c_lcd_address);
    i2c_Write(0x00); //command mode
    i2c_Write(0x40 | (location << 3)); //set location
    i2c_Stop();
    __delay_us(27); // >26.3us
    for (int i = 0; i != 8; i++) {
        i2c_write_char(charmap[i]); //write data sequentially
    }
    clear_lcd();
}

//void flash_created_character_zero (unsigned char pos)                                       // this function flashes the character in position '0' every 1s.
//{
//    lcd_set_cursor(2, pos);
//    i2c_write_char(32);                      //print blank
//    while (set_button == 1 && exit_button == 1 && right_button == 1  && x != 100)     //off for 100*5ms. also checking buttons
//    {
//        __delay_ms(5);
//        x++;
//    }
//    x = 0;
//    lcd_set_cursor(2, pos);
//    i2c_write_char(0);                      //print blank
//    while (set_button == 1 && exit_button == 1 && right_button == 1 && x != 100)     //on for 100*5ms. also checking buttons
//    {
//        __delay_ms(5);
//        x++;
//    }
//    x = 0;
//}

//void page_down_sunroutine(const unsigned char *page11, const unsigned char *page12, const unsigned char *page21, const unsigned char *page22)  //function to print flashing arrow and page down on RIGHT, skip on SET
//{
//    lcd_createChar(0, down_arrow);
//    print_screen(page11, page12);
//    while (set_button == 0);
//    while (set_button == 1 && exit_button == 1 && right_button == 1)
//    {
//        flash_created_character_zero (16);                       //flash a down arrow at position (pos)
//    }
//    if (right_button == 0)
//    {
//        button_beep(button_beep_delay);
//        print_screen(page21, page22);
//    }
//    while (right_button == 0);
//    while (set_button == 1 && exit_button == 1);
//    button_beep(button_beep_delay);
//    while (set_button == 0);                        //wait for set button to be pressed
//}

//void print_number(unsigned int number, unsigned int pos, unsigned char decimal_delay)            //prints 3 digit number at given LCD position can popup and set decimal
//{
//    lcd_set_cursor(2, pos);
//    ten_thou_conv(number);
//
//    if (hund != 0 || decimal_delay & 0x04)                                      //do not print leading 100s '0'.
//        i2c_write_char(hund + 48);
//    else i2c_write_char(32);
//    if (decimal_delay & 0x04)i2c_write_char ('.');
//    i2c_write_char(ten + 48);
//    if (decimal_delay & 0x02)i2c_write_char ('.');
//    i2c_write_char(unit + 48);
//}

void lcd_print_int(unsigned int number, unsigned int pos, unsigned char decimal, unsigned char ten_thou_digit) //prints 3 digit number at given LCD position can popup and set decimal
{
    lcd_set_cursor(2, pos + 1 - ten_thou_digit); //adjust position by one if digit is not being printed.
    ten_thou_conv(number);
    if (ten_thou_digit) 
    {
        if (ten_thou != 0 || decimal & 0x10) //do not print leading 100s '0'.
            i2c_write_char(ten_thou + 48);
        else i2c_write_char(' ');
    }
    if (decimal & 0x10)i2c_write_char('.');
    if (thou != 0 || ten_thou != 0 || decimal & 0x18) //do not print leading 100s '0'.
        i2c_write_char(thou + 48);
    else i2c_write_char(' ');
    if (decimal & 0x08)i2c_write_char('.');
    if (hund != 0 || thou != 0 || ten_thou != 0 || decimal & 0x1C) //do not print leading 100s '0'.
        i2c_write_char(hund + 48);
    else i2c_write_char(' ');
    if (decimal & 0x04)i2c_write_char('.');
    if (ten != 0 || hund != 0 || thou != 0 || ten_thou != 0 || decimal & 0x1E) //do not print leading 100s '0'.
        i2c_write_char(ten + 48);
    else i2c_write_char(' ');
    if (decimal & 0x02)i2c_write_char('.');
    i2c_write_char(unit + 48);
}

//void print_hr_min_min_s (unsigned int value, unsigned char pos)     //a function to time format settings
//{
//    save_beep();
//    lcd_set_cursor(2, pos);
//    unsigned char x = 0;
//    x = value / 60;
//    hund_conv(x);
//    if (ten != 0)                                           //do not print leading 100s '0'.
//        i2c_write_char(ten + 48);
//    else i2c_write_char(32);
//    i2c_write_char(unit + 48);
//    popup_print_number(value - (x * 60), 12, 1);
//}
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Uart reciever communications">

void init_uart1(void)
{ //init uart, 57600baud, No-parity, 8-bits, 1-stop, No hardware flow control (9600 8N1 None)
    TX1STA = 0b00100000; //txen
    RC1STA = 0b10010000; //serial port enabled ad cf
    BAUD1CON = 0b00001000;
    SP1BRGH = 0x00; 
    SP1BRGL = 103; 
}


void init_uart2(void)
{ //init uart, 57600baud, No-parity, 8-bits, 1-stop, No hardware flow control (9600 8N1 None)
    TX2STA = 0b00100000; //txen
    RC2STA = 0b10010000; //serial port enabled ad cf
    BAUD2CON = 0b00001000;
    SP2BRGH = 0x00; 
    SP2BRGL = 103; 
}

//void disable_uart(void)
//{
//    TX2STAbits.TXEN = 0; //disable transmitter
//    RC2STAbits.CREN = 0; //disable receiver
//    RC2STAbits.SPEN = 0; //disable UART
//    //    RF_In_3_RX_TRIS = 1; //RX is an input
//    //    RF_In_4_TX_TRIS = 1; //TX is an input
//}

//void enable_uart(void)
//{
//    TX2STAbits.TXEN = 1; //enable transmitter
//    //    RC2STAbits.CREN = 1; //enable receiver
//    RC2STAbits.SPEN = 1; //enable UART
//    uart_reset();
//    //    RF_In_3_RX_TRIS = 0; //RX is an output
//    //    RF_In_4_TX_TRIS = 0; //TX is an output
//}

void send_uart1(unsigned char uart1_data) {
    while (TX1STAbits.TRMT == 0); //Transmission still in progress? or exit pressed
    TX1REG = uart1_data; //send data
}

void send_uart2(unsigned char uart2_data) {
    while (TX2STAbits.TRMT == 0); //Transmission still in progress? or exit pressed
    TX2REG = uart2_data; //send data
}

//fill in 0 for data when using Other Commands //delay of 60ms at end ensures that you can send packet after packet without it being missed by Roll Up
void set_rollup(unsigned char cmd, unsigned char data) {
    
    unsigned int chksum = cmd+data;
    unsigned char chksum_lo;
    unsigned char chksum_hi;
    
    if (chksum>255)
    {
        chksum_hi=1;
        chksum_lo=chksum-255;
    }
    
    else
    {
        chksum_hi=0;
        chksum_lo=chksum;
    }
    
    send_uart2(cmd); //command
    send_uart2(0); //data high
    send_uart2(data); //data low
    send_uart2(chksum_hi); // chksum hi - no bits sen there as anything greater than 
    send_uart2(chksum_lo);
    __delay_ms(60);
}

//delay of 60ms at end ensures that you can send packet after packet without it being missed by Roll Up
void get_rollup(unsigned char cmd) {
    send_uart2(cmd); //command
    send_uart2(0); //data high
    send_uart2(0); //data low
    send_uart2(0); // chksum hi - no bits sen there as anything greater than 
    unsigned char chksum = cmd;
    send_uart2(chksum);
//    __delay_ms(60);
}

void uart_reset(void) {
    unsigned char dummy_clear;
    //RC1STAbits.CREN = 0;
    RC2STAbits.CREN = 0; // --RESET receiver
    NOP();
    //RC1STAbits.CREN = 1;
    RC2STAbits.CREN = 1;
    //while (RC1IF == 1)
    //    dummy_clear = RC1REG; //make sure buffer is clear
    while (RC2IF == 1)
        dummy_clear = RC2REG; //make sure buffer is clear
}
    
unsigned int DUT_comms(unsigned int callsign) { //assign testjig_timer before calling function unless it is in a loop, assign timer before loop starts
        
    while(testjig_timer)
    {
        uart_reset();
        get_rollup(callsign);
        uart_receive_four(1000);
        if (uart_buff_work_recieve[0] == callsign)
        {
            break;
        }
    }
    
    power_supply_set(NONE);
    lcd_print_int(callsign, 10, 0b00001000, 1);
    
    while(1)
    {}
    
//    if (testjig_timer==0)
//    {
//        lcd_print_int(callsign, 10, 0b00001000, 1);
//        __delay_ms(1000);
//        print_error("Wrong","Callsign Returned");
//    }
        
    //can potentially add checksum stuff here
    
    //gives the value received from using 
    ADC_val=uart_buff_work_recieve[1]*255+uart_buff_work_recieve[2]; //bytes 1, 2 = data_high, data_low
    voltage_i = ADC_to_mv_DUT(ADC_val);
//    lcd_print_int(voltage_i, 10, 0b00001000, 1);
//    __delay_ms(1000);
    return voltage_i;
}

unsigned int DUT_voltage_check(unsigned int voltage_actual, unsigned int voltage_req){ //generates the thresholds for voltage monitoring on DUT
    testjig_timer=250;
    while(testjig_timer){
        voltage_req=voltage_req*1000; //convert to mv
//        lcd_print_int(voltage_req, 10, 0b00001000, 1);
//        __delay_ms(1000);
        lower = voltage_req-voltage_req/10;
//        lcd_print_int(lower, 10, 0b00001000, 1);
//        __delay_ms(1000);
        higher = voltage_req+voltage_req/10;
//        lcd_print_int(higher, 10, 0b00001000, 1);
//        __delay_ms(1000);
        if (voltage_actual<lower)
        {
            return 0;
        }
        
        if (voltage_actual>higher)
        {
            return 1;
        }
    
        return 2;
        }    
}

unsigned char uart_receive_four(unsigned int uart2_timer) 
{
    unsigned char uart_rx_byte_count1 = 0;
    uart_receive_timer = uart2_timer;
    while (uart_rx_byte_count1 < 5 && uart_receive_timer)//wait for the timer or byte recieved or exit button press
    {
        if (RC2IF == 1) 
        {
            if ((RC2STAbits.OERR == 1) || (RC2STAbits.FERR == 1)) //IF ANY RX ERROR OCCURS // RESET RX SESSION!! >>
            {//yes
                int error1 = RC2STAbits.OERR;
                int error2 = RC2STAbits.FERR;
                lcd_print_int(error1,4,0,0);
                lcd_print_int(error2,12,0,0);
//                __delay_ms(1000);
                uart_reset();
                return UART1_RESET; //reset receive buffer & clear data
            } 
            else
            {
                uart_buff_work_recieve[uart_rx_byte_count1++] = RC2REG;
            }
        }
    }
    if (uart_rx_byte_count1 == 4)
        return UART1_BUFFER_FULL;
    else
        return UART1_TIMEOUT;
}

unsigned char uart_send_receive_packet(unsigned char opcode, unsigned char byte_1, unsigned char byte_2, unsigned char byte_3, unsigned int uart2_receive_timer) 
{
    RC2STAbits.OERR = 0; //reset receive buffer & clear data
    send_uart2(opcode);
    if (opcode != 0x89) //does more data need to be sent
    {
        send_uart2(byte_1);
        send_uart2(byte_2);
        send_uart2(byte_3);
    }
    //    else
    //        __delay_us(500);
    return uart_receive_four(uart2_receive_timer); //return - USART1_TIMEOUT, USART1_BUFFER_FULL
}

// </editor-fold>

void init_i2c(void) {
    lcd_reset = 1; //reset pin high
    __delay_ms(200);
    TRISC3 = 1; /* SDA and SCL as input pin */
    TRISC4 = 1; /* these pins must be configured as i/p for 1527 */
    SSP1STAT |= 0x80; /* Slew rate disabled */
    SSP1CON1 = 0x28; /* SSPEN = 1, I2C Master mode, clock = FOSC/(4 * (SSPADD + 1)) */
    SSP1ADD = 0x14; /* 100kHz @ 4MHz Fosc */
}

void init_ADC(void) {
    FVRCON = 0b10000011; //4.096v reference
    ADCON0 = 0b00000001; //channel 0, conversion not started, ADC on
    ADCON1 = 0b10100011; //right justified, Fosc/32, Vref is FVR
    ADRESH = 0b00000000;
    ADRESL = 0b00000000;
}

float ADC_to_voltage(unsigned int ADC){
    float return_val;
    return_val = ADC*11*4.096/1024; // 11=Vin/Vpin, 4.096=Vref, 1024 is 10 bit max
    return return_val;
}

unsigned int ADC_to_mv(unsigned int ADC){
    unsigned int return_val;
    return_val = ADC*44; // 11=Vin/Vpin, 4.096=Vref, 1024 is 10 bit max
    return return_val;
}

unsigned int ADC_to_mv_DUT(unsigned int ADC){
    unsigned int return_val;
    return_val = ADC*53; // only difference to above is that vref is 5v not 4.096v
    return return_val;
}

unsigned int ADC_get_val(unsigned char channel) 
{
    unsigned int return_val;
    GIE = 0;
    ADCON0 = channel; //set ADC channel
    __delay_ms(1);
    
    ADCON0bits.GO_nDONE = 1; //sample time
    
    while (ADCON0bits.GO_nDONE == 1); //wait for conversion to complete
    {}
    
    return_val = ADRESH << 8 | ADRESL;
    GIE = 1;
    return return_val;
}

unsigned int board_detected(unsigned int ADC) {
    unsigned int detected; // 1 for board present, 0

    if ((ADC >= 61)&&(ADC <= 67)) // 3.13v and 3.46 volts respectively (roughly))
        detected = 1;
    else
        detected = 0;

    return detected;
}

unsigned int board_type(unsigned int ADC) //tx or rx
{
    unsigned int detected; // 1 for board present, 0   this is not robust at all, change resistor value or change things up

    if ((ADC >= 15)&&(ADC <= 21)) // +- 0.7 volts is the threshold
        detected = 0; //TX
    else if (ADC < 15)
        detected = 1; //RX
    return detected;
}

void init_current_sink(void) 
{
    T2CON = 0b00000110; //timer on, interrupt every 1ms // prescalar is 16
    PR2 = 0xFA; //1ms
    PIE1bits.TMR2IE = 1; //enable timer interrupt

    T8CON = 0b00000100; //timer 8 on, prescaler = 1.
    PR8 = 0xFF;
    CCP9CON = 0b00001100; //CCP in PWM mode // 
    CCPTMRS2 = 0b00000001; //CCP9 based off timer 8
    CCPR9L = 0x00; //turn off PWM
}

void power_supply_set(unsigned char supply) 
{
    switch (supply) 
    {
        case AC1:
            power_supply_k6 = 0;
            power_supply_k7 = 1;
            power_supply_k8 = 0;
            power_supply_k9 = 0;
            power_supply_k10 = 0;
            //power_supply_k12 = 0;
            power_supply_k13 = 0;
            break;
        case AC2:
            power_supply_k6 = 0;
            power_supply_k7 = 1;
            power_supply_k8 = 1;
            power_supply_k9 = 0;
            power_supply_k10 = 0;
            //power_supply_k12 = 0;
            power_supply_k13 = 0;
            break;
        case BAT:
            power_supply_k6 = 0;
            power_supply_k7 = 0;
            power_supply_k8 = 0;
            power_supply_k9 = 0;
            power_supply_k10 = 0;
            //power_supply_k12 = 0;
            power_supply_k13 = 0;
            break;
        case ACTEST:
            power_supply_k6 = 0;
            power_supply_k7 = 0;
            power_supply_k8 = 0;
            power_supply_k9 = 0;
            power_supply_k10 = 0;
            //power_supply_k12 = 0;
            power_supply_k13 = 0;
            break;    
        case NONE:
            power_supply_k6 = 0;
            power_supply_k7 = 0;
            power_supply_k8 = 0;
            power_supply_k9 = 0;
            power_supply_k10 = 0;
            power_supply_k11 = 0;
            power_supply_k12 = 0;
            power_supply_k13 = 0;
            break;
    }
}

unsigned int get_current(void) 
{//64mA @ 50 counts
    unsigned int temp;
    temp = ADC_get_val(ADC_Q_curr_sense);
    
    temp = temp * 1.28; //factor 1.28 
//    if (temp > 4)
//        return ((temp * 3.4) - 16)/4; //this is multiplied by 4 to get mA
//    else
//        return 0;
    
    return temp; //mA
}

void Vout_set(int Volts) {
    unsigned long digi_step = 0;
    //   unsigned long res_variable = (SMPS_REF*PASSIVE_RES)/(milliVolts-1);

    // float fitting = SMPS_GAIN*(float)(milliVolts) + SMPS_CONST;

    //   digi_step = 128-(res_variable*128)/DIGITAL_RES_MAX;
    switch (Volts) {
        case 6:
            digi_step = 20;
            break;
        case 7:
            digi_step = 39;
            break;
        case 8:
            digi_step = 52;
            break;
        case 9:
            digi_step = 62;
            break;
        case 10:
            digi_step = 70;
            break;
        case 11:
            digi_step = 76;
            break;
        case 12:
            digi_step = 81;
            break;
        case 13:
            digi_step = 85;
            break;
        case 14:
            digi_step = 88;
            break;
        case 15:
            digi_step = 91;
            break;
        case 16:
            digi_step = 94;
            break;
        case 17:
            digi_step = 96;
            break;
        case 18:
            digi_step = 98;
            break;
        case 20:
            digi_step = 102;
            break;
        case 25:
            digi_step = 112;
            break;
        default:
            digi_step = 0;
            break;
    }

    //   test_funct_display("step", (int)digi_step);

    //the above is the ideal, 

    digitalpot((unsigned char) digi_step);
}

void digitalpot(unsigned char value) {
    i2c_Start();
    i2c_Write(i2c_pot_address); //0x5C slave address
    i2c_Write(0b00000000);
    i2c_Write(value);
    i2c_Stop();
    __delay_ms(20);
}

void debug_fast_tx(unsigned int data) {
    GIE = 0;
    trigger = 0;
    //    NOP();
    //    NOP();
    //    NOP();
    //    NOP();
    //    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    unsigned char i;
    for (i = 0; i < 16; i++) {
        trigger = data & 0x01;
        data = data >> 1;
    }
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    trigger = 1;
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    NOP();
    GIE = 1;
}

void TX_one(unsigned char data) {
    comms_TX_TRIS = 0;
    comms_TX = 0;
    __delay_us(100);

    unsigned char i;
    for (i = 0; i < 8; i++) {
        comms_TX = data & 0x01;
        data = data >> 1;
        __delay_us(100);
    }
    comms_TX = 1;
    __delay_us(100);
}

unsigned char RX_one(void) {
    //    comms_RX_TRIS = 1; //make receive pin an input
    while (comms_RX == 1); //wait for start bit
    __delay_us(150);
    unsigned char i;
    unsigned char data = 0;
    for (i = 0; i < 8; i++) {
        //debug_1 = 1;
        if (comms_RX == 0)
            data = data >> 1;
        else
            data = (data >> 1) | 0x80; // copies the old bit 0 into bit 7
        //debug_1 = 0;
        __delay_us(100);
    }
    __delay_us(100);
    //    debug_fast_tx(data);
    return data;
}

unsigned char RX_one_timeout(void) {
    //    comms_RX_TRIS = 1; //make receive pin an input
    uart_receive_timer = 500; //66ms
    while (comms_RX == 1 && uart_receive_timer); //wait for start bit
    __delay_us(150);
    unsigned char i;
    unsigned char data = 0;
    for (i = 0; i < 8; i++) {
        //debug_1 = 1;
        if (comms_RX == 0)
            data = data >> 1;
        else
            data = (data >> 1) | 0x80; // copies the old bit 0 into bit 7
        //debug_1 = 0;
        __delay_us(100);
    }
    __delay_us(100);
    //    debug_fast_tx(data);
    return data;
}

//test programmer
//        k3 = 1;
//        k4 = 1;
//        __delay_ms(1000);
//        pickit_button_relay = 1;
//        __delay_ms(200);
//        pickit_button_relay = 0;
//        
//        __delay_ms(3000);
//        __delay_ms(3000);
//        __delay_ms(3000);
//        __delay_ms(2000);
//        
//        k3 = 0;
//        k4 = 0;