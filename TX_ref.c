void Testjig(void)
{
    unsigned int i;
    unsigned char RF_data[25]; //24 data bytes and a 16 bit adding checksum
    unsigned int checksum = 0;
    LED_SetHigh();
    while(button_state != pressed);//wait for button press
    LED_SetLow();
    delay_10ms(10);

     
    menu_timeout = 1000; //10s
    myEUSART1_Initialize();
    
    PWM4EN = 0; //disable PWM
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00; // unlock PPS

    RX1DTPPSbits.RX1DTPPS = 0x03;   //RA3->EUSART1:RX1;
    RA0PPS = 0x00;   //RA0->RA0;

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01; // lock PPS

    GIE = 1;
    
    PORTAbits.RA0 = 1; //P6 high
    delay_10ms(30); //wait10ms
    PORTAbits.RA0 = 0; //P6 low
    if(INPUT1_GetValue() == 0)//check P8 is high, if it isnt wait in while loop forever.
        while(1);
    while(INPUT1_GetValue() == 1); //check P8 goes low
    if(INPUT2_GetValue() == 0)//check P7 is high
        while(1);
    while(INPUT2_GetValue() == 1); //wait for P7 to go low      
    myEUSART1_Read(); //flush data here
    PORTAbits.RA0 = 1;//P6 high
    
    LED_SetHigh();
//    while(myEUSART1_Read() == 0xFF);//wait for valid data
////    while(myEUSART1_Read() == 0xFF);
//    delay_10ms(5);                  //wait 50ms for packet to end
//    while(myEUSART1_Read() != 0x70);//wait for start of next packet
    for(i = 0; i < 25; i++)
    {
        RF_data[i] = myEUSART1_Read();
        checksum += RF_data[i];
    }
    LED_SetLow();

    if(((checksum & 0x00FF) != 0x01) || (menu_timeout == 0))//if checksum does not add
    {
        delay_10ms(100);
        LED_SetHigh();
        while(1);
    }
    else
    {
        for(i = 0; i < 24; i++)
        {
            FLASH_WriteChar(SERIAL0 + i, Flash_buf, RF_data[i]);
        }
        //enable PWM
        TX_loop(1, 0);
        TX_loop(1, 0);
        TX_loop(1, 0);
        FLASH_WriteChar(MEM_Testjig_status_adr, Flash_buf, 0xAA);
        while(1);
    }
}