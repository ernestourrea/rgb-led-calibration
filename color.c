/* 
 * File:   color.c
 * Author: rahu7p
 */

#include "color.h"
#include <string.h>
#include <math.h>
#include "i2c.h" 
#include "myprintf.h"
/**
 Section: Modules

 */
void Color_Initialize(void) {   
    //Initializes the Color sensor Click Board    
    DataByte[0] = 0x40;// SYSTEM_CONTROL
    DataByte[1] = 0xC0;
    I2C_MasterWrite(DataByte, 2, I2C_SLAVE_ADDR);
    __delay_ms(10);
    
    DataByte[0] = 0x41;// MODE_CONTROL1
    DataByte[1] = 0x7D;
    I2C_MasterWrite(DataByte, 2, I2C_SLAVE_ADDR);
    __delay_ms(10);
    
    DataByte[0] = 0x42;
    DataByte[1] = 0x10;// MODE_CONTROL2
    I2C_MasterWrite(DataByte, 2, I2C_SLAVE_ADDR);
    __delay_ms(10);    
}

/*
void Color_Read() {
    //The R,G,B values
    Red=Read_Register_Data(0x50); 
    RedData=Read_Register_Data(0x51);
    RedData = ((RedData<<8)|Red); 
    RedRatio = (float)(RedData * 1.8);
    
    Green=Read_Register_Data(0x52); 
    GreenData=Read_Register_Data(0x53);
    GreenData = ((GreenData<<8)|Green);
    GreenRatio = (float)(GreenData * 1.00);
     
    Blue=Read_Register_Data(0x54); 
    BlueData=Read_Register_Data(0x55);
    BlueData = ((BlueData<<8)|Blue);
    BlueRatio = (float)(BlueData * 1.83);
    
    Max = MAX_FLOAT(MAX_FLOAT(RedRatio, GreenRatio), BlueRatio);
    if( Max == 0)
        Max = 1.0;
    
    RedRatio = (RedRatio < 10) ? 0 : RedRatio;
    GreenRatio = (GreenRatio < 10) ? 0 : GreenRatio;
    BlueRatio = (BlueRatio < 10) ? 0 : BlueRatio;
    
    RedRatio = RedRatio / Max;
    GreenRatio = GreenRatio / Max;
    BlueRatio = BlueRatio / Max;
       
    RatioMax = MAX_FLOAT(MAX_FLOAT(RedRatio, GreenRatio), BlueRatio);
    RatioMin = MIN_FLOAT(MIN_FLOAT(RedRatio, GreenRatio), BlueRatio);
    
    Delta = RatioMax - RatioMin;
    
    //Hue calculation
    if( Delta == 0)
        Hue = 0;
    else if( RatioMax == RedRatio ){
        Hue = (int16_t)(fmod(((GreenRatio - BlueRatio) / Delta), 6) * 60);
        if( Hue < 0 )
            Hue = 360 + Hue;
    }
    else if( RatioMax == GreenRatio )
        Hue = (int16_t)((2 + ((BlueRatio - RedRatio) / Delta)) * 60);
    else
        Hue = (int16_t)((4 + ((RedRatio - GreenRatio) / Delta)) * 60);
    
    //Saturation calculation
    if( RatioMax == 0 )
        Saturation = 0;
    else
        Saturation = (uint8_t)((Delta / RatioMax) * 100);

    //Value calculation
    Value = floor(RatioMax*100.0);
}
*/

void Color_Read_RGB(uint8_t color) {
    if(color == 0){
        Red=Read_Register_Data(0x50); 
        RedData=Read_Register_Data(0x51);
        RedData = ((RedData<<8)|Red); 
    }
    else if (color == 1){
        Green=Read_Register_Data(0x52); 
        GreenData=Read_Register_Data(0x53);
        GreenData = ((GreenData<<8)|Green);
    }
    else if (color == 2){
        Blue=Read_Register_Data(0x54); 
        BlueData=Read_Register_Data(0x55);
        BlueData = ((BlueData<<8)|Blue);
    }
    else{
        Red=Read_Register_Data(0x50); 
        RedData=Read_Register_Data(0x51);
        RedData = ((RedData<<8)|Red); 
    
        Green=Read_Register_Data(0x52); 
        GreenData=Read_Register_Data(0x53);
        GreenData = ((GreenData<<8)|Green);
    
        Blue=Read_Register_Data(0x54); 
        BlueData=Read_Register_Data(0x55);
        BlueData = ((BlueData<<8)|Blue);
    }
}

void My_Color_Read() {
    //The R,G,B values
    Red=Read_Register_Data(0x50); 
    RedData=Read_Register_Data(0x51);
    RedData = ((RedData<<8)|Red); 
    RedRatio = (float)(RedData);
    
    Green=Read_Register_Data(0x52); 
    GreenData=Read_Register_Data(0x53);
    GreenData = ((GreenData<<8)|Green);
    GreenRatio = (float)(GreenData);
    
    /*
    Green2=Read_Register_Data(0x5A); 
    Green2Data=Read_Register_Data(0x5B);
    Green2Data = ((Green2Data<<8)|Green2);
    Green2Ratio = (float)(Green2Data);
     */
     
    Blue=Read_Register_Data(0x54); 
    BlueData=Read_Register_Data(0x55);
    BlueData = ((BlueData<<8)|Blue);
    BlueRatio = (float)(BlueData);
    
    Max = MAX_FLOAT(MAX_FLOAT(RedRatio, GreenRatio), BlueRatio);
    if( Max == 0)
        Max = 1.0;
    
    RedRatio = (RedRatio < LThreshold) ? 0.0 : RedRatio;
    RedRatio = (RedRatio > RedHThreshold) ? RedHThreshold : RedRatio;
    
    GreenRatio = (GreenRatio < LThreshold) ? 0.0 : GreenRatio;
    GreenRatio = (GreenRatio > GreenHThreshold) ? GreenHThreshold : GreenRatio;
    
    BlueRatio = (BlueRatio < LThreshold) ? 0.0 : BlueRatio;
    BlueRatio = (BlueRatio > BlueHThreshold) ? BlueHThreshold : BlueRatio;
    
    RedRatio = RedRatio / RedHThreshold;
    GreenRatio = GreenRatio / GreenHThreshold;
    BlueRatio = BlueRatio / BlueHThreshold;
       
    RatioMax = MAX_FLOAT(MAX_FLOAT(RedRatio, GreenRatio), BlueRatio);
    RatioMin = MIN_FLOAT(MIN_FLOAT(RedRatio, GreenRatio), BlueRatio);
    
    Delta = RatioMax - RatioMin;
    
    //Hue calculation
    if( Delta == 0)
        Hue = 0;
    else if( RatioMax == RedRatio ){
        Hue = (int16_t)(fmod(((GreenRatio - BlueRatio) / Delta), 6) * 60);
        if( Hue < 0 )
            Hue = 360 + Hue;
    }
    else if( RatioMax == GreenRatio )
        Hue = (int16_t)((2 + ((BlueRatio - RedRatio) / Delta)) * 60);
    else
        Hue = (int16_t)((4 + ((RedRatio - GreenRatio) / Delta)) * 60);
    
    //Saturation calculation
    if( RatioMax == 0 )
        Saturation = 0;
    else
        Saturation = (uint8_t)((Delta / RatioMax) * 100);

    //Value calculation
    Value = floor(RatioMax*100.0);
}

const uint16_t HSV_Values[8][3] = {
    {  0,  0,  0},
    {  0,  0,100},
    {  0,100,100},
    {120,100,100},
    {240,100,100},
    { 60,100,100},
    {180,100,100},
    {300,100,100},
};

const char HSV_Names[8][9] = {
    "Negro",
    "Blanco",
    "Rojo",
    "Lima",
    "Azul",
    "Amarillo",
    "Cian",
    "Magenta"
};

//color valores[7] = {
    
//};

void Color_Display() {
    //Based on the hue value, the color is detected and displayed
    const uint8_t HThreshold = 30;
    const uint8_t SThreshold = 100;
    const uint8_t VThreshold = 100;
    
    for(uint8_t i = 0; i < 8; i++){
        if ((abs(HSV_Values[i][0] - Hue) < HThreshold) && (abs(HSV_Values[i][1] - Saturation) < SThreshold) && (abs(HSV_Values[i][2] - Value) < VThreshold)){
            myprintf(HSV_Names[i]);
            return;
        }
    }
    myprintf("CNI   ");
}

uint8_t Read_Register_Data(uint8_t address) {
    //Reads the data present at data register over I2C
    I2C_MasterWriteRead(DataByte, 1, I2C_SLAVE_ADDR, address);//Write register address on I2C                            //Read data register
    __delay_ms(0x10);
    
    return DataByte[0]; 
}

/**
 End of File
*/
