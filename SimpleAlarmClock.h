/**************************************************************************
 * SimpleAlarmClock Library 
 * A library for the ZS-042 Module that has a DS3231 RTC and AT24C32 EEPROM. 
 *
 * An easy set of read/write functions or methods for the DS3231 Real Time
 * Clock and Calendar, and the companion AT24C32 EEPROM on common ZS-042 
 * modules.  The EEPROM is used by this library to store alarm settings on
 * registers 0x00 through 0x08 as byte values.
 *
 * The remainder of the EEPROM can be used for logging or storage 
 *
 * The EEPROM read and write bytes methods were copied from this Library
 * The AT24CX Library located here:
 *     https://github.com/cyberp/AT24Cx
 *
 * The intent of this library is to manage a basic alarm clock, that 
 * includes the display clock and two alarms, typically seen on 
 * standard desktop/nightstand alarm clocks.  It should accomidate switching 
 * between 12 hour and 24 modes on the fly.  Alarms have four 
 * alarm modes: Daily, Weekday, Weekend or Once.  This library assumes that  
 * alarm DOW will only be used, and alarm DATE shall not be used. The alarm 
 * modes are stored in EEPROM address 0x08.  Snooze option is available
 * for both alarms.
 *
 * This library also makes the assumption that if the year has been set to 
 * 2018 (018) or greater, the RTC clock is probably valid. This approach 
 * is used so that during a power cycle (blackout), clock settings
 * saved by the RTC's backup battery will not be reset.
 * 
 * Copyright (C) 2018 Ricardo Moreno Jr.
 *
 * Originally inspired by DS3231_Simple by 2016 James Sleeman, this forked
 * version is incompatible with original. However, since a small amount of
 * code is used the following notice is included:
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the "Software"), 
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included  
 * in all copies or substantial portions of the Software.
 *
 * AT24C32 EEPROM Read and Write byte code copied from AT24Cx Library 
 * by Christian Paul. Modified by Ricardo Moreno.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE  
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * @author Ricardo Moreno
 * @version 1.0.0
 * @copyright Ricardo Moreno
 * @license MIT License
 ************************************************************************ 
 *  Versions:
 *  11/25/2018 v1.0.0  Initial version/release
 *
 ************************************************************************ */

#ifndef SimpleAlarmClock_h
#define SimpleAlarmClock_h
#include <Wire.h>
#include <Arduino.h>

//Structure used for clock object
struct DateTime {      
  byte Second;       // 0-59 = 6 bits
  byte Minute;       // 0-59 = 6 bits
  byte Hour;         // 0-23 = 5 bits
  byte Dow;          // 1-7  = 3 bits (Day Of Week)
  byte Day;          // 1-31 = 5 bits (Day Date of the Month) 
  byte Month;        // 1-12 = 4 bits
  byte Year;         // 0-199 = 8 bits
  byte ClockMode;    // 0-2; 0=AM, 1=PM, 2=24hour
};

//Structure used for alarm object
struct AlarmTime {
  byte Second;       // 0-59 = 6 bits 0=for alarm2
  byte Minute;       // 0-59 = 6 bits
  byte Hour;         // 0-23 = 5 bits
  byte AlarmMode;    // 0=Daily, 1=Weekday, 2=Weekend 3=Once
  byte ClockMode;    // 0-2; 0=AM, 1=PM, 2=24hour
  bool Enabled;      // true or false 
};

class SimpleAlarmClock
{
  /********************************************************************************************/
  public:
    /************************************************************************
     * Simple Alarm Clock methods and properties 
     ********************************************************************* */
    /*	I2C address can be found in the datasheet Figure 1. AT24C32 Device 
		Address ZS-040 module has pull-up resistors on these pins
		giving them a default value of 1.  
		Shorting an individual pad results in different address:
			pads      Binary    Hex    Dec
		| Default  | b1101111 | 0x57 | 87 |
		| short A0 | b1101110 | 0x56 | 86 |
		| short A1 | b1101101 | 0x55 | 85 |
		| short A2 | b1101011 | 0x53 | 83 |
		| All Short| b1101000 | 0x50 | 80 |
                allowing up to eight combinations                          */	
    
    // CONSTRUCTOR
    SimpleAlarmClock(byte _rtc_Address=0x68, byte _eeprom_Address=0x57, bool alarmIntEnabled=true);
                                         /* CONSTRUCTOR                                       */
                                         /* <optional> Parameters:                            */
                                         /*      _rtc_Address = address of the RTC,           */
                                         /*                     0x68 <default>                */
                                         /*   _eeprom_Address = address of the AT24C32        */
                                         /*                     0x57 <default>                */
                                         /*   alarmIntEnabled = true or false, sets INTCN bit */
                                         /*                     allowing INTSQW pin monitoring*/
   
    void begin();                        /* Initialize SimpleAlarmClock                       */

    /******************************************************************************************/
    /* Main Clock Methods                                                                     */	
    DateTime read();                     /* Read the current date and time values as DateTime */
                                         /* Returns: a structure DateTime containing that     */
                                         /*          information.                             */
    
    byte write(const DateTime&);         /* Set the Clock from the given DateTime structure   */
                                         /*   Returns 0=fail or 1=success                     */
    
    /******************************************************************************************/
    /* Main Alarm Methods                                                                     */	
    AlarmTime readAlarm(byte alarmSelected);
                                         /* Read the alarm time as AlarmTime struct values    */
                                         /* Parameters:                                       */
                                         /*   alarmSelected = 1 (alarm1) or 2 (alarm2)        */
                                         /* Returns: a structure AlarmTime containing that    */
                                         /*          information                              */

    byte setAlarm(const AlarmTime&, byte alarmSelected);
                                         /* Set the alarm values as AlarmTime struct          */
                                         /* Parameters:
                                         /*   AlarmTime - structure as AlarmTime              */
                                         /*   alarmSelected - 1 or 2 (alarm 1 or alarm2)      */
                                         /* Returns 0=fail                                    */
                                         /*         1=success                                 */
                                         /*         2=error wrong index value                 */                                         
                                          
    byte snoozeAlarm(byte alarmSelected, byte SnoozeTime);
                                         /* Snooze alarm1 for a max 255 minutes)              */

    void armAlarm(byte alarmSelected, bool Enable);
                                         /* Arm or disarm alarms - INTCN, A2IE or A1IE bits   */
                                         /*  alarmSelected = 1 (alarm1) or 2 (alarm2)         */
                                         /*  Enable = true or false                           */            
    
    void clearAlarms();                  /* Clears the alarms flags                           */
    byte alarmStatus();                  /* Determines what alarms are enabled                */
                                         /*   Returns 0=No alarm                              */
                                         /*           1=Alarm 1                               */
                                         /*           2=Alarm 2                               */
                                         /*           3=Both Alarms                           */
	
    /******************************************************************************************/
    /* Control Register 0x0e bit fiddlers                                                     */
    byte getCtrlRegister();              /* Returns Control register 0x0e as a byte           */
    void setEnableOscillator(bool Enable);/* Set or clear EOSC bit in register 0x0e           */
                                          /* note: enable is 0 and disable is 1 for this case */
    void setBatteryBackedSquareWave(bool Enable); //Set or clear BBSQW bit in register 0x0e   */
    void setConvertTemperature();        /* Set CONV bit in register 0x0e                     */
    byte setRateSelect(byte Data);       /* Set RS2 and RS1 bits in register 0x0e (0-3)       */                                       
    void setInterruptCtrl(bool Enable);  /* Set or clear the INTCN bit in register 0x0e       */

    /******************************************************************************************/
    /* Status Register 0x0f bit fiddlers                                                      */
    byte getStatusRegister();            /* Returns the Status register 0x0f as a byte        */
    bool getOSFStatus();                 /* Returns true if Oscillator Stop Flag (OSF)=1      */
    byte clearOSFStatus();               /* This bit is set to logic 1 any time that the     
                                            oscillator stops. The following are examples of 
                                            conditions that can cause the OSF bit to be set:
                                               1) The first time power is applied.
                                               2) The voltages present on both VCC and VBAT
                                                  are insufficient to support oscillation.
                                               3) The EOSC bit is turned off in battery-backed
                                                  mode.
                                               4) External influences on the crystal (i.e., 
                                                  noise, leakage, etc.).
                                            This bit remains at logic 1 until written to 
                                            logic 0.
                                          *************************************************** */
    bool getEN32kHz();                   /* get 32kHz Output (EN32kHz) bit value              */
    byte setEN32kHz(bool Enable);        /* set 32kHz Output (EN32kHz) bit value              */
    bool busy();                         /* get the BSY bit value in register 0x0f            */
    byte flaggedAlarms();                /* Determines if an alarm has triggered/flagged      */
                                         /*   Returns 0=No alarm                              */
                                         /*           1=Alarm 1                               */
                                         /*           2=Alarm 2                               */
                                         /*           3=Both Alarms                           */
    
    /******************************************************************************************/
    /* Temperature Registers 0x11 and 0x12 methods                                            */
    byte getTemperature();               /* Get the temperature accurate to within 1 degree   */
                                         /*   (C)                                             */
    float getTemperatureFloat();         /* Get the temperature accurate to within 0.25       */
                                         /*   degrees (C)                                     */

    /******************************************************************************************/
    /* Misc Methods                                                                           */
    void toggleClockMode();              /* Toggles the ClockMode (clock & alarms) from 12hr  */
                                         /*   to 24 hr                                        */

    byte calcDow(byte mm, byte dd, uint16_t yyyy); 
                                         /* Returns day of the week based on the date         */
    void resetClock(void);               /* Reset Clock to default values                     */
    void resetAlarm(byte alarmSelected); /* Reset Alarm to default values                     */
                                         /*   alarmSelected = 1 for alarm1                    */
                                         /*                 = 2 for alarm2                    */    

    /******************************************************************************************/
    // Lower Level methods: 
    
    /* NOTE: the time, date, day of the week, and alarms are stored in BCD format             */ 
    byte readByte(byte Address,  byte &dataBuffer); 
    byte readBytes(byte Address, byte dataBuffer[], byte Length);
    byte getAgingOffset(void);          /* Return the aging offest register value             */
    byte setAgingOffset(int changeValue);/* The adjust the crystal aging offset register      */
                                        /* Parameter:                                         */
                                        /*   changeValue = -128 through 127 (two’s complement)*/
                                        /* Check Datasheet to adjust this properly.           */

    /******************************************************************************************/
    /* EEProm methods */
    /* NOTE: the time, date, day of the week, and alarms are stored in BCD format             */ 
    byte readMem(byte address);
    byte readMem(byte address, byte dataBuffer[], byte n);
    byte writeEeprom(byte address, byte dataBuffer); /* write a byte at address specified.    */
                                               /* address 0x00 though 0x08 write protected    */
                                               /* Returns: 0 - fail                           */
                                               /*          1 - no errors                      */

  /********************************************************************************************/
  protected:
    static byte bcd2bin(byte bcdValue);
    static byte bin2bcd(byte binaryValue);
 
  /********************************************************************************************/
  private:
    AlarmTime readRtcAlarm(byte alarmSelected);
    AlarmTime readMemAlarm(byte alarmSelected);
    byte writeByte(byte Address, byte dataBuffer);    
    byte writeBytes(byte Address, byte dataBuffer[], byte arrayLength);	
    void writeMem(byte address, byte dataBuffer);
    void writeMem(byte address, byte dataBuffer[], byte arrayLength);
    byte setRtcAlarm(const AlarmTime&, byte alarmSelected);
    byte setMemAlarm(const AlarmTime&, byte alarmSelected);
    bool checkMemAlarm(byte alarmSelected);
    byte nextAlarmDay(byte _AlarmMode, byte _ClockMode, byte _Hour, byte _Minute);
	
    byte RTC_ADDRESS;
    byte EEPROM_ADDRESS;
    byte alarmIntEnabled = 1;                     /* Enable/Disable alarm interupt INT/SQW  */
    bool SnoozingA1;                              //Track if snoozing Alarm 1 is active
    bool SnoozingA2;                              //Track if snoozing Alarm 2 is active
    byte EOSC;
    byte BBSQW;
    byte CONV;
    byte RS2;
    byte RS1;
    byte INTCN;
    byte pageSize = 32;

};

#endif
