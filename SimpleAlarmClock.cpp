#include <SimpleAlarmClock.h>
#include <Wire.h>
/** **********************************************************************
 * SimpleAlarmClock Library - cpp file
 * A library for the ZS-042 Module that has a DS3231 RTC and AT24C32 EEPROM. 
 * @author Ricardo Moreno
 * @version 1.0.0
 * @copyright Ricardo Moreno
 * @license MIT License
 ************************************************************************ 
 *  Versions:
 *  11/25/2018 v1.0.0  Initial version/release
 *
 *********************************************************************** */


SimpleAlarmClock::SimpleAlarmClock(byte _rtc_Address, byte _eeprom_Address, bool alarmIntEnabled){
    /** ******************************************************************
     *                          Constructor                              *
     ******************************************************************* */
    RTC_ADDRESS = _rtc_Address;
    EEPROM_ADDRESS = _eeprom_Address;
    if (alarmIntEnabled == true){ INTCN = 1;} else { INTCN = 0;}
    // begin method will update local variables
}

byte SimpleAlarmClock::readByte(byte Address, byte &dataBuffer){
    /** ******************************************************************
     * readByte performs a simple single byte Wire.read to the varable
     * dataBuffer using a pointer at the Address provided.
     * Paramters:
     *   Address - register address location
     *   dataBuffer    - the varable, as a pointer, to store byte value
     *
     * Returns:
     *   Bits 7-4: error from Wire.endTransmission
     *   Bits 3-0: length from Wire.requestFrom
     ****************************************************************** */
    byte upperbyte = 0;
    byte lowerbyte = 0;

    Wire.beginTransmission(RTC_ADDRESS);
    Wire.write(Address);
    //This holds the device for exclusive communication
    upperbyte = Wire.endTransmission(false);
    /* Returns:
        byte, which indicates the status of the transmission:
        0:success
        1:data too long to fit in transmit buffer
        2:received NACK on transmit of address
        3:received NACK on transmit of data
        4:other error
    */
    if (upperbyte == 0) {
        lowerbyte = Wire.requestFrom((int)RTC_ADDRESS, 1, (int)true);
        /* Returns:
           # - the number of bytes returned from the slave device
        */
        //while(Wire.available()<=1){ dataBuffer = Wire.read(); }
        dataBuffer = Wire.read();
    } else {
        lowerbyte = 0;
    }
    return ((upperbyte << 4) | lowerbyte);
}

byte SimpleAlarmClock::readBytes(byte Address, byte dataBuffer[], byte Length){
    /** ******************************************************************
     * Reads multiple bytes from the register and stores them in the
     * dataBuffer array.
     * Parameters:
     *   Address      - registry location value
     *   databuffer[] - array to store byte values, as a pointer
     *   Length       - number of bytes or array length
     *
     * Returns:
     *   Bits 7-4: error from Wire.endTransmission
     *   Bits 3-0: length from Wire.requestFrom
     ****************************************************************** */
    byte upperbyte;
    byte lowerbyte;

    Wire.beginTransmission(RTC_ADDRESS);    //I2C address of the device
    Wire.write(Address);                    //Starting register for Readings
    //Hold the device for exclusive communication
    upperbyte = Wire.endTransmission(false);
    /* Wire.endTransmission will return the following values:
        0:success
        1:data too long to fit in transmit buffer
        2:received NACK on transmit of address
        3:received NACK on transmit of data
        4:other error
    */

    if (upperbyte == 0) {
        //Request "number" of bytes and release I2C bus
        lowerbyte = Wire.requestFrom((int)RTC_ADDRESS, (int)Length, (int)true);
        /* Returns:
           # - the number of bytes returned from the slave device
        */
        if (lowerbyte == Length){
            //while(!Wire.available()){};  //wait
            for (byte i=0; i<Length; i++){
                dataBuffer[i] = Wire.read();
            }
        }
    } else {
        lowerbyte = 0;
    }
    return((upperbyte << 4) | lowerbyte);
}

byte SimpleAlarmClock::writeByte(byte Address, byte dataBuffer){
    /** ******************************************************************
     * writeByte performs a simple single byte Wire.write of the
     * dataBuffer to the Address provided.
     * Paramters:
     *   Address - register address location
     *   dataBuffer    - the byte value to write
     *
     * Returns:
     *   0:success
     *   1:data too long to fit in transmit buffer
     *   2:received NACK on transmit of address
     *   3:received NACK on transmit of data
     *   4:other error
     ****************************************************************** */
    Wire.beginTransmission(RTC_ADDRESS); // This begins the I2C communication
    Wire.write(Address);                 // Access the register
    Wire.write(dataBuffer);              // Set the byte value to register above
    return Wire.endTransmission(true);   // Close I2C communication
}

byte SimpleAlarmClock::writeBytes(byte Address, byte dataBuffer[], byte Length){
    /** ******************************************************************
     * Writes multiple bytes to registry from an array dataBuffer
     * Parameters:
     *   Address      - registry location value
     *   databuffer[] - array to with byte values
     *   Length       - number of bytes or array length
     *
     * Returns:
     *   0:success
     *   1:data too long to fit in transmit buffer
     *   2:received NACK on transmit of address
     *   3:received NACK on transmit of data
     *   4:other error
     ****************************************************************** */
    Wire.beginTransmission(RTC_ADDRESS);   //device address
    Wire.write(Address);                   //Registry address
    for (byte i=0; i<Length; i++) {
        Wire.write(dataBuffer[i]);
    }
    return Wire.endTransmission(true);     // Close I2C communication
}

void SimpleAlarmClock::begin(void){
    /** ******************************************************************
     * The begin method to get SimpleAlarmClock working.
     *
     * Rather than resetting the RTC after startup.  It assumes that there
     * was a power loss. The RTC battery may have saved all the clock
     * configurations.  To avoid a rest, perform a simple check the
     * clock's year. If it is equal to or greater than 2018 (or 18), it's
     * probably OK, else initialize the clock and reset alarms.
     *
     * If the RTC battery fails, the RTC.year returns 2001 (or just 1).
     ****************************************************************** */
    byte year = 0;
    byte _byteValue;

    Wire.begin();
    // Read and define various registry values stored as variables,
    // See datasheet Figure 1 Timekeeping Registers
    //Set INTCN based on passed constructor value
    if (INTCN == 1){setInterruptCtrl(true);} else {setInterruptCtrl(false);}
    getCtrlRegister();
    // this function returns the register 0x0e as a byte, not used here,
    // but also sets the following variables:
    // EOSC, BBSQW, CONV, RS2, RS1, INTCN
    // variables might be used in the future?
    // Clear all snoozing
    SnoozingA1 = false;
    SnoozingA2 = false;

    /* ******************************************
     *          Determine the RTC year
     * *************************************** */
    // First check the Century bit
    byte i = readByte(0x05, _byteValue);
    // i used for future error checking

    // bit 7 of month register 0x05 indicates the century
    if (_byteValue & _BV(7)) { year += 100; }

    // Read year from register 0x06
    i = readByte(0x06, _byteValue);
    // i used for future error checking

    year += bcd2bin(_byteValue);

    /* ******************************************
     *          Check for valid year
     * *************************************** */
    // If year is less than 2018, reset RTC
    if (year < 18){
        resetClock();
        resetAlarm(1);
        resetAlarm(2);
    } else {
        // RTC clock might be OK - check alarms
        AlarmTime alarm_i;
        //check eepromAlarm1
        if (checkMemAlarm(1) == true){
            // checkMemAlarm() has a few alarm checks.
            // If power was lost and snoozed was pressed
            // assume alarm stored in Mem is correct
            // restore saved alarm
            alarm_i = readMemAlarm(1);
            setAlarm(alarm_i,1);
        } else {
            //Reset Alarm
            resetAlarm(1);
        }

        //check eepromAlarm2
        if (checkMemAlarm(2) == true){
            //if power was lost and snoozed was pressed
            //assume alarm stored in Mem is correct
            //restore saved alarm
            alarm_i = readMemAlarm(2);
            setAlarm(alarm_i,2);
        } else {
            //Reset Alarm
            resetAlarm(2);
        }
    }
}

void SimpleAlarmClock::resetClock(void){
    /** ******************************************************************
     * Resets the RTC clock to 12:00 AM and 01/01/2018
     * sets ClockMode to 12hr
     ****************************************************************** */

    DateTime currentDate;       //New local object

    currentDate.Second = 0;
    currentDate.Minute = 0;
    currentDate.Hour = 12;
    currentDate.Dow = 2;        // 1-7  2=Monday (Day Of Week)
    currentDate.Day = 1;        // calendar dd
    currentDate.Month = 1;      // calendar mm
    currentDate.Year = 18;      // calendar yyy, note 2000 must be addded
    currentDate.ClockMode = 0;  // 0-2; 0=AM, 1=PM, 2=24hour

    write(currentDate);

    //Reset Register 0x0e
    // assume default bits values with alarms off
    writeByte(0x0e, 0b00011100);
    getCtrlRegister();
    // this function returns the register 0x0e as a byte, not used here,
    // but also sets the following variables:
    // EOSC, BBSQW, CONV, RS2, RS1, INTCN
}

void SimpleAlarmClock::resetAlarm(byte alarmSelected){
    /** ******************************************************************
     * Reset Alarms - 12hr ClockMode
     * Parameters:
     *   alarmSelected - 1 or 2 for alarm1 or alarm2, anything else is
     *                   ignored.
     ****************************************************************** */
    //Check for valid alarmSelected
    if ((alarmSelected == 1)||(alarmSelected == 2)){
        AlarmTime alarm_i;

        alarm_i.Second = 0;         // 0-59 = 6 bits
        alarm_i.Minute = 0;         // 0-59 = 6 bits
        alarm_i.Hour = 6;           // 0-23 = 5 bits
        alarm_i.AlarmMode = 1;      // 0=Daily, 1=Weekday, 2=Weekend, 3=Once
        alarm_i.ClockMode = 0;      // 0-2; 0=AM, 1=PM, 2=24hour
        alarm_i.Enabled = false;
        setAlarm(alarm_i, alarmSelected);
    }
    // else do nothing
}

DateTime SimpleAlarmClock::read(void){
    /** ******************************************************************
     * The method to capture the time, date, and clockmode of the RTC
     * and returns it to the passed DateTime object.  All times, dow,
     * dates are stored in BCD format.
     * Returns:
     *   DateTime object
     ****************************************************************** */
    DateTime currentDate;
    byte _byteValue[7];

    // Read in the next 7 bytes which store the
    //  Seconds, Minutes, Hours, Day-Of-Week, Day, Month, Year
    //  uses readBytes into an array to read all the bytes at once
    readBytes(0x00, _byteValue, 7);

    // Seconds from register 0x00
    currentDate.Second = bcd2bin(_byteValue[0]);

    // Minutes from register 0x01
    currentDate.Minute = bcd2bin(_byteValue[1]);

    // Hour byte from register 0x02
    // Determine if 12 or 24h modes
    // 6th Bit of hour indicates 12/24 Hour mode
    // 5th Bit of hour indicates 10hr or AM=0/PM=1
    if(_byteValue[2] & _BV(6)) {
        // 12hr is enabled
        if (_byteValue[2] & _BV(5)) {
            //PM found
            currentDate.ClockMode = 1;
        } else {
            //AM found
            currentDate.ClockMode = 0;
        }
        //Mask out upper bits
        currentDate.Hour = bcd2bin(_byteValue[2] & 0B00011111);
    } else {
        //24hr found
        currentDate.ClockMode = 2;
        //Mask out upper bits
        currentDate.Hour = bcd2bin(_byteValue[2] & 0B00111111);
    }

    // Day of the week from register 0x03
    currentDate.Dow    = bcd2bin(_byteValue[3]);

    // dd date from register 0x04
    currentDate.Day    = bcd2bin(_byteValue[4]);

    // Month byte from register 0x05
    // bit 7 of month indicates the year century
    if (_byteValue[5] & _BV(7)) {
        currentDate.Year = 100;
    } else {
        currentDate.Year = 0;
    }
    currentDate.Month = bcd2bin(_byteValue[5] & 0B00011111);

    // Year from register 0x06
    currentDate.Year += bcd2bin(_byteValue[6]);

    return currentDate;
}

byte SimpleAlarmClock::write(const DateTime &currentDate){
    /** ******************************************************************
     * Writes the Clock values stored in a DataTime object into RTC's
     * registers.  Use setAlarm(i) to set alarms. Code inherited from
     * DS3231_Simple library.  To be updated.
     * Parameters:
     *    DataTime Oject with values to be stored
     *
     * Returns: (work in progress)
     *    0 - fail
     *    1 - success
     ****************************************************************** */
    byte _byteValue[7];

    //Need to set or validate DOW
    byte Dow = calcDow(currentDate.Month,currentDate.Day,2000+currentDate.Year);

    //Register 0x00
    _byteValue[0] = (bin2bcd(currentDate.Second));

    // Register 0x01
    _byteValue[1] = (bin2bcd(currentDate.Minute));

    //Register 0x02
    //Format Hour byte Bit 7 and 6 could be stripped from Hour value
    if (currentDate.ClockMode == 2){
        //24hr
        // set bits 7=0, 6/24hr=0
        _byteValue[2] = (bin2bcd(currentDate.Hour) & 0B00111111);
    } else {
        //12hr
        if (currentDate.ClockMode == 0){
            //AM
            //set bits 7=0, 5/AM=0
            _byteValue[2] = bin2bcd(currentDate.Hour) & 0B01011111;
            //set bite 6/12hr=1
            _byteValue[2] |= (1 << 6);
        } else {
            //PM
            //set bits 7=0, 6/12hr=1, 5/PM=1
            _byteValue[2] = bin2bcd(currentDate.Hour) & 0B01111111;
            _byteValue[2] |= (3 << 5);
        }
    }

    //Register 0x03
    _byteValue[3] = bin2bcd(Dow);

    //Register 0x04
    _byteValue[4] = bin2bcd(currentDate.Day);

    //Registers 0x05 and 0x06
    //Format Month byte's bit 7 century if available
    if (currentDate.Year <= 99) {
        //No Century bit
        _byteValue[5] = bin2bcd(currentDate.Month);
        _byteValue[6] = bin2bcd(currentDate.Year);
    } else {
        //Century bit found
        _byteValue[5] = (bin2bcd(currentDate.Month) | 0B10000000);
        _byteValue[6] = bin2bcd(currentDate.Year - 100);
        //Note: range for this byte is only 0-99
    }
    return writeBytes(0x00, _byteValue, 7);
}

AlarmTime SimpleAlarmClock::readAlarm(byte alarmSelected){
    /** ******************************************************************
     * readAlarm will read from RTC if SnoozingAx = false, else it will
     * read eeprom.
     * Parameters:
     *   alarmSelected = 1 or 2 for alarm1 or alarm2,
     *                   ignore invalid values
     *
     * Returns:
     *   AlarmTime object value
     ****************************************************************** */
    AlarmTime alarm_i;

    switch (alarmSelected){
        case 1:
            if (SnoozingA1==true) {
                alarm_i = readMemAlarm(1);
            } else {
                alarm_i = readRtcAlarm(1);
            }
            break;
        case 2:
            if (SnoozingA2==true) {
                alarm_i = readMemAlarm(2);
            } else {
                alarm_i = readRtcAlarm(2);
            }
            break;
        default:
          //do nothing
          break;
    }
    return alarm_i;
}

AlarmTime SimpleAlarmClock::readRtcAlarm(byte alarmSelected){
    /** ******************************************************************
     * Private: Called by readAlarm method.
     * Parameters:
     *   alarmSelected - 1 or 2 for alarm1 or alarm2
     *
     * AlarmTime structure:
     *   byte Second;       // 0-59 = 6 bits 0=for alarm2
     *   byte Minute;       // 0-59 = 6 bits
     *   byte Hour;         // 0-23 = 5 bits
     *   byte AlarmMode;    // 0=Daily, 1=Weekday, 2=Weekend, 3=Once
     *   byte ClockMode;    // 0-2; 0=AM, 1=PM, 2=24hour
     *   bool Enabled;      // true or false
     *
     * Returns:
     *   AlarmTime object
     *
     *TODO: error checking for readByte
     ****************************************************************** */
    AlarmTime alarm_i;
    byte _byteValue[4];
    byte _bValue;
    byte _address;
    
    // Alarm1 uses 4 registers Starting at 0x07
    // Alarm2 uses 3 registers Starting at 0x0b
    // To keep them even, Alarm2 will start at register 0x0a    
    if (alarmSelected == 1){ _address = 0x07; } else { _address = 0x0a; }
    readBytes(_address, _byteValue, 4);
    // clear out Alarm2 zero byte 
    if (alarmSelected == 2){_byteValue[0] = 0;}
    // Seconds
    alarm_i.Second = bcd2bin(_byteValue[0] & 0B01111111);

    // minutes byte
    // alarm1 = register 0x08
    // alarm2 = register 0x0b
    // 7th bit A1M2/A2M2 = 0
    alarm_i.Minute = bcd2bin(_byteValue[1] & 0B01111111);

    // Hour byte
    // alarm1 = register 0x09
    // alarm2 = register 0x0c
    // 7th Bit A1M3/A2M3 = 0
    // 6th Bit of hour indicates 12/24 Hour mode
    // 5th Bit of hour indicates 10hr or AM=0/PM=1
    if(_byteValue[2] & _BV(6)) {
        // 12hr is enabled
        if (_byteValue[2] & _BV(5)) {
            // PM found
            alarm_i.ClockMode = 1;
        } else {
            // AM found
            alarm_i.ClockMode = 0;
        }
        // mask out upper bits
        alarm_i.Hour = bcd2bin(_byteValue[2] & 0B00011111);
    } else {
        // 24hr found
        alarm_i.ClockMode = 2;
        // mask out upper bits
        alarm_i.Hour = bcd2bin(_byteValue[2] & 0B00111111);
    }

    // Day/Date byte
    // alarm1 = register 0x0a
    // alarm2 = register 0x0d
    // 7th Bit A1M4/A2M4 = 0 for day mode, = 1 for daily or once
    //     - we can ignore it here, but used for alarm purposes
    // 6th Bit DY/DT = 1 always
    // 5th & 4th Bits store 10Date value (0-3)
    // Four bits, bit 3-bit up to bit 0, store the next specific Day
    // that the alarm will go off, unless daily or once.
    // This is automatically handled by other routines, and can be
    // ignored here. We don't care about the specfic DAY.

    // Read AlarmMode
    // Relocated to EEprom register 0x08
    // alarm1 = bits 3&2
    // alarm2 = bits 1&0
    _bValue = readMem(0x08);
    if (alarmSelected == 1){
        _bValue &= 0B00000011;
    } else {
        _bValue = (_bValue >> 2);
        _bValue &= 0B00000011;
    }
    alarm_i.AlarmMode = (_bValue);

    // Get alarm enabled status from register 0x0e
    readByte(0x0e, _bValue);
    // mask out all but bit 0
    if (alarmSelected == 1){
        // alarm1
        if(_bValue & _BV(0)) {
            alarm_i.Enabled = true;
        } else {
            alarm_i.Enabled = false;
        }
    } else {
        // alarm2
        if(_bValue & _BV(1)) {
            alarm_i.Enabled = true;
        } else {
            alarm_i.Enabled = false;
        }
    }

    return alarm_i;
}

AlarmTime SimpleAlarmClock::readMemAlarm(byte alarmSelected){
    /** ******************************************************************
     * Private: Called by readAlarm method.
     * Parameters:
     *   alarmSelected - 1 or 2 for alarm1 or alarm2
     *
     * AlarmTime structure:
     *   byte Second;       // 0-59 = 6 bits 0=for alarm2
     *   byte Minute;       // 0-59 = 6 bits
     *   byte Hour;         // 0-23 = 5 bits
     *   byte AlarmMode;    // 0=Daily, 1=Weekday, 2=Weekend, 3=Once
     *   byte ClockMode;    // 0-2; 0=AM, 1=PM, 2=24hour
     *   bool Enabled;      // true or false
     *
     * Returns:
     *   AlarmTime object
     *
     *TODO: error checking for readByte
     ****************************************************************** */
    /* readAlarm1 will read from RTC if SnoozingA1 = false
       else, it will read eeprom alarm1 address
     */
    AlarmTime alarm_i;
    byte _byteValue;
    byte _address;

    // readeepromAlarm
    // Typically Seconds are ignored, However
    // Read Second byte from MEM 0x00
    // A1M1 = 0
    if (alarmSelected == 1){
        _byteValue = bcd2bin(readMem(0x00));
    } else {
        _byteValue = 0;
    }
    alarm_i.Second = _byteValue;

    // Read Minute
    // alarm1 from MEM 0x01
    // alarm2 from MEM 0x04
    // A1M2/A2M2 = 0
    if (alarmSelected == 1){ _address = 0x01; } else { _address = 0x04; }
    _byteValue = bcd2bin(readMem(_address));
    alarm_i.Minute = _byteValue;

    //Read Hour
    // alarm1 from Mem 0x02
    // alarm2 from Mem 0x05
    // 7th Bit A1M3 = 0
    // 6th Bit of hour indicates 12/24 Hour mode
    // 5th Bit - 10hr bit (ie 20) or AM=0/PM=1
    // 4th Bit - 10hr bit (ie 10)
    // 3dr Bit - hour
    // 2nd Bit - hour
    // 1st Bit - hour
    //   0 Bit - hour
    if (alarmSelected == 1){ _address = 0x02;} else { _address = 0x05;}
    _byteValue = readMem(_address);
    if(_byteValue & _BV(6)) {
        // 12hr is enabled
        if (_byteValue & _BV(5)) {
          //PM found
          alarm_i.ClockMode = 1;
        } else {
          //AM found
          alarm_i.ClockMode = 0;
        }
        alarm_i.Hour = bcd2bin(_byteValue & 0B00011111);  //mask out upper bits
    } else {
        //24hr found
        alarm_i.ClockMode = 2;
        alarm_i.Hour = bcd2bin(_byteValue & 0B00111111);  //mask out upper bits
    }

    //Read Day/Date byte
    // alarm1 = register 0x03
    // alarm2 = register 0x06
    // 7th Bit A1M4 = 0 for day mode, = 1 for daily or once
    //     - we can ignore it here, but used for alarm purposes
    // 6th Bit DY/DT = 1 always, assumed by Library
    // 5th Bit 10date Value
    // 4th Bit 10Date Value
    // 3rd Bit - Day
    // 2nd Bit - Day
    // 1st Bit - Day
    //   0 Bit - Day

    // Ok - Day is managed by the Library so we can ignore it

    // Read AlarmMode
    // Relocated to EEprom register 0x08
    // alarm1 = bits 3&2
    // alarm2 = bits 1&0
    _byteValue = readMem(0x08);
    if (alarmSelected == 1){
        _byteValue &= 0B00000011;
    } else {
        _byteValue = (_byteValue >> 2);
        _byteValue &= 0B00000011;
    }
    alarm_i.AlarmMode = (_byteValue);

    //Get alarm enabled status
    _byteValue = readMem(0x7);
    //Alarm1 is located in bit 0
    if (alarmSelected == 1){
        if(_byteValue & _BV(0)) {
            alarm_i.Enabled = true;
        } else {
            alarm_i.Enabled = false;
        }
    } else {
        if(_byteValue & _BV(1)) {
            alarm_i.Enabled = true;
        } else {
            alarm_i.Enabled = false;
        }
    }
    return alarm_i;
}

byte SimpleAlarmClock::nextAlarmDay(byte _AlarmMode, byte _ClockMode, byte _Hour, byte _Minute){
    /** ******************************************************************
     * Returns the next appropriate alarm day value
     * Parameters:
     *   _AlarmMode
     *   _ClockMode
     *   _Hour
     *   _Minute
     *
     * Returns:
     *   1=Sunday 2 3 4 5 6 7=Saturday
     ****************************************************************** */
    int CurrentX;
    int AlarmX;
    byte dayReturn;
    DateTime currentTime;
    currentTime = read();
    //These are not actual times, but are useful for comparison purposes
    // example: current: AM,08,35 = 0*10000+8*100+35 = 835
    //            alarm: PM,08,30 = 1*10000+8*100+30 = 10835
    //The alarm could occur that same day.
    CurrentX = currentTime.ClockMode*10000+currentTime.Hour*100+currentTime.Minute;
    AlarmX = _ClockMode*10000+_Hour*100+_Minute;

    //Find current Day value
    if (AlarmX > CurrentX){
        //Could happen later same day
        switch (_AlarmMode){
            case 0:
               //Daily
               dayReturn = currentTime.Dow;
               break;
            case 1:
               //WeekDay 2-6
               //If it's a weekday now
               if ((currentTime.Dow > 1)&&(currentTime.Dow < 7)){
                   dayReturn = currentTime.Dow;
               } else {
                   //It's a weekend now
                   dayReturn = 2;  //Monday
               }
               break;
            case 2:
               //WeekEnd
               //If it's a weekend now
               if ((currentTime.Dow == 1)||(currentTime.Dow == 7)){
                   dayReturn = currentTime.Dow;
               } else {
                   //it's a weekday now
                   dayReturn = 7;
               }
               break;
            case 3:
               //Once
               dayReturn = currentTime.Dow;
               break;
            default:
               //Return nothing
               break;
        }
    } else {
        //It's too late today what's the next day?
        switch (_AlarmMode){
            case 0:
               //Daily
               dayReturn = currentTime.Dow + 1;
               if (dayReturn == 8){dayReturn = 1;}
               break;
            case 1:
               //WeekDay
               dayReturn = currentTime.Dow + 1;
               if (dayReturn > 6){ dayReturn = 2;}
               break;
            case 2:
               //WeekEnd
               if ((currentTime.Dow < 7)||(currentTime.Dow == 1)) {
                   dayReturn = 7;
               } else {
                   dayReturn = 1;
               }
               break;
            case 3:
               //Once
               dayReturn = currentTime.Dow + 1;
               if (dayReturn == 8){dayReturn = 1;}
               break;
            default:
               //Return nothing
               break;
        }
    }
    return dayReturn;
}

byte SimpleAlarmClock::getCtrlRegister(void){
    /** ******************************************************************
     * Returns the entire Control register 0x0e
     * sets various local variables except alarm1&alarm2 enabled status
     ****************************************************************** */
    byte  _byteValue;

    readByte(0x0e, _byteValue);

    if(_byteValue & _BV(7)) {EOSC=1;}else{EOSC=0;}
    if(_byteValue & _BV(6)) {BBSQW=1;}else{BBSQW=0;}
    if(_byteValue & _BV(5)) {CONV=1;}else{CONV=0;}
    if(_byteValue & _BV(4)) {RS2=1;}else{RS2=0;}
    if(_byteValue & _BV(3)) {RS1=1;}else{RS1=0;}
    if(_byteValue & _BV(2)) {INTCN=1;}else{INTCN=0;}
    //last two bits are the alarms, which are ignored here
    return _byteValue;
}

byte SimpleAlarmClock::getStatusRegister(void){
    /** ******************************************************************
     * Returns the entire Status register 0x0f
     *
     ****************************************************************** */
    byte  _byteValue;

    readByte(0x0f, _byteValue);

    return _byteValue;
}

void SimpleAlarmClock::setEnableOscillator(bool Enable){
    /** ******************************************************************
     * Set or clear EOSC bit in register 0x0e
     *
     * EOSC = 0 - oscillator is started <default>
     *      = 1 - the oscillator is stopped when the DS3231 switches to VBAT
     *
     * When the DS3231 is powered by VCC, the oscillator is always
     * on regardless of the status of the EOSC bit.
     ****************************************************************** */
    byte _byteValue;
    _byteValue = getCtrlRegister();
    if (Enable == true){
        _byteValue &= ~(1<<7);
        EOSC = 0;
    } else {
        _byteValue |= (1<<7);
        EOSC = 1;
    }
    writeByte(0x0e, _byteValue);
}

void SimpleAlarmClock::setBatteryBackedSquareWave(bool Enable){
    /** ******************************************************************
     * Sets or clears the BBSQW bit on register 0x0e
     * BBSQW = 0 - the INT/SQW pin goes high impedance when VCC falls
     *             below the power-fail trip point. <default>
     *       = 1 - When set to logic 1 and the DS3231 is being
     *             powered by the VBAT pin, this bit enables the
     *            squarewave or interrupt output when VCC is absent.
     ****************************************************************** */
    byte _byteValue;
    _byteValue = getCtrlRegister();
    if (Enable = true) {
        _byteValue |= (1<<6);  //set bit to one
        BBSQW = 1;
    } else {
        _byteValue &= ~(1<<6); //set bit to zero
        BBSQW = 0;
    }
    writeByte(0x0e, _byteValue);
}

void SimpleAlarmClock::setConvertTemperature(void){
    /** ******************************************************************
     * Set CONV bit in register 0x0e, have to wait until bsy bit is
     * cleared first.
     * CONV = 1 - forces the temperature sensor to convert the temperature
     *            into digital code and execute the TCXO algorithm
     *            to update the capacitance array to the oscillator. Once
     *            completed, it returns to 0.
     *      = 0 - <default> Can't be set
     ****************************************************************** */
    byte _byteValue;
    byte x;

    _byteValue = getCtrlRegister();
    //Set the CONV bit
    _byteValue |= (1<<5);
    //Read Register 0x0f to get the bsy bit
    readByte(0x0f, x);
    //check the bsy bit
    while(x & _BV(2)) {
        //bsy bit is set so we must wait
        delay(10);
        readByte(0x0f, x);
    } //repeat until bsy is done
    writeByte(0x0e, _byteValue);
}

byte SimpleAlarmClock::setRateSelect(byte Data){
    /** ******************************************************************
     * Rate Select control the frequency of the square-wave output when
     * the square wave has been enabled. The following table
     * shows the square-wave frequencies that can be selected
     * with the RS bits. These bits are both set to logic 1
     * (8.192kHz) when power is first applied.
     * Table from Data sheet:
     * --------------------------------------------
     * | RS2 | RS1 | SQUARE-WAVE OUTPUT FREQUENCY |
     * --------------------------------------------
     * |  0  |  0  |   1 Hz                       |
     * --------------------------------------------
     * |  0  |  1  |   1.024kHz                   |
     * --------------------------------------------
     * |  1  |  0  |   4.096kHz                   |
     * --------------------------------------------
     * |  1  |  1  |   8.192kHz <default>         |
     * --------------------------------------------
     *
     * Returns:
     *     4 = Error
     *   0-3 = Data value
     ****************************************************************** */
    byte _byteValue;
    _byteValue = getCtrlRegister();
    if (Data <= 3){
        _byteValue |= (Data << 3);
        writeByte(0x0e, _byteValue);
        return(Data);
    } else {
        return(4);
    }
}

void SimpleAlarmClock::setInterruptCtrl(bool Enable){
    /** ******************************************************************
     * Sets of clears the INTCN bit in register 0x0e
     * INTCN = 0 - SQW pin output square wave.
     *       = 1 - SQW Stay HIGH, goes LOW only during alarm
     * Typically, set to 1 when first powered up
     ****************************************************************** */
    byte _byteValue;
    _byteValue = getCtrlRegister();
    if (Enable = true) {
        _byteValue |= (1<<2);  //set bit to one
    } else {
        _byteValue &= ~(1<<2); //set bit to zero
    }
    writeByte(0x0e, _byteValue);
}

byte SimpleAlarmClock::setAlarm(const AlarmTime &alarm_i, byte alarmSelected){
    /** ******************************************************************
     * sets to both RTC and MEM for either alarm1 or alarm2
     * Returns:
     *   returnValue = 0 failed
     *                 1 Success
     *                 2 wrong alarm index
     ****************************************************************** */
    byte returnValue = 0;
    switch (alarmSelected){
        case 1:
            returnValue = setRtcAlarm(alarm_i, alarmSelected);
            returnValue = setMemAlarm(alarm_i, alarmSelected);
            break;
        case 2:
            returnValue = setRtcAlarm(alarm_i, alarmSelected);
            returnValue = setMemAlarm(alarm_i, alarmSelected);
            break;
        default:
            //do nothing
            returnValue = 2;
            break;
    }
    return returnValue;
}

byte SimpleAlarmClock::setRtcAlarm(const AlarmTime &alarm_i, byte alarmSelected){
    /** ******************************************************************
     * Private called by setAlarm
     * Alarm settings - see Table 2 Alarm Mask Bits
     * --------------------------------------------------------------------
     * |DY/DT| A1M4| A1M3| A1M2| A1M1| Alarm1 Comments
     * --------------------------------------------------------------------
     * |DY/DT| A2M4| A2M3| A2M2|  X  | Alarm2 Comments (no seconds)
     * --------------------------------------------------------------------
     * |  X  |  1  |  1  |  1  |  1  | Alarm1 only once per second
     * --------------------------------------------------------------------
     * |  X  |  1  |  1  |  1  |  0  | Alarm1 second match
     * |     |  1  |  1  |  1  |  X  | Alarm2 only once per minute
     * --------------------------------------------------------------------
     * |  X  |  1  |  1  |  0  |  0  | Alarm minute (& second) match
     * --------------------------------------------------------------------
     * |  X  |  1  |  0  |  0  |  0  | Alarm hour, min, (& sec) match
     * --------------------------------------------------------------------
     * |  0  |  0  |  0  |  0  |  0  | Alarm date, hrs, min,(& sec) match
     * --------------------------------------------------------------------
     * |  1  |  0  |  0  |  0  |  0  | Alarm day, hrs, min,(& sec) match
     * --------------------------------------------------------------------
     ****************************************************************** */
    /* AlarmTime structure:
     byte Second;       // 0-59 = 6 bits 0=for alarm2
     byte Minute;       // 0-59 = 6 bits
     byte Hour;         // 0-23 = 5 bits
     byte AlarmMode;    // 0=Daily, 1=Weekday, 2=Weekend, 3=Once
     byte ClockMode;    // 0-2; 0=AM, 1=PM, 2=24hour
     bool Enabled;      // true or false
    ******************************************************************* */
    byte _byteValue[3];
    byte _address;
    byte _Dow;

    // Seconds
    // alarm1 = Register 0x07 only
    if (alarmSelected == 1) {
        _byteValue[0] = bin2bcd(alarm_i.Second & 0B01111111);
        writeByte(0x07, _byteValue[0]);       //A1M1=0
    }

    // Build the array - We skipped the seconds to keep arrays the same length
    // Minutes
    // alarm1 = Register 0x08
    // alarm2 = Register 0x0b
    // Here we reuse array [0]
    _byteValue[0] = bin2bcd(alarm_i.Minute & 0B01111111);

    // Hour
    // alarm1 = Register 0x09
    // alarm2 = Register 0x0c
    // Build hour byte
    if (alarm_i.ClockMode == 2){
        //24hour mode
        //A1M3=0,12/24=0
        _byteValue[1] = (bin2bcd(alarm_i.Hour) & 0B00111111);
    } else if (alarm_i.ClockMode == 0){
        //AM
        //A1M3=0,12/24=1,AM/PM=0
        _byteValue[1] = (bin2bcd(alarm_i.Hour) & 0B01011111);
        _byteValue[1] |= (1 << 6);
    } else {
        //PM
        //A1M3=0,12/24=1,AM/PM=1
        _byteValue[1] = (bin2bcd(alarm_i.Hour) & 0B01111111);
        _byteValue[1] |= (3 << 5);
    }

    // DOW
    // alarm1 = 0x0a
    // alarm2 = 0x0d
    // Build day/date byte - Sorry always DAY (DOW) never DATE (dd)
    // Get the next valid DOW
    _Dow = nextAlarmDay(alarm_i.AlarmMode, alarm_i.ClockMode, alarm_i.Hour, alarm_i.Minute);
    _byteValue[2] = bin2bcd(_Dow);

    // Based on AlarmMode
    // 0=Daily, 1=Weekday, 2=Weekend, 3=Once
    // If Daily make A1M4/A2M4 = 1
    // Else
    // If Weekday, Weekend, or Once A1M4/A2M4 =0 DY/DT = 1
    if (alarm_i.AlarmMode == 0) {
        //A1M4/A2M4 = 1, DY/DT = X
        _byteValue[2] |= (1<<7);
    } else {
        //A1M4/A2M4 = 0, DY/DT = 1
        _byteValue[2] |= (1<<6);
    }
    
    // Begin writing the array to I2C 
    if (alarmSelected == 1) { _address = 0x08; } else { _address = 0x0b;}
    writeBytes(_address, _byteValue, 3);

    // Write AlarmMode
    // Relocated to EEprom register 0x08
    // alarm1 = bits 3&2
    // alarm2 = bits 1&0
    // build the byte First by getting the current values
    _byteValue[0] = readMem(0x08);
    if (alarmSelected == 1){
        _byteValue[0] &= 0B11111100;
        _byteValue[0] |= alarm_i.AlarmMode;
    } else {
        _byteValue[0] &= 0B11110011;
        _byteValue[0] |= (alarm_i.AlarmMode << 2);
    }
    writeMem(0x08, _byteValue[0]);

    // Use armAlarm method to set register 0x0e
    armAlarm(alarmSelected, alarm_i.Enabled);

    return 1;  // for future error return codes
}

byte SimpleAlarmClock::setMemAlarm(const AlarmTime &alarm_i, byte alarmSelected){
    /** ******************************************************************
     * Private called by setAlarm
     ****************************************************************** */
    byte _byteValue[3];
    byte _address;
    byte _Dow;

    // Seconds are typically ignored by this library, however
    // Write Second byte
    // alarm1 = MEM 0x00
    // A1M1=0
    if (alarmSelected == 1){
        _byteValue[0] = bin2bcd(alarm_i.Second);
        writeMem(0x00, _byteValue[0]);
    }

    // Build array
    // Write Minute byte
    // alarm1 = MEM 0x01
    // alarm2 = MEM 0x04
    // First byte = minutes
    // A1M2=0
    // Here we reuse array [0]
    _byteValue[0] = bin2bcd(alarm_i.Minute);

    // Write Hour byte
    // alarm1 = MEM 0x02
    // alarm2 = MEM 0x05

    //First build hour byte
    // 7th Bit A1M3 = 0
    // 6th Bit of hour indicates 12/24 Hour mode
    // 5th Bit - 10hr bit (ie 20) or AM=0/PM=1
    // 4th Bit - 10hr bit (ie 10)
    // 3dr Bit - hour
    // 2nd Bit - hour
    // 1st Bit - hour
    //   0 Bit - hour
    if (alarm_i.ClockMode == 2){
        //24hour mode
        _byteValue[1] = bin2bcd(alarm_i.Hour) & 0B00111111;  //A1M3=0,12/24=0
    } else if (alarm_i.ClockMode == 0){
        //AM
        _byteValue[1] = bin2bcd(alarm_i.Hour) & 0B01011111 | (1 << 6);  //A1M3=0,12/24=1,AM/PM=0
    } else {
        //PM
        _byteValue[1] = bin2bcd(alarm_i.Hour) & 0B01111111 | (3 << 5);  //A1M3=0,12/24=1,AM/PM=1
    }

    //DOW Byte
    // alarm1 = MEM 0x03
    // alarm2 = MEM 0x06
    //Build Byte DOW - Sorry always DAY (DOW) never DATE (dd)
    //  use 10Date bits to store AlarmMode
    _Dow = nextAlarmDay(alarm_i.AlarmMode, alarm_i.ClockMode, alarm_i.Hour, alarm_i.Minute);

    //Add BCD cpnverted DOW
    _byteValue[2] = bin2bcd(_Dow);

    // Based on the AlarmMode
    // 0=Daily, 1=Weekday, 2=Weekend, 3=Once
    // If Daily make A1M4/A2M4 = 1
    // Else
    // If Weekday, Weekend, or Once A1M4/A2M4 =0 DY/DT = 1
    if (alarm_i.AlarmMode == 0) {
        //A1M4/A2M4 = 1, DY/DT = X
        _byteValue[2] |= (1<<7);
    } else {
        //A1M4/A2M4 = 0, DY/DT = 1
        _byteValue[2] |= (1<<6);
    }

    if (alarmSelected == 1){_address = 0x01;} else { _address = 0x04; }
    writeMem(_address, _byteValue, 3);

    //Technically, the Alarm mode was just written by setRtcAlarm
    //So perhaps no need to repeat it here.
    // Write AlarmMode
    // Relocated to EEprom register 0x08
    // alarm1 = bits 3&2
    // alarm2 = bits 1&0
    // build the byte First by getting the current values
    //_byteValue = readMem(0x08);
    //if (alarmSelected == 1){
    //    _byteValue &= 0B11111100;
    //    _byteValue |= alarm_i.AlarmMode;
    //} else {
    //    _byteValue &= 0B11110011;
    //    _byteValue |= (alarm_i.AlarmMode << 2);
    //}
    //writeMem(0x08, _byteValue);


    //Read Status byte 0x07
    _byteValue[0] = readMem(0x07);
    _byteValue[0] &= 0B00000111;              //mask out any garbage
    if (alarmSelected == 1){
        if (alarm_i.Enabled == true){
            //Set the bit
            _byteValue[0] |= 1;
        } else {
            //Clear/mask the bit
            _byteValue[0] &= 0B11111110;
        }
    } else {
        if (alarm_i.Enabled == true){
            //Set the bit
            _byteValue[0] |= (1 << 1);
        } else {
            //Clear/mask the bit
            _byteValue[0] &= 0B11111101;
        }
    }
    //Set the Alarm Enabled bit
    writeMem(0x07, _byteValue[0]);

    return 1;  // for future error return codes
}

void SimpleAlarmClock::armAlarm(byte alarmSelected, bool Enable) {
    /** ******************************************************************
     * armAlarm - enables or disables an alarm
     *   alarmSelected = 1 or 2 for alarm1 or alarm2
     *   Enable = true or false to enable or disable alarm
     *
     * Address 0x0E
     * -------------------------------------------------------------------
     * | EOSC |BBSQW| CONV | RS2 | RS1 |INTCN| A2IE| A1IE|Comments
     * -------------------------------------------------------------------
     * |  0   |  0  |  0   |  1  |  1  |  1  |  0  |  0  | Default Powerup
     * -------------------------------------------------------------------
     * EOCS    = Enable Oscillator logic 0
     * BBSQW   = Battery-backed Square-Wave Enable 0 off 1 on
     * CONV    = Convert Temperature Enable TCXO algorithm to update
     *           cappacitance array to oscillator.
     * RS2&RS1 = Square wave output selectAlarm
     * INTCN   = Interrupt Control for INT\SQW pin. Set to 1 to active
     *           pin when alarm occurs. 0 enables square wave.
     * A2IE    = Alarm 2 Interrupt Enable. 1=Enable 0=Disable
     * A1IE    = Alarm 1 Interrupt Enabel. 1=Enable 0=Disable
     * Enable Alarm =>  enabling A1IE or A2IE and INTCN bit
     ****************************************************************** */
    byte _byteValue;
    byte returnValue = 1;

    _byteValue = getCtrlRegister();
    //TODO: Clean up code that forces INTCN = 1
    //confirm that INTCN = 1
    _byteValue |= (1 << 2);    //uses compound bitwise OR operator shift register trick
    INTCN = 1;
    //Now enable/disable alarm1 or alarm2
    switch (alarmSelected){
    case 1:
        //Alarm1
        if (Enable) {
          //Set to true
          //uses compound bitwise OR operator shift register trick
          _byteValue |= (1);
        } else {
          //Set to false
          //uses compound bitwise inverted AND operator shift register trick
          _byteValue &= ~(1);
        }
        break;
    case 2:
        //Alarm2
        if (Enable) {
          //Set to true
          //uses compound bitwise OR operator shift register trick
          _byteValue |= (1 << 1);
        } else {
          //Set to false
          //uses compound bitwise inverted AND operator shift register trick
          _byteValue &= ~(1 << 1);
        }
        break;
    default:
        returnValue = 0;
        break;
    }

    //Write to Registry
    if (returnValue == 1){writeByte(0x0e, _byteValue);}
    //TODO: write to eeprom
}

void SimpleAlarmClock::clearAlarms(void){
    /** ******************************************************************
     * Clears all alarm Control Status flags,
     * - reset RTC alarm(s) if SnoozeAx active
     * - set SnoozingA1 or SnoozingA2 to false
     * - use it to acknowledge or clear alarms flags
     * - disabled Alarm with Once AlarmMode
     *
     * Address 0x0F
     * --------------------------------------------------
     * |  B7 | B6 | B5 | B4 |  Bit 3  |  B2 |  B1 |  B0 |
     * -------------------------------------------------------------------
     * | OSF |  0 |  0 |  0 | EN32kHz | BSY | A2F | A1F |Comments
     * -------------------------------------------------------------------
     * |  X  |  0 |  0 |  0 |    1    |  X  |  0  |  0  | Default PowerUp
     * -------------------------------------------------------------------
     ****************************************************************** */
    // 0=Daily, 1=Weekday, 2=Weekend 3=Once
    // Clears any alarms
    AlarmTime alarm_i;
    byte _byteValue = 0;

    //Read Registry 0x0f by using getStatusRegister method
    _byteValue = getStatusRegister();
    //Clear Snoozing Status
    //and reset RTC alarm for next use
    if (_byteValue & _BV(0)) {
        //A1F Found
        if (SnoozingA1 == true){
            alarm_i = readMemAlarm(1);
            SnoozingA1 = false;
        } else {
            alarm_i = readAlarm(1);
        }
        //Once Alarm should automatically disable itself.
        if (alarm_i.AlarmMode == 3) { alarm_i.Enabled = false;}
        setAlarm(alarm_i, 1);  //Sets both RTC and MEM with new DOW
    }
    if (_byteValue & _BV(1)) {
        if (SnoozingA2 == true){
            alarm_i = readMemAlarm(2);
            SnoozingA2 = false;
        } else {
            alarm_i = readAlarm(2);
        }
        //Once Alarm should automatically disable itself.
        if (alarm_i.AlarmMode == 3) { alarm_i.Enabled = false;}
        setAlarm(alarm_i, 2);  //Sets both RTC and MEM with new DOW
    }

    _byteValue &= ~(0x3);                //clears both A2F and A1F bits
    //Note: 3 = 0B00000011, and by inverting it we get 0B1111100
    writeByte(0x0f, _byteValue);
}

byte SimpleAlarmClock::alarmStatus(void){
    /** ******************************************************************
     * Reads alarm enabled or disabled settings - Bits 1 & 0
     * Returns:
     *    0 - No alarms
     *    1 - Alarm 1 enabled
     *    2 - Alarm 2 enabled
     *    3 - Both alarms enabled
     *
     * Address 0x0E
     * -----------------------------------------------------
     * |  B7  |  B6 |  B5  |  B4 |  B3 |  B2 |  B1  |  B0  |
     * --------------------------------------------------------------------
     * | EOSC |BBDWQ| CONV | RS2 | RS1 |INTCN| A2IE | A1IE | Comments
     * --------------------------------------------------------------------
     * |   0  |  0  |   0  |  1  |  1  |  1  |  ?   |  ?   | Default PowerUp
     * --------------------------------------------------------------------
     ****************************************************************** */
    byte _byteValue = 0;
    readByte(0x0e, _byteValue);
    _byteValue &= 0B00000011;
    return (_byteValue);
}


byte SimpleAlarmClock::flaggedAlarms(void){
    /** ******************************************************************
     * Check to see if the actual alarm flags are triggered
     * Looking at Registry 0x0F - Bits 1 & 0
     * Use alarmStatus to check if alarms are enabled
     * Returns:
     *  0 - No alarms
     *  1 - Alarm 1 enabled
     *  2 - Alarm 2 enabled
     *  3 - Both alarms enabled
     ****************************************************************** */
    byte _byteValue = 0;
    readByte(0x0f,_byteValue);
     _byteValue &= 0B00000011;
    return (_byteValue);
}

bool SimpleAlarmClock::checkMemAlarm(byte alarmSelected = 1){
    /** ******************************************************************
     * Used to check eeprom Alarm1 memory area for valid alarm
     * Parameters:
     *   alarmSelected - 1 checks alarm 1 <default>
     *                 - 2 checks alarm 2
     * Returns:
     *   true  - Past checks OK
     *   false - Error found
     ****************************************************************** */
    AlarmTime alarm_i;
    bool returnValue = true;

    //Check for valid alarmSelected
    switch (alarmSelected){
        case 1:
        case 2:
            alarm_i = readMemAlarm(alarmSelected);
            break;
        default:
            returnValue = false;
            break;
    }
    if (returnValue == true){
        //Add additional checks here:
        if ((alarm_i.Hour > 12)&&(alarm_i.ClockMode != 2)) { returnValue = false; }
        if ((alarm_i.Hour == 0)&&(alarm_i.ClockMode != 2)) { returnValue = false; }
    }
    return returnValue;
}

byte SimpleAlarmClock::snoozeAlarm(byte alarmSelected = 1, byte SnoozeTime = 9){
    /** ******************************************************************
     * Used to snooze the alarm
     *    Parameters:
     *      alarmSelected - 2 for alarm 2, 1 for alarm 1 <default>
     *      SnoozeTime - length of time in munites added to alarm, max 255 minutes
     *    Returns:
     *      0 - Fail
     *      1 - Success
     *      2 - Error
     ****************************************************************** */
    AlarmTime alarm_i;
    byte returnValue = 1;
    byte _byteValue;

    switch (alarmSelected){
        case 1:
            SnoozingA1 = true;
            alarm_i = readRtcAlarm(alarmSelected);
            //Remove alarm flag
            readByte(0x0f, _byteValue);
            _byteValue &= ~(1);
            writeByte(0x0f, _byteValue);
            break;
        case 2:
            SnoozingA2 = true;
            alarm_i = readRtcAlarm(alarmSelected);
            //Remove alarm flag
            readByte(0x0f, _byteValue);
            _byteValue &= ~(1<<1);
            writeByte(0x0f, _byteValue);
            break;
        default:
            returnValue = 2;
            break;
    }

    if (returnValue != 2){
        alarm_i.Minute += SnoozeTime;
        if (alarm_i.Minute > 59) {alarm_i.Hour += 1;}
        alarm_i.Minute %= 60;
        if (alarm_i.ClockMode != 2) {
            alarm_i.Hour %= 12;
            if (alarm_i.Hour == 0){alarm_i.Hour = 12;}
        } else {
            alarm_i.Hour %= 24;
        }
        setRtcAlarm(alarm_i, alarmSelected);
    }
    return returnValue;
}

byte SimpleAlarmClock::writeEeprom(byte address, byte dataBuffer){
    /** ******************************************************************
     * Write MEM byte - must avoid addresses between 0x00 through 0x07
     * Parameters:
     *   address - address location to write to as a byte
     *      dataBuffer - the byte value to write
     * Returns:
     *         0 - Fail
     *         1 - No errors
     ****************************************************************** */
     if (address > 0x08) {
         writeMem(address, dataBuffer);
         return(1);
     } else {
         return(0);
     }
}

void SimpleAlarmClock::writeMem(byte address, byte dataBuffer) {
    /** ******************************************************************
     * Write MEM byte
     ****************************************************************** */
    Wire.beginTransmission(EEPROM_ADDRESS);
    if(Wire.endTransmission()==0) {
        Wire.beginTransmission(EEPROM_ADDRESS);
        Wire.write(address >> 8);
        Wire.write(address & 0xFF);
        Wire.write(dataBuffer);
        Wire.endTransmission();
        delay(10);
    }
}


void SimpleAlarmClock::writeMem(byte address, byte dataBuffer[], byte arrayLength) {
    /** ******************************************************************
     * Write sequence of n bytes
     ****************************************************************** */
    int remainingBytes = arrayLength;   // bytes left to write
    int offsetDataBuffer = 0;           // current offset in dataBuffer pointer
    int offsetPage;                     // current offset in page
    int nextByte = 0;                   // next n bytes to write

    // write alle bytes in multiple steps
    while (remainingBytes > 0) {
        // calc offset in page
        offsetPage = address % pageSize;
        // maximal 30 bytes to write
        nextByte = min(min(remainingBytes, 30), pageSize - offsetPage);
        Wire.beginTransmission(EEPROM_ADDRESS);
        if (Wire.endTransmission() == 0) {
            Wire.beginTransmission(EEPROM_ADDRESS);
            Wire.write(address >> 8);
            Wire.write(address & 0xFF);
            byte *adr = dataBuffer + offsetDataBuffer;
            Wire.write(adr, arrayLength);
            Wire.endTransmission();
            delay(10);
        }
    //write(address, dataBuffer, offsetDataBuffer, nextByte);
        remainingBytes -= nextByte;
        offsetDataBuffer += nextByte;
        address += nextByte;
    }
}

byte SimpleAlarmClock::readMem(byte address) {
    /** ******************************************************************
     * Read MEM byte
     ****************************************************************** */
    byte _byteValue = 0;
    int r = 0;
    Wire.beginTransmission(EEPROM_ADDRESS);
    if (Wire.endTransmission()==0) {
        Wire.beginTransmission(EEPROM_ADDRESS);
        Wire.write(address >> 8);
        Wire.write(address & 0xFF);
        if (Wire.endTransmission()==0) {
            Wire.requestFrom(int(EEPROM_ADDRESS), 1);
            while (Wire.available() > 0 && r < 1) {
                _byteValue = (byte)Wire.read();
                r++;
            }
        }
    }
    return _byteValue;
}

byte SimpleAlarmClock::readMem(byte address, byte dataBuffer[], byte arrayLength) {
    /** ******************************************************************
     * Read sequence of arrayLength bytes
     ****************************************************************** */
    int remainingBytes = arrayLength;
    int offsetDataBuffer = 0;
    // read until are arrayLength bytes read
    while (remainingBytes > 0) {
        // read maximum 32 bytes
        int nextByte = remainingBytes;
        if (nextByte > 32) { nextByte = 32;}
        Wire.beginTransmission(EEPROM_ADDRESS);
        if (Wire.endTransmission() == 0) {
            Wire.beginTransmission(EEPROM_ADDRESS);
            Wire.write(address >> 8);
            Wire.write(address & 0xFF);
            if (Wire.endTransmission() == 0) {
                int r = 0;
                Wire.requestFrom(EEPROM_ADDRESS, arrayLength);
                while (Wire.available() > 0 && r < arrayLength) {
                    dataBuffer[offsetDataBuffer + r] = (byte)Wire.read();
                    r++;
                }
            }
        }
        address += nextByte;
        offsetDataBuffer += nextByte;
        remainingBytes -= nextByte;
    }
    return arrayLength;
}

void SimpleAlarmClock::toggleClockMode(void){
    /** ******************************************************************
     * Toggle Clock Mode - use it to set the clock mode for current time
     *                     and all the alarms
     *   AM = 0
     *   PM = 1
     * 24hr = 2
     ****************************************************************** */
    DateTime currentTime;
    AlarmTime alarm1;
    AlarmTime alarm2;

    //Read currentTime
    //Read alarm1
    //Read alarm2
    currentTime = read();
    alarm1 = readAlarm(1);
    alarm2 = readAlarm(2);

    //Switch if the currentTime is:
    switch (currentTime.ClockMode){
        case  0:  //if AM and switching to 24
            //AM
            currentTime.ClockMode = 2;
            if (alarm1.ClockMode==1){alarm1.Hour += 12;}
            if (alarm2.ClockMode==1){alarm2.Hour += 12;}
            alarm1.ClockMode = 2;
            alarm2.ClockMode = 2;
            break;
        case 1:  //if PM and switching to 24
            //PM
            currentTime.ClockMode = 2;
            if (alarm1.ClockMode==1){alarm1.Hour += 12;}
            if (alarm2.ClockMode==1){alarm2.Hour += 12;}
            alarm1.ClockMode = 2;
            alarm2.ClockMode = 2;
            break;
        case 2:  //if 24 and switching to 12hr (AM/PM)
            //24
            if (currentTime.Hour >= 12){
              //PM found
              currentTime.ClockMode = 1;
              currentTime.Hour -= 12;
            } else {
              //AM Found
              currentTime.ClockMode = 0;
            }
            if (alarm1.Hour >=12) {
              //PM found
              alarm1.ClockMode = 1;
              alarm1.Hour -= 12;
            } else {
              //AM Found
              alarm1.ClockMode = 0;
            }
            if (alarm2.Hour >=12) {
              //PM found
              alarm2.ClockMode = 1;
              alarm2.Hour -= 12;
            } else {
              //AM Found
              alarm2.ClockMode = 0;
            }
            break;
        default:
            //do nothing
            break;
    }
    write(currentTime);
    setAlarm(alarm1,1);
    setAlarm(alarm2,2);
}

bool SimpleAlarmClock::getOSFStatus(void){
    /** ******************************************************************
     * Oscillator Stop Flag (OSF) value
     * This bit is set to logic 1 any time that the oscillator stops.
     * The following are examples of conditions that can cause the OSF bit
     * to be set:
     *      1) The first time power is applied.
     *      2) The voltages present on both VCC and VBAT are insufficient
     *         to support oscillation.
     *      3) The EOSC bit is turned off in battery-backed mode.
     *      4) External influences on the crystal (i.e., noise, leakage,
     *         etc.).
     *  This bit remains at logic 1 until written to logic 0.
     ****************************************************************** */
    byte _byteValue;
    readByte(0x0f, _byteValue);
    if (_byteValue & _BV(7)){return(true);}else{return(false);}
}

byte SimpleAlarmClock::clearOSFStatus(void){
    /** ******************************************************************
     * Oscillator Stop Flag (OSF)
     *   Clears OSF bit to a logic 0
     *   This bit remains at logic 1 until written to logic 0.
     ****************************************************************** */
    byte _byteValue;
    readByte(0x0f, _byteValue);
    _byteValue &= ~(1<<7);
    return(writeByte(0x0f, _byteValue));
}

bool SimpleAlarmClock::getEN32kHz(void){
    /** ******************************************************************
     * 32kHz Output (EN32kHz) bit value
     *  This bit controls the status of the 32kHz pin. When set to logic 1,
     *  the 32kHz pin is enabled and outputs a 32.768kHz square-wave signal.
     *  When set to logic 0, the 32kHz pin goes to a high-impedance state.
     *  The initial power-up state of this bit is logic 1, and a 32.768kHz
     *  square-wave signal appears at the 32kHz pin after a power source is
     *  applied to the DS3231 (if the oscillator is running).Clears OSF bit
     *  to a logic 0.
     ****************************************************************** */
    byte _byteValue;
    readByte(0x0f, _byteValue);
    if (_byteValue & _BV(3)){return(true);}else{return(false);}
}

byte SimpleAlarmClock::setEN32kHz(bool Enable){
    /** ******************************************************************
     * Enable 32kHz Output (EN32kHz) bit
     *  This bit controls the status of the 32kHz pin. When set to logic 1,
     *  the 32kHz pin is enabled and outputs a 32.768kHz square-wave signal.
     *  When set to logic 0, the 32kHz pin goes to a high-impedance state.
     *  The initial power-up state of this bit is logic 1, and a 32.768kHz
     *  square-wave signal appears at the 32kHz pin after a power source is
     *  applied to the DS3231 (if the oscillator is running).Clears OSF bit
     *  to a logic 0.
     ****************************************************************** */
    byte _byteValue;
    readByte(0x0f, _byteValue);
    if (Enable==true){
        _byteValue |= (1<<3);
    } else {
        _byteValue &= ~(1<<3);
    }
    return(writeByte(0x0f, _byteValue));
}

bool SimpleAlarmClock::busy(void){
    /** ******************************************************************
     * Busy (BSY). Checks the BSY bit in register 0x0f. This bit indicates
     * the device is busy executing TCXO functions. It goes to logic 1
     * when the conversion signal to the temperature sensor is asserted
     * and then is cleared when the device is in the 1-minute idle state.
     ****************************************************************** */
    byte _byteValue;
    readByte(0x0f, _byteValue);
    if (_byteValue & _BV(2)){return(true);}else{return(false);}
}

byte SimpleAlarmClock::getTemperature(void){
    /** ******************************************************************
     * Temperature is broken up into Most Significant Bit (MSB) and
     * Least Significant Bit (LSB).  The LSB are the number of .25 degree
     * above the whole number.  This method returns a Whole number using
     * "Bankers Rounding" such the 0.5 (.25 *2) is rounded up to the
     * nearest whole number.
     * Returns:
     *   Temperature as byte. No decimal.
     ****************************************************************** */
    byte temperature = 0;
    byte LSB = 0;

    readByte(0x11, temperature);                      //Reads the MSB
    readByte(0x12, LSB);                              //Reads the LSB
    LSB = (LSB >> 6);                                 //Shift bits
    if (LSB >= 2) { temperature++; }                  //Roundup
    return temperature;
}

float SimpleAlarmClock::getTemperatureFloat(void){
    /** ******************************************************************
     * Temperature is broken up into Most Significant Bit (MSB) and
     * Least Significant Bit (LSB).  The LSB are the number of .25 degree
     * above the whole number.  This method returns a float value.
     * Returns:
     *   Temperature as a float
     ****************************************************************** */
    float temperature = 0;
    byte _byteValue;
    byte LSB = 0;

    readByte(0x11, _byteValue);                       //Reads the MSB
    readByte(0x12, LSB);                              //Reads the LSB
    LSB = (LSB >> 6);                                 //Shift bits
    temperature = _byteValue + (LSB * 0.25);          //Reads the LSB
    return temperature;
}

byte SimpleAlarmClock::calcDow(byte mm, byte dd, uint16_t yyyy){
    /** ******************************************************************
     * Returns the day of the week
     * https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week
     * this formula good for years greater than 1752
     * returns a number between 1-7 with 1 = Sunday
     ****************************************************************** */
    static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    yyyy -= mm < 3;
    return ((yyyy + yyyy/4 - yyyy/100 + yyyy/400 +t[mm - 1] + dd) % 7) + 1;
}

byte SimpleAlarmClock::getAgingOffset(void){
    /** ******************************************************************
     * Returns the crystal aging offset register value
     * (two’s complement) Check Datasheet to adjust this properly.
     ****************************************************************** */
    byte _byteValue = 0;
    
    readByte(0x10, _byteValue);
    return (_byteValue);    
}

byte SimpleAlarmClock::setAgingOffset(int changeValue){
    /** ******************************************************************
     * The adjust the crystal aging offset register
     * Check Datasheet to adjust this properly.
        The crystal aging offset register provides an 8-bit code
        to add to the codes in the capacitance array registers.
        The code is encoded in two’s complement. One LSB
        represents one small capacitor to be switched in or out
        of the capacitance array at the crystal pins. The offset
        register is added to the capacitance array register
        under the following conditions: during a normal temperature
        conversion, if the temperature changes from the
        previous conversion, or during a manual user conversion
        (setting the CONV bit). To see the effects of the
        aging register on the 32kHz output frequency immediately,
        a manual conversion should be started after each
        aging register change.
        Positive aging values add capacitance to the array,
        slowing the oscillator frequency. Negative values
        remove capacitance from the array, increasing the
        oscillator frequency.
        The change in ppm per LSB is different at different temperatures.
        The frequency vs. temperature curve is shifted
        by the values used in this register. At +25°C, one LSB
        typically provides about 0.1ppm change in frequency.
     * Parameter:
     *   changeValue = -128 through 127 (two’s complement)
     *                 this is automatically handled by method
     * Returns:
     *   0 = fail
     *   1 = writeByte failed
     *   2 = changeValue to big
     ****************************************************************** */
    byte returnValue = 0;

    if ((changeValue >= -128)&&(changeValue < 127)){
        byte i = writeByte(0x10, byte(changeValue));
        if (i > 0){returnValue = 1;}
    } else {
        returnValue = 2;
    }
    return (returnValue);
}

/** **********************************************************************
   Binary-Coded Decimal (bcd) and Binary (bin) format routines:

   Page 11 of Datasheet, it indicates that the contents of the time
   and calendar registers are in the binary-coded decimal (BCD) format.

   Now it gets complicated. Reading the time or calendar values, a mask
   may be necessary to clear some of the upper bits and then converted
   to binary.  Writing to thoses same regisries require converting the
   values back to BCD format, and adding any missing supporting bits.
*********************************************************************** */
/*  Example conversion:
    byte bcdToDec(byte val){
      return( (val/16*10) + (val%16) );
    }
    byte decToBcd(byte val){
      return( (val/10*16) + (val%10) );
    }
*/
byte SimpleAlarmClock::bcd2bin (byte val) { return ((val >> 4) * 10) + (val & 0x0F); }
byte SimpleAlarmClock::bin2bcd (byte val) { return (val / 10) << 4 | (val % 10);     }
