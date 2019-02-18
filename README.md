# SimpleAlarmClock Library

Alarm Clock library that uses the real time clock of a common ZS-042 module, aka. DS3231, with a AT24c32N EEprom and CR3032 battery.

## Information

A library for the **ZS-042** module that has a **DS3231** RTC and **AT24C32** EEPROM. 

An easy set of read/write functions or methods for the DS3231 Real Time Clock and Calendar, and the companion AT24C32 EEPROM on common ZS-042 modules.  The EEPROM is used by this library to store alarm settings on registers 0x00 through 0x08 as byte values.

The remainder of the EEPROM can be used for logging or storage.

The EEPROM read and write bytes methods were copied from this Library [The AT24CX Library](https://github.com/cyberp/AT24Cx)

The intent of this library is to manage a basic alarm clock, that includes the display clock and two alarms, typically seen on standard desktop/nightstand alarm clocks.  It should accomidate switching between 12 hour and 24 modes on the fly.  Alarms have four alarm modes: Daily, Weekday, Weekend or Once.  This library assumes that alarm DOW will only be used, and alarm DATE shall not be used. The alarm modes are stored in EEPROM address 0x08.  Snooze option is available for both alarms.

This library also makes the assumption that if the year has been set to 2018 (018) or greater, the RTC clock is probably valid. This approach is used so that during a power cycle (blackout), clock settings saved by the RTC's backup battery will not be reset.

Copyright (C) 2018 Ricardo Moreno Jr.

Originally inspired by DS3231_Simple by 2016 James Sleeman, this forked version is incompatible with original. However, since a small amount of code is used the following notice is included:

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

AT24C32 EEPROM Read and Write byte code copied from AT24Cx Library by Christian Paul. Modified by Ricardo Moreno.hub.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.

## Properties and Methods

### New structures:
- DateTime
- AlarmTime

### CONSTRUCTOR:
- SimpleAlarmClock(byte _rtc_Address=0x68, byte _eeprom_Address=0x57, bool alarmIntEnabled=true);

### Main Clock Metods:
- DateTime read();
- byte write(const DateTime&);

### Main Alarm Methods:
- AlarmTime readAlarm(byte alarmSelected);
- byte setAlarm(const AlarmTime&, byte alarmSelected);
- byte snoozeAlarm(byte alarmSelected, byte SnoozeTime);
- void armAlarm(byte alarmSelected, bool Enable);
- void clearAlarms();
- byte alarmStatus();

### Control Register 0x0e bit fiddlers:
- byte getCtrlRegister();
- void setEnableOscillator(bool Enable);
- void setBatteryBackedSquareWave(bool Enable);
- void setConvertTemperature();
- byte setRateSelect(byte Data);
- void setInterruptCtrl(bool Enable);

### Status Register 0x0f bit fiddlers:
- byte getStatusRegister();
- bool getOSFStatus();
- byte clearOSFStatus();
- bool getEN32kHz();
- byte setEN32kHz(bool Enable);
- bool busy()
- byte flaggedAlarms();

### Temperature Registers 0x11 and 0x12 methods:
- byte getTemperature();
- float getTemperatureFloat();

### Misc Methods:
- void toggleClockMode();
- byte calcDow(byte mm, byte dd, uint16_t yyyy);
- void resetClock(void);
- void resetAlarm(byte alarmSelected);

### Lower Level methods:
- byte readByte(byte Address,  byte &dataBuffer);
- byte readBytes(byte Address, byte dataBuffer[], byte Length);
- byte getAgingOffset(void);
- byte setAgingOffset(int changeValue);

### EEProm methods:
- byte readMem(byte address);
- byte readMem(byte address, byte dataBuffer[], byte n);
- byte writeEeprom(byte address, byte dataBuffer);
