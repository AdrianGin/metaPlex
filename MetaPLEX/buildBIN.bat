cd default

SET APP_END=0xD6FE
SET PROFILE_START=0xD700

REM FOR EEPROM
REM SET APP_END=0x6F7E
REM SET PROFILE_START=0x6F80

avr-objcopy -Iihex MetaPlex.hex -Obinary MetaPlex.bin
REM Padding to 0x6FFE size 
rm Filled.bin
srec_cat MetaPlex.bin -binary -fill 0x00 0x00 0xE000 -output Filled.bin -binary
REM Creating CRC'ed BIN file
rm CRCed.bin
srec_cat Filled.bin -binary -crop 0x00 %APP_END% --l-e-crc16 %APP_END% -xmodem -output CRCed.bin -binary
rm CRCwProf.bin
srec_cat CRCed.bin -binary Filled.bin -binary -crop %PROFILE_START% 0xFFFF -output CRCwProf.bin -binary
REM avrdude -B 9600 -patmega8 -e -U flash:w:AVRMIDI.hex -c ftisp -P ft0
REM ftisp -E -fw EdrumAVR.bin
cd ..
"SysEx Packer.exe" default/CRCed.bin output.syx
"SysEx Packer.exe" default/CRCwProf.bin outputwProf.syx