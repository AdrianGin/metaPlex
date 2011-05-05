.section .jumps,"ax",@progbits

.global _jump_table
_jump_table:
   jmp usbPoll
   jmp usbFunctionWriteOut
   jmp 0
   jmp bootloader_enter
   jmp usbInit
   jmp USBMIDI_PutByte
   jmp USBMIDI_OutputData
   jmp USBMIDI_EnableRequests
   jmp USBMIDI_GetByte
   jmp _flashmem_erase
   jmp _flashmem_finalise
   jmp _flashmem_writeWord
   jmp _flashmem_release
   jmp hardwareReset

