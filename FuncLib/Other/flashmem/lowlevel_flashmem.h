#ifndef 	_LOWLEVEL_FLASHMEM_ROUTINES
#define	_LOWLEVEL_FLASHMEM_ROUTINES



/* Both in Byte sizes */
//Normally defined in hardwareSpecific.h
//#define FLASH_END        (0x7FFF)
//#define FLASH_BLOCK_SIZE (512)
/* Hardware specific section to be located in hardwareSpecific.h */
/*
#define FLASH_PAGE_ERASE(address)         boot_page_erase_safe(address)
#define FLASH_WORD_WRITE(address, data)   boot_page_fill_safe(address, data)
#define FLASH_FINALISE_WRITE(address)     boot_page_write_safe(address)
#define FLASH_RELEASE()                   boot_rww_enable_safe()
#define FLASH_WAIT_READY()                boot_spm_busy_wait()
#define FLASH_GET_PGM_WORD(address) pgm_read_word(address)
#define FLASH_GET_PGM_BYTE(address) pgm_read_byte(address)
*/

void _flashmem_erase(uint32_t address);
void _flashmem_finalise(uint32_t address);
void _flashmem_release(void);
void _flashmem_writeWord(uint32_t address, uint16_t data);

#endif
