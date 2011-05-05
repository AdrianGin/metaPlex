
#include <stdint.h>
#include "lowlevel_flashmem.h"
#include "hardwareSpecific.h"

void _flashmem_release()
{
   FLASH_RELEASE();
}

void _flashmem_writeWord(uint32_t address, uint16_t data)
{
   FLASH_WORD_WRITE(address, data);
}

void _flashmem_finalise(uint32_t address)
{
   FLASH_FINALISE_WRITE(address);
   FLASH_RELEASE();
}

void _flashmem_erase(uint32_t address)
{
   FLASH_PAGE_ERASE(address);
   FLASH_RELEASE();
}
