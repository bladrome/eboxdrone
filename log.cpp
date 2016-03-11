#include "ebox.h"
#include "log.h"

FLASHCLASS eflashlog;

struct s_log logtable;

bool Log_read(void);
bool Log_write(void);


bool Log_read(void)
{
		int nsize = eflashlog.read(LOG_EEPROM, (uint8_t*)&logtable, sizeof(logtable));
		if( nsize == sizeof(logtable) )
				return true;

		return false;
}

bool Log_write(void)
{
		int nsize = eflashlog.write(LOG_EEPROM, (uint8_t*)&logtable, sizeof(logtable));
		if( nsize == sizeof(logtable) )
				return true;

		return false;
}
