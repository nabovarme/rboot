//////////////////////////////////////////////////
// rBoot OTA and config API for ESP8266.
// Copyright 2015 Richard A Burton
// richardaburton@gmail.com
// See license.txt for license terms.
// OTA code based on SDK sample from Espressif.
//////////////////////////////////////////////////

#include <string.h>
#include <mem.h>
// c_types.h needed for spi_flash.h
#include <c_types.h>
#include <spi_flash.h>
#include <user_interface.h>

#include "rboot-api.h"

#ifdef __cplusplus
extern "C" {
#endif

// Prototypes for ESP8266 hardware ROM functions (Bypassing SDK Flash wrappers)
extern int SPIEraseSector(uint16_t);
extern int SPIWrite(uint32_t, const void*, uint32_t);
extern void Cache_Read_Disable(void);
extern void Cache_Read_Enable_New(uint32_t map, uint32_t base, uint32_t val);

// Force compiler to put these wrappers in Instruction RAM, untouched by flash cache
#define RBOOT_IRAM_ATTR __attribute__((section(".iram.text")))

// Global tracker for the active ROM offset
static uint32_t rboot_active_rom_addr = 0;

// =========================================================================
// IRAM WRAPPERS 
// These execute from Instruction RAM, surviving the cache wipe.
// =========================================================================

static void RBOOT_IRAM_ATTR rboot_restore_cache_iram(void) {
	if (rboot_active_rom_addr != 0) {
		Cache_Read_Disable();
		Cache_Read_Enable_New(0, 0, rboot_active_rom_addr >> 16);
	}
}

static bool RBOOT_IRAM_ATTR rboot_safe_erase(uint16_t sector) {
	int ret;
	ETS_INTR_LOCK(); // Must lock interrupts when using raw ROM functions
	ret = SPIEraseSector(sector);
	rboot_restore_cache_iram();
	ETS_INTR_UNLOCK();
	return (ret == 0);
}

static bool RBOOT_IRAM_ATTR rboot_safe_write(uint32_t addr, uint32_t *buf, uint32_t len) {
	int ret;
	ETS_INTR_LOCK(); // Must lock interrupts when using raw ROM functions
	ret = SPIWrite(addr, buf, len);
	rboot_restore_cache_iram();
	ETS_INTR_UNLOCK();
	return (ret == 0);
}

// =========================================================================
// STANDARD FLASH FUNCTIONS
// =========================================================================

#if defined(BOOT_CONFIG_CHKSUM) || defined(BOOT_RTC_ENABLED)
// calculate checksum for block of data
// from start up to (but excluding) end
static uint8_t calc_chksum(uint8_t *start, uint8_t *end) {
	uint8_t chksum = CHKSUM_INIT;
	while(start < end) {
		chksum ^= *start;
		start++;
	}
	return chksum;
}
#endif

// get the rboot config
rboot_config ICACHE_FLASH_ATTR rboot_get_config(void) {
	rboot_config conf;
	// SDK 3.0 spi_flash_read requires 4-byte multiples. 
	// sizeof(rboot_config) is 25, so we read into an aligned 128-byte array.
	uint32_t buffer[32]; 
	
	spi_flash_read(BOOT_CONFIG_SECTOR * SECTOR_SIZE, buffer, sizeof(buffer));
	// os_memcpy safely handles unaligned struct mappings
	os_memcpy(&conf, buffer, sizeof(rboot_config)); 
	
	// Cache the active ROM address so the IRAM wrappers know where to map
	rboot_active_rom_addr = conf.roms[conf.current_rom];
	
	return conf;
}

// write the rboot config
// preserves the contents of the rest of the sector,
// so the rest of the sector can be used to store user data
// updates checksum automatically (if enabled)
bool ICACHE_FLASH_ATTR rboot_set_config(rboot_config *conf) {
	uint8_t *buffer;
	buffer = (uint8_t*)os_malloc(SECTOR_SIZE);
	if (!buffer) {
		return false;
	}

	// Update the active ROM address tracker for IRAM wrappers
	rboot_active_rom_addr = conf->roms[conf->current_rom];

#ifdef BOOT_CONFIG_CHKSUM
	conf->chksum = calc_chksum((uint8_t*)conf, (uint8_t*)&conf->chksum);
#endif
	
	spi_flash_read(BOOT_CONFIG_SECTOR * SECTOR_SIZE, (uint32_t*)((void*)buffer), SECTOR_SIZE);
	// os_memcpy safely handles unaligned struct mappings
	os_memcpy(buffer, conf, sizeof(rboot_config));
	
	// Use IRAM wrappers to prevent cache wipe crashes
	rboot_safe_erase(BOOT_CONFIG_SECTOR);
	rboot_safe_write(BOOT_CONFIG_SECTOR * SECTOR_SIZE, (uint32_t*)((void*)buffer), SECTOR_SIZE);
	
	os_free(buffer);
	return true;
}

// get current boot rom
uint8_t ICACHE_FLASH_ATTR rboot_get_current_rom(void) {
	rboot_config conf;
	conf = rboot_get_config();
	return conf.current_rom;
}

// set current boot rom
bool ICACHE_FLASH_ATTR rboot_set_current_rom(uint8_t rom) {
	rboot_config conf;
	conf = rboot_get_config();
	if (rom >= conf.count) return false;
	conf.current_rom = rom;
	return rboot_set_config(&conf);
}

// create the write status struct, based on supplied start address
rboot_write_status ICACHE_FLASH_ATTR rboot_write_init(uint32_t start_addr) {
	rboot_write_status status = {0};
	// Ensure active ROM is loaded before flashing begins
	rboot_get_config(); 
	
	status.start_addr = start_addr;
	status.start_sector = start_addr / SECTOR_SIZE;
	status.last_sector_erased = status.start_sector - 1;
	return status;
}

// ensure any remaning bytes get written (needed for files not a multiple of 4 bytes)
bool ICACHE_FLASH_ATTR rboot_write_end(rboot_write_status *status) {
	uint8_t i;
	if (status->extra_count != 0) {
		for (i = status->extra_count; i < 4; i++) {
			status->extra_bytes[i] = 0xff;
		}
		return rboot_write_flash(status, status->extra_bytes, 4);
	}
	return true;
}

// function to do the actual writing to flash
// call repeatedly with more data (max len per write is the flash sector size (4k))
bool ICACHE_FLASH_ATTR rboot_write_flash(rboot_write_status *status, uint8_t *data, uint16_t len) {
	
	bool ret = false;
	uint8_t *buffer;
	int32_t lastsect;
	
	if (data == NULL || len == 0) {
		return true;
	}
	
	// get a buffer
	buffer = (uint8_t *)os_malloc(len + status->extra_count);
	if (!buffer) {
		return false;
	}

	// copy in any remaining bytes from last chunk (os_memcpy is unaligned-safe)
	if (status->extra_count != 0) {
		os_memcpy(buffer, status->extra_bytes, status->extra_count);
	}
	// copy in new data (os_memcpy is unaligned-safe)
	os_memcpy(buffer + status->extra_count, data, len);

	// calculate length, must be multiple of 4
	// save any remaining bytes for next go
	len += status->extra_count;
	status->extra_count = len % 4;
	len -= status->extra_count;
	os_memcpy(status->extra_bytes, buffer + len, status->extra_count);

	// erase any additional sectors needed by this chunk
	lastsect = ((status->start_addr + len) - 1) / SECTOR_SIZE;
	while (lastsect > status->last_sector_erased) {
		status->last_sector_erased++;
		// Use IRAM ROM Wrapper
		if (!rboot_safe_erase(status->last_sector_erased)) {
			os_free(buffer);
			return false;
		}
	}

	// write current chunk - Use IRAM ROM Wrapper
	// buffer is guaranteed 32-bit aligned because it came from os_malloc
	if (rboot_safe_write(status->start_addr, (uint32_t *)((void*)buffer), len)) {
		ret = true;
		status->start_addr += len;
	}

	os_free(buffer);
	return ret;
}

#ifdef BOOT_RTC_ENABLED
bool ICACHE_FLASH_ATTR rboot_get_rtc_data(rboot_rtc_data *rtc) {
	if (system_rtc_mem_read(RBOOT_RTC_ADDR, rtc, sizeof(rboot_rtc_data))) {
		return (rtc->chksum == calc_chksum((uint8_t*)rtc, (uint8_t*)&rtc->chksum));
	}
	return false;
}

bool ICACHE_FLASH_ATTR rboot_set_rtc_data(rboot_rtc_data *rtc) {
	// calculate checksum
	rtc->chksum = calc_chksum((uint8_t*)rtc, (uint8_t*)&rtc->chksum);
	return system_rtc_mem_write(RBOOT_RTC_ADDR, rtc, sizeof(rboot_rtc_data));
}

bool ICACHE_FLASH_ATTR rboot_set_temp_rom(uint8_t rom) {
	rboot_rtc_data rtc;
	// invalid data in rtc?
	if (!rboot_get_rtc_data(&rtc)) {
		// set basics
		rtc.magic = RBOOT_RTC_MAGIC;
		rtc.last_mode = MODE_STANDARD;
		rtc.last_rom = 0;
	}
	// set next boot to temp mode with specified rom
	rtc.next_mode = MODE_TEMP_ROM;
	rtc.temp_rom = rom;

	return rboot_set_rtc_data(&rtc);
}

bool ICACHE_FLASH_ATTR rboot_get_last_boot_rom(uint8_t *rom) {
	rboot_rtc_data rtc;
	if (rboot_get_rtc_data(&rtc)) {
		*rom = rtc.last_rom;
		return true;
	}
	return false;
}

bool ICACHE_FLASH_ATTR rboot_get_last_boot_mode(uint8_t *mode) {
	rboot_rtc_data rtc;
	if (rboot_get_rtc_data(&rtc)) {
		*mode = rtc.last_mode;
		return true;
	}
	return false;
}
#endif

#ifdef __cplusplus
}
#endif
