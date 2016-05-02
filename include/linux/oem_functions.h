#ifndef _LINUX_OEM_FUNCTIONS_H
#define _LINUX_OEM_FUNCTIONS_H


struct oem_shared_emmc_info
{
  uint32_t   manufacturer_id;
  uint32_t   oem_id;
  uint32_t   prod_rev;
  uint32_t   prod_serial_num;
  uint32_t   block_count;
  uint32_t   bytes_per_block;
  uint8_t    product_name[8];
  uint8_t    manufactured_date[8];
  uint64_t fw_version;
  uint32_t ext_csd_rev;
}  __attribute__((packed));

struct oem_shared_ddr_info
{
  uint32_t device_name;
  uint32_t manufacture_name;
  uint32_t revision;
  uint32_t device_type;
  uint32_t device_density_cs0;
  uint32_t device_density_cs1;
}  __attribute__((packed));

struct oem_shared_info
{
	uint32_t version;
	uint32_t auth_enabled;
	uint32_t msm_hw_revision;
	struct oem_shared_emmc_info emmc;
	//struct oem_shared_ddr_info ddr[2];
	struct oem_shared_ddr_info ddr[1];	// There is only one DDR Interface on MSM8916
}  __attribute__((packed));


uint32_t get_rf_sku_id(void);

const char* get_oem_ssn(void);

uint32_t get_modem_debug_value(void);
void set_modem_debug_value(uint32_t);


#endif /* _LINUX_ATOMIC_H */
