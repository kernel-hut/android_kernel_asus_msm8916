/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#include "laura_shipping_func.h"
#include "show_log.h"

/* Log count */
static int read_range_log_count = 0; // Read range log count

/* Module id */
static bool module_id_flag = false;
static uint16_t module_id[34];
static uint16_t debug_raw_range = 0, debug_raw_confidence = 0;

static void init_debug_raw_data(void){
		debug_raw_range = 0;
		debug_raw_confidence = 0;
}

uint16_t get_debug_raw_range(void){
	return debug_raw_range;
}

uint16_t get_debug_raw_confidence(void){
	return debug_raw_confidence;
}

/** @brief Swap high and low of the data (e.g 0x1234 => 0x3412)
*
*	@param register_data the data which will be swap
*
*/
uint16_t swap_data(uint16_t register_data){
	return ((register_data >> 8) | ((register_data & 0xff) << 8)) ;
}


/** @brief Mailbox: create calibration data
*		  This mailbox command is used to retrieve data to be used for the computation of calibration parameters.
*		  This is a singleentry MBX command with MBX message response with Msg_len = 6
*
*	@param dev_t the laser focus controller
*	@param cal_data the calibration ouput data
*
*/
int Mailbox_Command(struct msm_laser_focus_ctrl_t *dev_t, int16_t cal_data[]){
	int status = 0, msg_len = 0, M2H_Msg_Len = 0;
	uint16_t i2c_read_data, i2c_read_data2;
	struct timeval start;//, now;

	start = get_current_time();
	while(1){
		status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
		if (status < 0){
			return status;
       	}
		i2c_read_data = swap_data(i2c_read_data);
		
		if((i2c_read_data&0x20) == 0x00){
			break;
		}

		/* Busy pending MCPU Msg */
		/* Read M2H_MBX */
		status = CCI_I2C_RdWord(dev_t, 0x12, &i2c_read_data2);
		if (status < 0){
			return status;
       	}
		i2c_read_data2 = swap_data(i2c_read_data2);
		LOG_Handler(LOG_ERR, "%s: register(0x00, 0x10): (0x%x, 0x%x)\n", __func__,  i2c_read_data, i2c_read_data2);
#if 0
		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Verify ICSR(2:1) time out - register(0x10): 0x%x\n", __func__, i2c_read_data);
                    	return -TIMEOUT_VAL;
              }
#endif
		//usleep(DEFAULT_DELAY_TIME);
	}

	status = CCI_I2C_WrWord(dev_t, 0x10, swap_data(0x0004));
       if (status < 0){
               return status;
       }

	while(1){
		status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
		if (status < 0){
			return status;
       	}
		i2c_read_data = swap_data(i2c_read_data);
		
		if((i2c_read_data&0x20) == 0x20){
			break;
		}

#if 0
		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Verify ICSR(2:1) time out - register(0x10): 0x%x\n", __func__, i2c_read_data);
                    	return -TIMEOUT_VAL;
              }
#endif
		//usleep(DEFAULT_DELAY_TIME);
	}


	/* Read M2H_MBX */
	status = CCI_I2C_RdWord(dev_t, 0x12, &i2c_read_data);
	i2c_read_data = swap_data(i2c_read_data);
	LOG_Handler(LOG_ERR, "%s: Verify M2H_MBX(1)  register(0x12): 0x%x\n", __func__, i2c_read_data);
	if (status < 0){
		return status;
       }
	M2H_Msg_Len = (i2c_read_data&0x3F00)>>8;
	LOG_Handler(LOG_ERR, "%s: Verify M2H_MBX(1) M2H_Msg_Len: %d\n", __func__, M2H_Msg_Len);

	if(((i2c_read_data&0xFF) == 0xCC) && (M2H_Msg_Len == CAL_MSG_LEN)){
		for(msg_len=0; msg_len<M2H_Msg_Len; msg_len++){
			start = get_current_time();
			while(1){
				status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
				if (status < 0){
					return status;
       			}
				i2c_read_data = swap_data(i2c_read_data);
				LOG_Handler(LOG_ERR, "%s: Verify ICSR(1)  register(0x00): 0x%x\n", __func__, i2c_read_data);
			
				if((i2c_read_data&0x20) != 0x00){
					break;
				}
#if 0
				/* Check if time out */
				now = get_current_time();
              		if(is_timeout(start,now,TIMEOUT_VAL)){
					LOG_Handler(LOG_ERR, "%s: Verify ICSR(1) time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    			return -TIMEOUT_VAL;
              		}
#endif
		 		//usleep(DEFAULT_DELAY_TIME);
			}

			/* Read M2H_MBX */
       		status = CCI_I2C_RdWord(dev_t, 0x12, &i2c_read_data);
			if (status < 0){
              		return status;
       		}
			i2c_read_data = swap_data(i2c_read_data);

			/* Append to previosly saved data */
			cal_data[msg_len] = i2c_read_data;
			LOG_Handler(LOG_DBG, "%s: Calibration data[%d]: 0x%x", __func__, msg_len, cal_data[msg_len]);
		}
	}
	else{
		LOG_Handler(LOG_ERR, "%s: M2H_MBX(7:0): 0x%x, Msg_Len: %d\n", __func__, i2c_read_data&0xFF, M2H_Msg_Len);
		return -1;
	}
	return status;
}

/** @brief Load calibration data 
*
*	@param dev_t the laser focus controller
*
*/
int Laura_device_Load_Calibration_Value(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t indirect_addr, data[SIZE_OF_LAURA_CALIBRATION_DATA];
#if DEBUG_LOG_FLAG
	int i = 0;
	uint16_t data_verify;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);


	/* Read Calibration data */
	indirect_addr = swap_data(0xC010);

	status = Larua_Read_Calibration_Data_From_File(data, SIZE_OF_LAURA_CALIBRATION_DATA);
	if(status < 0){
		LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		return status;
	}

	Laura_device_indirect_addr_write(dev_t, 0x18, 0x19, indirect_addr, 0x1A, data, 10);

	/* Check patch memory write */
	CCI_I2C_WrByte(dev_t, 0x18, 0x10);
	CCI_I2C_WrByte(dev_t, 0x19, 0xC0);
#if DEBUG_LOG_FLAG
	for(i = 0; i < 20; i++){
		CCI_I2C_RdByte(dev_t, 0x1A, &data_verify);
		LOG_Handler(LOG_DBG, "%s: 0x1A: 0x%x\n", __func__, data_verify);
	}
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
}

/** @brief Read calibration data from calibration file
*
*	@param vfile
*       @param cal_data the calibration data
*
*/
int Laura_Read_Calibration_Value_From_File(struct seq_file *vfile, uint16_t *cal_data){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = Larua_Read_Calibration_Data_From_File(cal_data, SIZE_OF_LAURA_CALIBRATION_DATA);
        if(status < 0){
                LOG_Handler(LOG_ERR, "%s: Load calibration fail!!\n", __func__);
		if(vfile!=NULL){
			seq_printf(vfile,"No calibration data!!\n");
		}
                return status;
        }

	LOG_Handler(LOG_CDBG,"Cal data: %04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
		cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);	

	if(vfile!=NULL){
		seq_printf(vfile,"%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x\n",
               		cal_data[0], cal_data[1], cal_data[2], cal_data[3], cal_data[4],
                	cal_data[5], cal_data[6], cal_data[7], cal_data[8], cal_data[9]);

	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief laura read range 
*	
*	@param dev_t the laser focus controller
*
*/
int16_t Laura_device_read_range(struct msm_laser_focus_ctrl_t *dev_t, int *errStatus)
{
	uint16_t RawRange = 0, i2c_read_data = 0, realRange = 0, RawConfidence = 0, confidence_level = 0;
	int status;
	struct timeval start, now;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* Init debug raw data */
	init_debug_raw_data();

	/* read count +1  */
	read_range_log_count++;

	/* Verify status is MCPU on */
	while(1){
		status = CCI_I2C_RdWord(dev_t, 0x06, &i2c_read_data);
		if (status < 0){
       		return status;
       	}
		i2c_read_data = swap_data(i2c_read_data);

		if(i2c_read_data == 0x01F8){
			break;
		}

		/* Check if time out */
		now = get_current_time();
              if(is_timeout(start,now,5)){
			LOG_Handler(LOG_ERR, "%s: Verify MCPU status time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
                  	return OUT_OF_RANGE;
              }

		LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);
		
	}
	
#if READ_RETRY_FLAG
	while(1){
#endif
		/* Trigger single measure */
        	status = CCI_I2C_WrWord(dev_t, 0x04, swap_data(0x0081));
        	if (status < 0){
              	return status;
        	}

		/* Wait until data ready */
       	while(1){
			status = CCI_I2C_RdWord(dev_t, 0x00, &i2c_read_data);
			if (status < 0){
       			return status;
       		}
			i2c_read_data = swap_data(i2c_read_data);

			if((int)(i2c_read_data&0x10) == 0x10){
				break;
			}

			/* Check if time out */
			now = get_current_time();
             		if(is_timeout(start,now,80)){
				LOG_Handler(LOG_ERR, "%s: Wait data ready time out - register(0x00): 0x%x\n", __func__, i2c_read_data);
                    		return OUT_OF_RANGE;
             		}

			/* Delay: waitting laser sensor sample ready */
			usleep(READ_DELAY_TIME);
       	}

		/* Read distance */
       	status = CCI_I2C_RdWord(dev_t, 0x08, &RawRange);
		if (status < 0){
              	return status;
       	}
		RawRange = swap_data(RawRange);
		debug_raw_range = RawRange;
		LOG_Handler(LOG_DBG, "%s: [SHOW_LOG] Non-shift Read range:%d\n", __func__, RawRange);

		/* Check if target is valid */
		/* RawRange bit 15 is 1 */
		if((RawRange&0x8000)==0x8000){	
			/* RawRange bit 13,14 are 00 */
			if((RawRange&0x6000)==0x00){
				
				/* Get real range success */
				realRange = (RawRange&0x1fff)>>2;
				
				/* [Issue solution] Inf report smale value : 
				 * When the distance measured < 250mm and confidence level < 15, the result is invalid */
				
				/* Read result confidence level */
				status = CCI_I2C_RdWord(dev_t, 0x0A, &RawConfidence);
				if (status < 0){
					return status;
				}
				RawConfidence = swap_data(RawConfidence);
				debug_raw_confidence = RawConfidence;
				confidence_level = (RawConfidence&0x7fff)>>4;
				LOG_Handler(LOG_DBG,"%s: [SHOW_LOG] confidence level is: %d (Raw data:%d)\n", __func__, confidence_level, RawConfidence);
				
				/* distance <= 400 and confidence < 11*400/distance 
				* , distance > 400 and confidence < 11
				* or distance = 0,
				* then ignored this distance measurement.
				* */
                if((realRange <= 400 && confidence_level < (11*400/realRange))
				  || (realRange > 400 && confidence_level < 11)
				  || realRange == 0){
					LOG_Handler(LOG_DBG, "%s: Read range fail (range,confidence level): (%d,%d)\n", __func__,realRange,confidence_level);
					*errStatus = 0;
					realRange = OUT_OF_RANGE;
                }
				else{
					*errStatus = 0;
				}

#if READ_OUTPUT_LIMIT_FLAG
				/* Display distance */
				if(read_range_log_count >= LOG_SAMPLE_RATE){
					read_range_log_count = 0;
					LOG_Handler(LOG_CDBG, "%s: Read range:%d\n", __func__, realRange);
				}
#endif
			}
			/* RawRange bit 13,14 are 11 */
			else if((RawRange&0x6000)==0x6000){
				LOG_Handler(LOG_DBG, "%s: Read range bit 13, 14, 15 are '1'(%d)\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = OUT_OF_RANGE;
			}
			/* RawRange bit 13 is 1, but bit 14 is 0 */
			else if((RawRange&0x2000)==0x2000){
		  		LOG_Handler(LOG_DBG, "%s: The target is near of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = 0;
				realRange = 0;
	   	  	}
	   	  	/* RawRange bit 13 is 0, but bit 14 is 1 */
	   	  	else if((RawRange&0x4000)==0x4000){
				LOG_Handler(LOG_DBG, "%s: The target is out of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = 0;
				realRange = OUT_OF_RANGE;
		  	}
		  	else{
				LOG_Handler(LOG_DBG, "%s: Read range fail(%d)!!\n", __func__,RawRange);
				*errStatus = 0;
				realRange = OUT_OF_RANGE;
		  	}
#if READ_RETRY_FLAG
			break;
#endif
		}
		/* RawRange bit 15 is 0 */
       	else {
			/* RawRange bit 13 is 1, but bit 14 is 0 */
	   	  	if((RawRange&0x2000)==0x2000){
		  		LOG_Handler(LOG_ERR, "%s: The target is near of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = 0;
	   	  	}
	   	  	/* RawRange bit 13 is 0, but bit 14 is 1 */
	   	  	else if((RawRange&0x4000)==0x4000){
				LOG_Handler(LOG_ERR, "%s: The target is out of field of view(%d)!!\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = OUT_OF_RANGE;
		  	}
		  	/* RawRange bit 13,14 are 11 or 00 */
		  	else{
				LOG_Handler(LOG_ERR, "%s: Read range fail(%d)!!\n", __func__,RawRange);
				*errStatus = RANGE_ERR;
				realRange = OUT_OF_RANGE;
		  	}
#if READ_RETRY_FLAG == 0
			//return OUT_OF_RANGE;
#endif
       	}
#if READ_RETRY_FLAG
		/* Check if time out */
		now = get_current_time();
        if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_DBG, "%s: Read range time out!!\n", __func__);
        	return OUT_OF_RANGE;
        }
		usleep(1);
	}
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return (int)realRange;
}

/** @brief MCPU Contorller
*
*	@param dev_t the laser focus controller
*	@param mode the MCPU go to status
*
*/
int Laura_MCPU_Controller(struct msm_laser_focus_ctrl_t *dev_t, int mode){
	int status;
#if DEBUG_LOG_FLAG
	uint16_t i2c_read_data = 0;
#endif


	switch(mode){
		case MCPU_ON:
			/* Enable MCPU to run coming out of standby */
			LOG_Handler(LOG_DBG, "%s: MCPU ON\n", __func__);
			status = CCI_I2C_WrWord(dev_t, 0x14, swap_data(0x0600));
       		if (status < 0){
             			return status;
      			}
			
			/* Wake up MCPU to ON mode */
			status = CCI_I2C_WrWord(dev_t, 0x04, swap_data(0x0092));
			if (status < 0){
       			return status;
       		}
			break;
		case MCPU_OFF:
			/* Enable patch memory */
			LOG_Handler(LOG_DBG, "%s: MCPU OFF\n", __func__);
			status = CCI_I2C_WrWord(dev_t, 0x14, swap_data(0x0500));
       		if (status < 0){
             			return status;
      			}
			
			/* Go MCPU to OFF status */
			status = CCI_I2C_WrWord(dev_t, 0x04, swap_data(0x0091));
        		if (status < 0){
              		return status;
        		}

			break;
		case MCPU_STANDBY:
			/* Change MCUP to standby mode */
			LOG_Handler(LOG_DBG, "%s: MCPU STANDBY\n", __func__);
			status = CCI_I2C_WrWord(dev_t, 0x04, swap_data(0x0090));
       		if (status < 0){
       			return status;
     			}
			break;
		default:
			LOG_Handler(LOG_ERR, "%s Type fail(%d) !!\n", __func__, mode);
			break;
	}

	/* wait hardware booting(least 500us) */
	usleep(MCPU_DELAY_TIME);
	
#if DEBUG_LOG_FLAG
	/* Verify MCPU status */
	status = CCI_I2C_RdWord(dev_t, 0x06, &i2c_read_data);
	if (status < 0){
             	return status;
       }
	i2c_read_data = swap_data(i2c_read_data);
	LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);
#endif

	return status;
}

/** @brief Initialize Laura tof configure
*
*	@param dev_t the laser focus controller
*	@param config the configuration param
*
*/
int Laura_device_UpscaleRegInit(struct msm_laser_focus_ctrl_t *dev_t, uint16_t *config)
{
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

#if DEBUG_LOG_FLAG
	LOG_Handler(LOG_DBG, "%s: config:(0x%x,0x%x,0x%x,0x%x,0x%x,0x%x)\n", __func__,
		config[0], config[1], config[2], config[3], config[4], config[5]);
#endif

	/* Drive INT_PAD high */
	status = CCI_I2C_WrWord(dev_t, 0x00, swap_data(0x0004));
       if (status < 0){
               return status;
       }

	/* Change the default VCSEL threshold and VCSEL peak */
       status = CCI_I2C_WrWord(dev_t, 0x0C, swap_data(config[0]));
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x0E, swap_data(config[1]));
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x20, swap_data(config[2]));
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x22, swap_data(config[3]));
       if (status < 0){
               return status;
       }
	   
      status = CCI_I2C_WrWord(dev_t, 0x24, swap_data(config[4]));
       if (status < 0){
               return status;
       }
	   
       status = CCI_I2C_WrWord(dev_t, 0x26, swap_data(config[5]));
       if (status < 0){
               return status;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
} 

/** @brief Wait device go to standby mode
*
*	@param dev_t the laser focus controller
*
*/
int Laura_WaitDeviceStandby(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
#if 1
	struct timeval start, now;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
#if 1
	start = get_current_time();
#endif
	/* Wait chip standby */
	while(1){
		/* Go MCPU to standby mode */
		Laura_MCPU_Controller(dev_t, MCPU_STANDBY);
		
		status = CCI_I2C_RdWord(dev_t, 0x06, &i2c_read_data);
		if (status < 0){
              		return status;
        	}
		i2c_read_data = swap_data(i2c_read_data);
		LOG_Handler(LOG_DBG, "%s: register(0x06): 0x%x\n", __func__, i2c_read_data);
		i2c_read_data = (i2c_read_data & 0x1F);

		if(i2c_read_data == 0){
			status = i2c_read_data;
			break;
		}

#if 1
		/* Check if time out */
		now = get_current_time();
             	if(is_timeout(start,now,1000)){
			LOG_Handler(LOG_ERR, "%s: Wait chip standby time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              		return -TIMEOUT_VAL;
             	}
#endif
		
		//usleep(DEFAULT_DELAY_TIME);
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Configure i2c interface
*
*	@param dev_t the laser focus controller
*
*/
int Laura_Config_I2C_Interface(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Configure I2C interface */
	status = CCI_I2C_WrWord(dev_t, 0x1C, swap_data(0x0065));
       if (status < 0){
               return status;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Power up initialization without applying calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_Power_Up_Init_No_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Wake up MCPU to ON mode */
	Laura_MCPU_Controller(dev_t, MCPU_ON);

#if 0
	/* wait hardware booting(least 500us) */
	//usleep(MCPU_DELAY_TIME);
#endif

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Power up initialization which apply calibration data
*
*	@param dev_t the laser focus controller
*
*/
int Laura_Power_Up_Init_Apply_Calibration(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t i2c_read_data = 0;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Go MCPU to OFF status */
	Laura_MCPU_Controller(dev_t, MCPU_OFF);

#if 0
	/* wait hardware handling (least 500us) */
	usleep(MCPU_DELAY_TIME);
#endif

	while(1){
		/* Verify status is MCPU off */
		status = CCI_I2C_RdWord(dev_t, 0x06, &i2c_read_data);
		if (status < 0){
             		return status;
       	}
		i2c_read_data = swap_data(i2c_read_data);
		i2c_read_data = (i2c_read_data & 0x1F);

		if(i2c_read_data == 0x10){
			break;
		}

		//usleep(DEFAULT_DELAY_TIME);
	}
	
	/* Load calibration data */
	Laura_device_Load_Calibration_Value(dev_t);

	/* Wake up MCPU to ON mode */
	Laura_MCPU_Controller(dev_t, MCPU_ON);
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

/** @brief Go to standby mode when do not do measure
*
*	@param dev_t the laser focus controller
*
*/
int Laura_non_measures_go_standby(struct msm_laser_focus_ctrl_t *dev_t)
{
	int status = 0;
	uint16_t i2c_read_data = 0;
	struct timeval start, now;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	start = get_current_time();

	/* Verify status is MCPU on */
	while(1){
		status = CCI_I2C_RdWord(dev_t, 0x06, &i2c_read_data);
		if (status < 0){
             		return status;
       	}
		i2c_read_data = swap_data(i2c_read_data);

		if(i2c_read_data==0x01F8){
			break;
		}

		/* Check if time out */
		now = get_current_time();
             	if(is_timeout(start,now,TIMEOUT_VAL)){
			LOG_Handler(LOG_ERR, "%s: Wait MCPU on time out - register(0x06): 0x%x\n", __func__, i2c_read_data);
              	return -TIMEOUT_VAL;
       	}
		LOG_Handler(LOG_DBG, "%s: register(0x06):0x%x!!\n", __func__,i2c_read_data);

		//usleep(DEFAULT_DELAY_TIME);
	}

	/* Go MCPU to standby mode */
	Laura_MCPU_Controller(dev_t, MCPU_STANDBY);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return status;
}

/** @brief Get module id from chip
*
*       @param dev_t the laser focus controller
*
*/
void Laura_Get_Module_ID_From_Chip(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0, i = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	status = CCI_I2C_WrByte(dev_t, 0x18, 0x04);
        status = CCI_I2C_WrByte(dev_t, 0x19, 0xC8);

	for(i = 0; i < 34; i++){
               	CCI_I2C_RdByte(dev_t, 0x1A, &module_id[i]);
        }

	LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
		module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
		module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
		module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
		module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
		module_id[32],module_id[33]);

	module_id_flag=true;

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
}

/** @brief Get Chip from driver
*
*       @param dev_t the laser focus controller
*       @param vfile
*
*/
void Laura_Get_Module_ID(struct msm_laser_focus_ctrl_t *dev_t, struct seq_file *vfile){

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!module_id_flag){
		Laura_MCPU_Controller(dev_t, MCPU_OFF);
		Laura_Get_Module_ID_From_Chip(dev_t);	
	}
	else{
		LOG_Handler(LOG_DBG,"Module ID:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
                	module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                	module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                	module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                	module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                	module_id[32],module_id[33]);
	}

	if(vfile!=NULL){
                seq_printf(vfile,"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n",
                        module_id[0],module_id[1],module_id[2],module_id[3],module_id[4],module_id[5],module_id[6],module_id[7],
                        module_id[8],module_id[9],module_id[10],module_id[11],module_id[12],module_id[13],module_id[14],module_id[15],
                        module_id[16],module_id[17],module_id[18],module_id[19],module_id[20],module_id[21],module_id[22],module_id[23],
                        module_id[24],module_id[25],module_id[26],module_id[27],module_id[28],module_id[29],module_id[30],module_id[31],
                        module_id[32],module_id[33]);
        }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
}

/** @brief Verify firmware version
*
*	@param dev_t the laser focus controller
*
*/
bool Laura_FirmWare_Verify(struct msm_laser_focus_ctrl_t *dev_t){
	int status = 0;
	uint16_t fw_major_version, fw_minor_version;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Laura_MCPU_Controller(dev_t, MCPU_OFF);

#if 0
	/* wait hardware handling (least 500us) */
	usleep(MCPU_DELAY_TIME);
#endif

	status = CCI_I2C_WrByte(dev_t, 0x18, 0xC0);
	status = CCI_I2C_WrByte(dev_t, 0x19, 0xFF);

	status = CCI_I2C_RdByte(dev_t, 0x1A, &fw_major_version);
	fw_major_version = fw_major_version & 0x3F;
	status = CCI_I2C_RdByte(dev_t, 0x1A, &fw_minor_version);

	LOG_Handler(LOG_DBG, "%s: LSB: 0x%x ; MSB: 0x%x\n", __func__, fw_major_version, fw_minor_version);

	Laura_Get_Module_ID_From_Chip(dev_t);

	if( fw_major_version >= 0 && fw_minor_version >= 14 ){
		/* Can do calibraion */
		LOG_Handler(LOG_DBG, "%s: It can do calibration!!\n", __func__);
		return true;
	}
	else{
		/* Can not do calibraion */
		LOG_Handler(LOG_DBG, "%s: The fireware is too old, it can not do calibration!!\n", __func__);
		return false;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return false;
}
