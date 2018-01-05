#ifndef _DT_PMIC_H
#define _DT_PMIC_H

#define PMIC_DOMIAN_DISABLE	0
#define PMIC_DOMIAN_ENABLE	1

#define PMIC_PULLDOWN_FIXED     0
#define PMIC_PULLDOWN_EN        1
#define PMIC_PULLDOWN_DIS       2

#define PMIC_MODE_NOT_KNOWN             0 /**< Unknown mode => never use this, hard coded value */
#define PMIC_MODE_FIXED                 1 /**< Mode can/shall not be modified */
#define PMIC_MODE_IDLE_STDBY            2 /**< LDO is automatically switched into idle mode (low power) in standby (sleep) mode by HW. It is automatically switched to active mode if system is active. */
#define PMIC_MODE_OFF                   3 /**< LDO is off */
#define PMIC_MODE_OFF_STDBY             4 /**< LDO is automatically switched off in standby (sleep) mode by HW */
#define PMIC_MODE_ON                    5 /**< LDO is always on */
#define PMIC_MODE_STANDBYHIGHVOLTAGE    6 /**< ONLY for LCABB LDO !!! Use programmed voltage during sleep (standby) */
#define PMIC_MODE_STANDBYLOWVOLTAGE     7 /**< ONLY for LCABB LDO !!! Use 1.00V during sleep (standby) */
#define PMIC_NOF_MODE                   8 /**< End indicator, hard coded value */

#define PMIC_VOLTAGE_FIXED		0
#define PMIC_VOLTAGE_0V7125		712500	/**< 0.7125 Volts */
#define PMIC_VOLTAGE_0V725		725000	/**< 0.725 Volts */
#define PMIC_VOLTAGE_0V7375		737500	/**< 0.7375 Volts */
#define PMIC_VOLTAGE_0V75		750000	/**< 0.75 Volts */
#define PMIC_VOLTAGE_0V7625		762500	/**< 0.7625 Volts */
#define PMIC_VOLTAGE_0V775		775000	/**< 0.775 Volts */
#define PMIC_VOLTAGE_0V7875		787500	/**< 0.7875 Volts */
#define PMIC_VOLTAGE_0V80		800000	/**< 0.80 Volts */
#define PMIC_VOLTAGE_0V8125		812500	/**< 0.8125 Volts */
#define PMIC_VOLTAGE_0V825		825000	/**< 0.825 Volts */
#define PMIC_VOLTAGE_0V8375		837500	/**< 0.8375 Volts */
#define PMIC_VOLTAGE_0V85		850000	/**< 0.85 Volts */
#define PMIC_VOLTAGE_0V8625		862500	/**< 0.8625 Volts */
#define PMIC_VOLTAGE_0V875		875000	/**< 0.875 Volts */
#define PMIC_VOLTAGE_0V8875		887500	/**< 0.8875 Volts */
#define PMIC_VOLTAGE_0V90		900000	/**< 0.90 Volts */
#define PMIC_VOLTAGE_0V9125		912500	/**< 0.9125 Volts */
#define PMIC_VOLTAGE_0V925		925000	/**< 0.925 Volts */
#define PMIC_VOLTAGE_0V9375		937500	/**< 0.9375 Volts */
#define PMIC_VOLTAGE_0V95		950000	/**< 0.95 Volts */
#define PMIC_VOLTAGE_0V9625		962500	/**< 0.9625 Volts */
#define PMIC_VOLTAGE_0V975		975000	/**< 0.975 Volts */
#define PMIC_VOLTAGE_0V9875		987500	/**< 0.9875 Volts */
#define PMIC_VOLTAGE_1V00		1000000	/**< 1.00 Volts */
#define PMIC_VOLTAGE_1V0125		1012500	/**< 1.0125 Volts */
#define PMIC_VOLTAGE_1V02		1020000	/**< 1.02 Volts */
#define PMIC_VOLTAGE_1V025		1025000	/**< 1.025 Volts */
#define PMIC_VOLTAGE_1V0375		1037500	/**< 1.0375 Volts */
#define PMIC_VOLTAGE_1V05		1050000	/**< 1.05 Volts */
#define PMIC_VOLTAGE_1V0625		1062500	/**< 1.0625 Volts */
#define PMIC_VOLTAGE_1V075		1075000	/**< 1.075 Volts */
#define PMIC_VOLTAGE_1V0875		1087500	/**< 1.0875 Volts */
#define PMIC_VOLTAGE_1V10		1100000	/**< 1.10 Volts */
#define PMIC_VOLTAGE_1V1125		1112500	/**< 1.1125 Volts */
#define PMIC_VOLTAGE_1V12		1120000	/**< 1.12 Volts */
#define PMIC_VOLTAGE_1V125		1125000	/**< 1.125 Volts */
#define PMIC_VOLTAGE_1V1375		1137500	/**< 1.1375 Volts */
#define PMIC_VOLTAGE_1V15		1150000	/**< 1.15 Volts */
#define PMIC_VOLTAGE_1V16		1160000	/**< 1.16 Volts */
#define PMIC_VOLTAGE_1V1625		1162500 /**< 1.1625 Volts */
#define PMIC_VOLTAGE_1V17		1170000	/**< 1.17 Volts */
#define PMIC_VOLTAGE_1V175		1175000	/**< 1.175 Volts */
#define PMIC_VOLTAGE_1V1875		1187500	/**< 1.1875 Volts */
#define PMIC_VOLTAGE_1V20		1200000	/**< 1.20 Volts */
#define PMIC_VOLTAGE_1V2125		1212500	/**< 1.2125 Volts */
#define PMIC_VOLTAGE_1V22		1220000	/**< 1.22 Volts */
#define PMIC_VOLTAGE_1V225		1225000	/**< 1.225 Volts */
#define PMIC_VOLTAGE_1V226		1226000	/**< 1.226 Volts */
#define PMIC_VOLTAGE_1V2375		1237500	/**< 1.2375 Volts */
#define PMIC_VOLTAGE_1V25		1250000	/**< 1.25 Volts */
#define PMIC_VOLTAGE_1V2625		1262500	/**< 1.2625 Volts */
#define PMIC_VOLTAGE_1V275		1275000	/**< 1.275 Volts */
#define PMIC_VOLTAGE_1V2875		1287500	/**< 1.2875 Volts */
#define PMIC_VOLTAGE_1V30		1300000	/**< 1.30 Volts */
#define PMIC_VOLTAGE_1V3125		1312500	/**< 1.3125 Volts */
#define PMIC_VOLTAGE_1V325		1325000	/**< 1.325 Volts */
#define PMIC_VOLTAGE_1V3375		1337500	/**< 1.3375 Volts */
#define PMIC_VOLTAGE_1V35		1350000	/**< 1.35 Volts */
#define PMIC_VOLTAGE_1V3625		1362500	/**< 1.3625 Volts */
#define PMIC_VOLTAGE_1V375		1375000	/**< 1.375 Volts */
#define PMIC_VOLTAGE_1V3875		1387500	/**< 1.3875 Volts */
#define PMIC_VOLTAGE_1V40		1400000	/**< 1.40 Volts */
#define PMIC_VOLTAGE_1V4125		1412500	/**< 1.4125 Volts */
#define PMIC_VOLTAGE_1V425		1425000	/**< 1.425 Volts */
#define PMIC_VOLTAGE_1V4375		1437500	/**< 1.4375 Volts */
#define PMIC_VOLTAGE_1V45		1450000	/**< 1.45 Volts */
#define PMIC_VOLTAGE_1V4625		1462500	/**< 1.4625 Volts */
#define PMIC_VOLTAGE_1V475		1475000	/**< 1.475 Volts */
#define PMIC_VOLTAGE_1V4875		1487500	/**< 1.4875 Volts */
#define PMIC_VOLTAGE_1V50		1500000	/**< 1.50 Volts */
#define PMIC_VOLTAGE_1V60		1600000	/**< 1.60 Volts */
#define PMIC_VOLTAGE_1V70		1700000	/**< 1.70 Volts */
#define PMIC_VOLTAGE_1V80		1800000	/**< 1.80 Volts */
#define PMIC_VOLTAGE_1V90		1900000	/**< 1.90 Volts */
#define PMIC_VOLTAGE_2V00		2000000	/**< 2.00 Volts */
#define PMIC_VOLTAGE_2V05		2050000	/**< 2.05 Volts */
#define PMIC_VOLTAGE_2V10		2100000	/**< 2.10 Volts */
#define PMIC_VOLTAGE_2V20		2200000	/**< 2.20 Volts */
#define PMIC_VOLTAGE_2V30		2300000	/**< 2.30 Volts */
#define PMIC_VOLTAGE_2V40		2400000	/**< 2.40 Volts */
#define PMIC_VOLTAGE_2V50		2500000	/**< 2.50 Volts */
#define PMIC_VOLTAGE_2V60		2600000	/**< 2.60 Volts */
#define PMIC_VOLTAGE_2V70		2700000	/**< 2.70 Volts */
#define PMIC_VOLTAGE_2V80		2800000	/**< 2.80 Volts */
#define PMIC_VOLTAGE_2V85		2850000	/**< 2.85 Volts */
#define PMIC_VOLTAGE_2V90		2900000	/**< 2.90 Volts */
#define PMIC_VOLTAGE_2V91		2910000	/**< 2.91 Volts */
#define PMIC_VOLTAGE_3V00		3000000	/**< 3.00 Volts */
#define PMIC_VOLTAGE_3V10		3100000	/**< 3.10 Volts */
#define PMIC_VOLTAGE_3V15		3150000	/**< 3.15 Volts */
#define PMIC_VOLTAGE_3V20		3200000	/**< 3.20 Volts */
#define PMIC_VOLTAGE_3V30		3300000	/**< 3.30 Volts */
#define PMIC_VOLTAGE_3V40		3400000	/**< 3.40 Volts */
#define PMIC_VOLTAGE_3V50		3500000	/**< 3.50 Volts */
#define PMIC_VOLTAGE_3V60		3600000	/**< 3.60 Volts */
#define PMIC_VOLTAGE_4V70		4700000	/**< 4.70 Volts */
#define PMIC_VOLTAGE_4V80		4800000	/**< 4.80 Volts */
#define PMIC_VOLTAGE_4V90		4900000	/**< 4.90 Volts */
#define PMIC_VOLTAGE_5V00		5000	/**< 5.00 Volts */
#define PMIC_VOLTAGE_5V10		5100000	/**< 5.10 Volts */
#define PMIC_VOLTAGE_5V20		5200000	/**< 5.20 Volts */
#define PMIC_VOLTAGE_5V30		5300000	/**< 5.30 Volts */
#define PMIC_VOLTAGE_5V40		5400000	/**< 5.40 Volts */

/* virt domain */
#define PMIC_VDDP_MMC                       0   /**< Pad supply and possibly the supply for the SD MMC card */
#define PMIC_VDDP_MMC_TD                    1   /**< Pad supply and possibly the supply for the trace and debugger */
#define PMIC_VDD_VEMMC                      2   /**<  */
#define PMIC_VDD_SDIO                       3   /**< Supply for SDIO */
#define PMIC_VDD_TOUCHSCREEN                4   /**< Supply voltage for Touchscreen(TP) - 1V8 */
#define PMIC_VDD_EMIC_1V8_IO                5   /**< Supply for RX */
#define PMIC_VDD_MIPI_CSI                   6   /**< Supply for MIPI CSI */
#define PMIC_VDD_MIPI_DSI                   7   /**< Supply for MIPI DSI */
#define PMIC_VDD_DIGRF1                     8   /**< Supply for DIGRF V3.0.9 */
#define PMIC_VDD_USB_1V1                    9   /**< Supply for USB HS PHY 1,1V domain */
#define PMIC_VDD_EMIC_CORE                  10  /**< Supply for EMIC_CORE */
#define PMIC_VDD_EMIC_DLL                   11  /**< Supply for EMIC DLL */
#define PMIC_VDD_IDI_RX                     12  /**< Supply for IDI receiver */
#define PMIC_VDD_USB_3V3                    13  /**< Supply for USB HS I/O, USB PLL */
#define PMIC_VDDP_SIM                       14  /**< Supply voltage for SIM generated from VSIM PMU regulator */
#define PMIC_VDDP_SIM2                      15  /**< Supply voltage for SIM2 generated from VSIM PMU regulator */
#define PMIC_VDD_PLL                        16  /**< Supply for PLL */
#define PMIC_VDD_IO_VOLT_TOUCH_SENSOR       17  /**< io voltage for TOUCH_SENSOR */
#define PMIC_VDD_IO_VOLT_PROXIMITY_SENSOR   18  /**< io voltage for PROXIMITY_SENSOR */
#define PMIC_VDD_IO_VOLT_ACCELEROMETER      19  /**< io voltage for ACCELEROMETER */
#define PMIC_VDD_IO_VOLT_MAGNETOMETER       20  /**< io voltage for MAGNETOMETER */
#define PMIC_VDD_IO_VOLT_GYROSCOPE          21  /**< io voltage for GYROSCOPE */
#define PMIC_VDD_CAM_PRIM_ANALOG            22  /**<  */
#define PMIC_VDD_CAM_SEC_ANALOG             23  /**<  */
#define PMIC_VDD_PRIM_DISPLAY               24  /**<  */
#define PMIC_VDD_TOUCH_SENSOR               25  /**< Supply voltage for TOUCH_SENSOR */
#define PMIC_VDD_PROXIMITY_SENSOR           26  /**< Supply voltage for PROXIMITY_SENSOR */
#define PMIC_VDD_ACCELEROMETER              27  /**< Supply voltage for ACCELEROMETER */
#define PMIC_VDD_MAGNETOMETER               28  /**< Supply voltage for MAGNETOMETER */
#define PMIC_VDD_GYROSCOPE                  29  /**< Supply voltage for GYROSCOPE */
#define PMIC_VDD_EMIC_IO                    30  /**< Supply for TX */
#define PMIC_VDD_ABB_GNSS_TCXO_LDO          31  /**<  */
#define PMIC_VDDP_VEMMC_P                   32  /**<  */
#define PMIC_VDD_CAM_PRIM_DIGITAL           33  /**<  */
#define PMIC_VDD_CAM_PRIM_IO                34  /**<  */
#define PMIC_VDD_CAM_PRIM_AF                35  /**<  */
#define PMIC_VDD_CAM_SEC_DIGITAL            36  /**<  */
#define PMIC_VDD_CAM_SEC_IO                 37  /**<  */
#define PMIC_VDD_CAM_SEC_AF                 38  /**<  */
#define PMIC_VDD_PRIM_DISP_BACKLIGHT        39  /**<  */
#define PMIC_VDD_SD_CARD                    40
#define PMIC_VDD_EMMC                       41
#define PMIC_VDD_VCPU                       42  /**< Supply voltage for CPU domain */
#define PMIC_VDD_LVDS_1V8                   43
#define PMIC_VDD_H_5V                       44
#define PMIC_VDD_H_3V3                      45
#define PMIC_VDD_GPU                        46
#define PMIC_VDD_EMIC_RET                   47
#define PMIC_VDD_LCD_DSI                    48
#define PMIC_VDD_GPS_2V5                    49  /*Supply voltage for GPS PA*/
#define PMIC_VDD_DIF_LCD_RGB                (PMIC_VDD_GPS_2V5 + 1)
#define PMIC_VDD_OTG_5V                     (PMIC_VDD_DIF_LCD_RGB + 1)
#define PMIC_VDD_SPECIAL_POW_1              (PMIC_VDD_OTG_5V + 1)
#define PMIC_VDD_SPECIAL_POW_2              (PMIC_VDD_SPECIAL_POW_1 + 1)
#define PMIC_VDD_SPECIAL_POW_3              (PMIC_VDD_SPECIAL_POW_2 + 1)
#define PMIC_VDD_DEPRECATED                 (PMIC_VDD_SPECIAL_POW_3 + 1)
#define PMIC_MAX_VOLTAGE_ID                 0x7FFFFFFF

/* phys domapn */
#define PMIC_DOMAIN_LSIM1           0       /**< LSIM1 power domain */
#define PMIC_DOMAIN_LSIM2           1       /**< LSIM2 power domain */
#define PMIC_DOMAIN_LAUX1           2       /**< LAUX1 power domain */
#define PMIC_DOMAIN_LAUX2           3       /**< LAUX2 power domain */
#define PMIC_DOMAIN_LMMC1           4       /**< LMMC1 power domain */
#define PMIC_DOMAIN_LUSB            5       /**< LUSB power domain */
#define PMIC_DOMAIN_LPMU            6       /**< LPMU power domain */
#define PMIC_DOMAIN_LAIF            7       /**< LAIF power domain */
#define PMIC_DOMAIN_LCABB           8       /**< LCABB power domain */
#define PMIC_DOMAIN_LMEM            9       /**< LMEM power domain */
#define PMIC_DOMAIN_LMIPI           10      /**< LMIPI power domain */
#define PMIC_DOMAIN_LPLL            11      /**< LPLL power domain */
#define PMIC_DOMAIN_SD1             12      /**< SD1 power domain */
#define PMIC_DOMAIN_SD2             13      /**< SD2 power domain */
#define PMIC_DOMAIN_ELDO_VMMC       14      /**< EMMC power domain provided by external LDO - controlled via GPIO */
#define PMIC_DOMAIN_GNSS_TCXO_LDO   15      /**< GNSS_TCXO_LDO power domain provided by external LDO */
#define PMIC_DOMAIN_PMIC_LDO1       16
#define PMIC_DOMAIN_PMIC_LDO2       17
#define PMIC_DOMAIN_PMIC_LDO3       18
#define PMIC_DOMAIN_PMIC_LDO4       19
#define PMIC_DOMAIN_PMIC_LDO5       20
#define PMIC_DOMAIN_PMIC_LDO6       21
#define PMIC_DOMAIN_PMIC_LDO7       22
#define PMIC_DOMAIN_PMIC_LDO8       23
#define PMIC_DOMAIN_PMIC_LDO9       24
#define PMIC_DOMAIN_PMIC_BUCK1      25
#define PMIC_DOMAIN_PMIC_BUCK2      26
#define PMIC_DOMAIN_PMIC_BUCK3      27
#define PMIC_DOMAIN_PMIC_BUCK4      28
#define PMIC_DOMAIN_PMIC_BOOST      29
#define PMIC_DOMAIN_PMIC_VSWOUT     30
#define PMIC_DOMAIN_PMIC_OTG_5V     (PMIC_DOMAIN_PMIC_VSWOUT + 1)
#define PMIC_DOMAIN_SYR827_VSEL0    (PMIC_DOMAIN_PMIC_OTG_5V + 1)      /**< SYR827 VSEL0 power domain */
#define PMIC_DOMAIN_SYR828_VSEL0    (PMIC_DOMAIN_SYR827_VSEL0 + 1)      /**< SYR828 VSEL0 power domain */
#define PMIC_DOMAIN_XZ3215_BUCK     (PMIC_DOMAIN_SYR828_VSEL0 + 1)
#define PMIC_DOMAIN_XZ3216_BUCK     (PMIC_DOMAIN_XZ3215_BUCK + 1)
#define PMIC_DOMAIN_DUMMY           (PMIC_DOMAIN_XZ3216_BUCK + 1) /**< Dummy power domain */
#define PMIC_NOF_DOMAINS            (PMIC_DOMAIN_DUMMY + 1)      /**< End indicator, hard coded value */
#define PMIC_DOMAIN_H_5V            (PMIC_NOF_DOMAINS + 1)
#define PMIC_DOMAIN_NOTCFG          0xFE    /**< Not configurable domain, hard coded value */
#define PMIC_DOMAIN_UNDEF           0xFF    /**< Undefined domain, hard coded value */

#endif
