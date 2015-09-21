#ifndef __ASM_MACH_MFP_PXA988_H
#define __ASM_MACH_MFP_PXA988_H

#include <plat/mfp.h>

#define GPIO000_KP_MKIN0		MFP_CFG(GPIO0, AF1)
#define GPIO001_KP_MKOUT0		MFP_CFG(GPIO1, AF1)
#define GPIO002_KP_MKIN1		MFP_CFG(GPIO2, AF1)
#define GPIO003_GPIO_3			MFP_CFG(GPIO3, AF0)
#define GPIO004_GPIO_4			MFP_CFG(GPIO4, AF0)
#define GPIO005_GPIO_5			MFP_CFG(GPIO5, AF0)
#define GPIO006_GPIO_6			MFP_CFG(GPIO6, AF0)

#define GPIO007_GPIO_7			MFP_CFG(GPIO7, AF0)
#define GPIO008_GPIO_8			MFP_CFG(GPIO8, AF0)
#define GPIO009_GPIO_9			MFP_CFG(GPIO9, AF0)
#define GPIO010_GPIO_10			MFP_CFG(GPIO10, AF0)
#define GPIO011_GPIO_11			MFP_CFG(GPIO11, AF0)
#define GPIO012_GPIO_12			MFP_CFG(GPIO12, AF0)
#define GPIO013_KP_DKIN4		MFP_CFG(GPIO13, AF4)
#define GPIO014_GPIO_14			MFP_CFG(GPIO14, AF0)
#define GPIO015_GPIO_15			MFP_CFG(GPIO15, AF0)
#define GPIO016_GPIO_16			MFP_CFG(GPIO16, AF0)
#define GPIO017_GPIO_17			MFP_CFG(GPIO17, AF0)
#define GPIO018_GPIO_18			MFP_CFG(GPIO18, AF0)
#define GPIO019_GPIO_19			MFP_CFG(GPIO19, AF0)
#define GPIO020_GPIO_20			MFP_CFG(GPIO20, AF0)

#define GPIO021_I2S_BITCLK		MFP_CFG(GPIO21, AF1)
#define GPIO022_I2S_SYNC		MFP_CFG(GPIO22, AF1)
#define GPIO023_I2S_DATA_OUT		MFP_CFG(GPIO23, AF1)
#define GPIO024_I2S_SDATA_IN		MFP_CFG(GPIO24, AF1)

#define GPIO021_GPIO_21			MFP_CFG(GPIO21, AF0)
#define GPIO022_GPIO_22			MFP_CFG(GPIO22, AF0)
#define GPIO023_GPIO_23                 MFP_CFG(GPIO23, AF0)
#define GPIO024_GPIO_24                 MFP_CFG(GPIO24, AF0)

#define GPIO025_GSSP_SCLK		MFP_CFG(GPIO25, AF1)
#define GPIO026_GSSP_SFRM		MFP_CFG(GPIO26, AF1)
#define GPIO027_GSSP_TXD		MFP_CFG(GPIO27, AF1)
#define GPIO028_GSSP_RXD		MFP_CFG(GPIO28, AF1)

#define GPIO025_GPIO_INPUT		(MFP_CFG(GPIO25, AF0) | MFP_LPM_INPUT)
#define GPIO026_GPIO_INPUT		(MFP_CFG(GPIO26, AF0) | MFP_LPM_INPUT)
#define GPIO027_GPIO_INPUT		(MFP_CFG(GPIO27, AF0) | MFP_LPM_INPUT)
#define GPIO028_GPIO_INPUT		(MFP_CFG(GPIO28, AF0) | MFP_LPM_INPUT)

#define GPIO029_GPIO_29			MFP_CFG(GPIO29, AF0)
#define GPIO030_GPIO_30			MFP_CFG(GPIO30, AF0)
#define GPIO029_UART3_CTS			MFP_CFG(GPIO29, AF4)
#define GPIO030_UART3_RTS			MFP_CFG(GPIO30, AF4)
#define GPIO031_GPIO_31			MFP_CFG(GPIO31, AF0)
#define GPIO032_GPIO_32			MFP_CFG(GPIO32, AF0)

#define GPIO033_GPIO_33			MFP_CFG(GPIO33, AF0)
//#define GPIO034_SPI_CS0			MFP_CFG(GPIO34, AF2)
#define GPIO034_GPIO_34			MFP_CFG(GPIO34, AF0)
#define GPIO035_GPIO_35		MFP_CFG(GPIO35, AF0)
#define GPIO036_GPIO_36		MFP_CFG(GPIO36, AF0)

#define GPIO037_MMC2_DATA3		MFP_CFG(GPIO37, AF1)
#define GPIO038_MMC2_DATA2		MFP_CFG(GPIO38, AF1)
#define GPIO039_MMC2_DATA1		MFP_CFG(GPIO39, AF1)
#define GPIO040_MMC2_DATA0		MFP_CFG(GPIO40, AF1)
#define GPIO041_MMC2_CMD		MFP_CFG_DRV(GPIO41, AF1, DS01X)
#define GPIO042_MMC2_CLK		MFP_CFG_DRV(GPIO42, AF1, DS01X)

#define GPIO043_GPIO_43			MFP_CFG(GPIO43, AF0)
#define GPIO044_GPIO_44			MFP_CFG(GPIO44, AF0)

#define GPIO043_GPIO_43_Ax		MFP_CFG(GPIO43, AF6)
#define GPIO044_GPIO_44_Ax		MFP_CFG(GPIO44, AF6)

#define GPIO045_UART2_RXD		MFP_CFG(GPIO45, AF1)
#define GPIO046_UART2_TXD		MFP_CFG(GPIO46, AF1)

#define GPIO047_UART1_RXD		MFP_CFG(GPIO47, AF6)
#define GPIO048_UART1_TXD		MFP_CFG(GPIO48, AF6)

#define GPIO049_GPIO_49			MFP_CFG(GPIO49, AF0)
#define GPIO050_GPIO_50			MFP_CFG(GPIO50, AF0)

#define GPIO051_UART0_RXD		MFP_CFG(GPIO51, AF1)
#define GPIO052_UART0_TXD		MFP_CFG(GPIO52, AF1)

#define GPIO053_BT_UART_URTS		MFP_CFG(GPIO53, AF1)
#define GPIO054_BT_UART_UCTS		MFP_CFG(GPIO54, AF1)
#define GPIO051_GPIO_51			MFP_CFG(GPIO51, AF0)
#define GPIO052_GPIO_52			MFP_CFG(GPIO52, AF0)
#define GPIO053_GPIO_53			MFP_CFG(GPIO53, AF0)
#define GPIO054_GPIO_54			MFP_CFG(GPIO54, AF0)

#define GPIO067_GPIO_67		MFP_CFG(GPIO67, AF0)
#define GPIO068_GPIO_68		MFP_CFG(GPIO68, AF0)
#define GPIO069_GPIO_69		MFP_CFG(GPIO69, AF0)
#define GPIO070_GPIO_70		MFP_CFG(GPIO70, AF0)
#define GPIO071_GPIO_71		MFP_CFG(GPIO71, AF0)
#define GPIO072_GPIO_72		MFP_CFG(GPIO72, AF0)
#define GPIO073_GPIO_73		MFP_CFG(GPIO73, AF0)
#define GPIO074_GPIO_74		MFP_CFG(GPIO74, AF0)
#define GPIO075_GPIO_75		MFP_CFG(GPIO75, AF0)
#define GPIO076_GPIO_76		MFP_CFG(GPIO76, AF0)
#define GPIO077_GPIO_77		MFP_CFG(GPIO77, AF0)
#define GPIO078_GPIO_78		MFP_CFG(GPIO78, AF0)

#define GPIO067_CCIC_IN7		MFP_CFG(GPIO67, AF1)
#define GPIO068_CCIC_IN6		MFP_CFG(GPIO68, AF1)
#define GPIO069_CCIC_IN5		MFP_CFG(GPIO69, AF1)
#define GPIO070_CCIC_IN4		MFP_CFG(GPIO70, AF1)
#define GPIO071_CCIC_IN3		MFP_CFG(GPIO71, AF1)
#define GPIO072_CCIC_IN2		MFP_CFG(GPIO72, AF1)
#define GPIO073_CCIC_IN1		MFP_CFG(GPIO73, AF1)
#define GPIO074_CCIC_IN0		MFP_CFG(GPIO74, AF1)
#define GPIO075_CAM_HSYNC		MFP_CFG(GPIO75, AF1)
#define GPIO076_CAM_VSYNC		MFP_CFG(GPIO76, AF1)
#define GPIO077_CAM_MCLK		MFP_CFG(GPIO77, AF1)
#define GPIO078_CAM_PCLK		MFP_CFG(GPIO78, AF1)
#define GPIO079_CAM_SCL			MFP_CFG(GPIO79, AF1)
#define GPIO080_CAM_SDA			MFP_CFG(GPIO80, AF1)

#define GPIO079_GPIO_79			MFP_CFG(GPIO79, AF0)
#define GPIO080_GPIO_80			MFP_CFG(GPIO80, AF0)

#define GPIO081_GPIO_81			MFP_CFG(GPIO81, AF0)
#define GPIO082_GPIO_82			MFP_CFG(GPIO82, AF0)
#define GPIO083_GPIO_83			MFP_CFG(GPIO83, AF0)
#define GPIO084_GPIO_84			MFP_CFG(GPIO84, AF0)
#define GPIO085_GPIO_85			MFP_CFG(GPIO85, AF0)
#define GPIO086_GPIO_86			MFP_CFG(GPIO86, AF0)

#define GPIO087_CI2C_SCL_2		MFP_CFG(GPIO87, AF5)
#define GPIO088_CI2C_SDA_2		MFP_CFG(GPIO88, AF5)
#define GPIO087_GPIO_87			MFP_CFG(GPIO87, AF0)
#define GPIO088_GPIO_88			MFP_CFG(GPIO88, AF0)

#define GPIO089_GPIO_89			MFP_CFG(GPIO89, AF0)
#define GPIO089_GPS_CLK			MFP_CFG(GPIO89, AF5)
#define GPIO090_GPIO_90		MFP_CFG(GPIO90, AF0)

#define GPIO091_GPIO_91			MFP_CFG(GPIO91, AF0)
#define GPIO092_GPIO_92			MFP_CFG(GPIO92, AF0)
#define GPIO093_GPIO_93			MFP_CFG(GPIO93, AF0)
#define GPIO094_GPIO_94			MFP_CFG(GPIO94, AF0)
#define GPIO095_GPIO_95			MFP_CFG(GPIO95, AF0)
#define GPIO096_GPIO_96			MFP_CFG(GPIO96, AF0)
#define GPIO097_GPIO_97			MFP_CFG(GPIO97, AF0)
#define GPIO098_GPIO_98			MFP_CFG(GPIO98, AF0)
#define GPIO104_FIX_NONE		MFP_CFG(GPIO104, AF7)
#define GPIO124_GPIO_124		MFP_CFG(GPIO124, AF0)
#define GPIO126_GPIO_126		MFP_CFG(GPIO126, AF0)
#define GPIO127_GPIO_127		MFP_CFG(GPIO127, AF0)

#define GPIO125_VCXO_REQ		MFP_CFG(VCXOREQ, AF1)


#define MMC1_DAT7_MMC1_DAT7		MFP_CFG(MMC1_DAT7, AF0)
#define MMC1_DAT6_MMC1_DAT6		MFP_CFG(MMC1_DAT6, AF0)
#define MMC1_DAT5_MMC1_DAT5		MFP_CFG(MMC1_DAT5, AF0)
#define MMC1_DAT4_MMC1_DAT4		MFP_CFG(MMC1_DAT4, AF0)
#define MMC1_DAT3_MMC1_DAT3		MFP_CFG(MMC1_DAT3, AF0)
#define MMC1_DAT2_MMC1_DAT2		MFP_CFG(MMC1_DAT2, AF0)
#define MMC1_DAT1_MMC1_DAT1		MFP_CFG(MMC1_DAT1, AF0)
#define MMC1_DAT0_MMC1_DAT0		MFP_CFG(MMC1_DAT0, AF0)
#define MMC1_CMD_MMC1_CMD		MFP_CFG(MMC1_CMD, AF0)
#define MMC1_CLK_MMC1_CLK		MFP_CFG(MMC1_CLK, AF0)
#define MMC1_CD_MMC1_CD			MFP_CFG(MMC1_CD, AF1)
#define MMC1_WP_MMC1_WP			MFP_CFG(MMC1_WP, AF1) //GPIO99

#define ND_IO7_MMC3_DAT7		MFP_CFG(DF_IO7, AF1)
#define ND_IO6_MMC3_DAT6		MFP_CFG(DF_IO6, AF1)
#define ND_IO5_MMC3_DAT5		MFP_CFG(DF_IO5, AF1)
#define ND_IO4_MMC3_DAT4		MFP_CFG(DF_IO4, AF1)
#define ND_IO3_MMC3_DAT3		MFP_CFG(DF_IO3, AF1)
#define ND_IO2_MMC3_DAT2		MFP_CFG(DF_IO2, AF1)
#define ND_IO1_MMC3_DAT1		MFP_CFG(DF_IO1, AF1)
#define ND_IO0_MMC3_DAT0		MFP_CFG(DF_IO0, AF1)
#define ND_CLE_SM_OEN_MMC3_CMD		MFP_CFG(DF_CLE_SM_OEn, AF1)
//#define SM_SCLK_MMC3_CLK		MFP_CFG(SM_SCLK, AF1)
#define SM_SCLK_MMC3_CLK		MFP_CFG_DRV(SM_SCLK, AF1, DS02X)
#define SM_BEN0_MMC3_RSTN		MFP_CFG(SM_BE0, AF2)
#define SM_BEN0_GPIO126         MFP_CFG(SM_BE0, AF0)

#define ANT_SW4_GPIO_28			MFP_CFG(ANT_SW4, AF6)
#define SM_ADV_GPIO_0			MFP_CFG(SM_ADV, AF1)
#define ND_RDY1_GPIO_1			MFP_CFG(DF_RDY1, AF1)
#define SM_ADVMUX_GPIO_2		MFP_CFG(SM_ADVMUX, AF1)
#define SM_RDY_GPIO_3			MFP_CFG(SM_RDY, AF1)
#define SM_BEN1_GPIO_127		MFP_CFG(SM_BE1, AF0)
#define SM_CSN0_GPIO_103		MFP_CFG(SM_nCS0, AF1)
#define SM_CSN1_GPIO_104		MFP_CFG(SM_nCS1, AF1)
#define ND_CS1N3_GPIO_102		MFP_CFG(DF_nCS1_SM_nCS3, AF1)
#define GPIO_ND_IO15_DAT15		MFP_CFG(DF_IO15, AF1)
#define GPIO_ND_IO13_DAT13      MFP_CFG(DF_IO13, AF1)
#define GPIO_ND_IO12_DAT12      MFP_CFG(DF_IO12, AF1)
#define GPIO_ND_IO11_DAT11      MFP_CFG(DF_IO11, AF1)
#define GPIO_ND_IO10_DAT10      MFP_CFG(DF_IO10, AF1)
#define GPIO_ND_IO9_DAT9        MFP_CFG(DF_IO9, AF1)

#define ND_IO15_ND_DAT15		MFP_CFG(DF_IO15, AF1) //GPIO60
#define ND_IO14_ND_DAT14		MFP_CFG(DF_IO14, AF1) //GPIO61
#define ND_IO13_ND_DAT13		MFP_CFG(DF_IO13, AF1) //GPIO62
#define ND_IO12_ND_DAT12		MFP_CFG(DF_IO12, AF1) //GPIO63
#define ND_IO11_ND_DAT11		MFP_CFG(DF_IO11, AF1) //GPIO64
#define ND_IO10_ND_DAT10		MFP_CFG(DF_IO10, AF1) //GPIO65
#define ND_IO9_ND_DAT9			MFP_CFG(DF_IO9, AF1) //GPIO66
#define ND_IO8_ND_DAT8			MFP_CFG(DF_IO8, AF1) //GPIO100
#define ND_nCS0_SM_nCS2			MFP_CFG(DF_nCS0_SM_nCS2, AF1) //GPIO101
#define DF_ALE_SM_WEn_ND_ALE		MFP_CFG(DF_ALE_SM_WEn, AF0) //GPIO107
#define DF_WEn_DF_WEn			MFP_CFG(DF_WEn, AF0) //GPIO105
#define DF_REn_DF_REn			MFP_CFG(DF_REn, AF0) //GPIO106
#define DF_RDY0_DF_RDY0			MFP_CFG(DF_RDY0, AF1) //GPIO108

#endif /* __ASM_MACH_MFP_PXA988_H */
