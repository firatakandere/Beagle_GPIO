#ifndef BEAGLE_GPIO_H
#define BEAGLE_GPIO_H

#include <iostream>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define GPIO_ERROR(msg)	std::cout << "[GPIO] Error : " << msg << std::endl;

#define BEAGLE_GPIO_DEBUG
#ifdef BEAGLE_GPIO_DEBUG
#define GPIO_PRINT(msg)	std::cout << "[GPIO] : " << msg << std::endl;
#define assert( condition ) 	\
    if (!(condition))	\
{			\
    GPIO_ERROR( "Assert Failed in file '" << __FILE__ << "' on line " << __LINE__ );	\
    exit(0);	\
    }

#else
#define GPIO_PRINT(msg)
#define assert( condition )
#endif

/**
 * @brief GPIO Control Class for BeagleBone Black
 */
class Beagle_GPIO
{
public:
    /** @brief Return status */
    typedef enum
    {
        kFail 		= 0, /*!< Failure */
        kSuccess 	= 1 /*!< Success */
    } Beagle_GPIO_Status;

    /** @brief BeagleBone GPIO Register Offsets */
    enum
    {
        kREVISION           = 0x0,
        kSYSCONFIG          = 0x10,
        kIRQSTATUS_RAW_0	= 0x24,
        kIRQSTATUS_RAW_1	= 0x28,
        kIRQSTATUS_0		= 0x2C,
        kIRQSTATUS_1		= 0x30,
        kIRQSTATUS_SET_0	= 0x34,
        kIRQSTATUS_SET_1	= 0x38,
        kIRQSTATUS_CLR_0	= 0x3C,
        kIRQSTATUS_CLR_1	= 0x40,
        kIRQWAKEN_0         = 0x44,
        kIRQWAKEN_1         = 0x48,
        kSYSSTATUS          = 0x114,
        kCTRL   			= 0x130,
        kOE             	= 0x134,
        kDATAIN             = 0x138,
        kDATAOUT            = 0x13C,
        kLEVELDETECT0		= 0x140,
        kLEVELDETECT1		= 0x144,
        kRISINGDETECT		= 0x148,
        kFALLINGDETECT		= 0x14C,
        kDEBOUNCEENABLE		= 0x150,
        kDEBOUNCINGTIME		= 0x154,
        kCLEARDATAOUT		= 0x190,
        kSETDATAOUT         = 0x194
    } Beagle_GPIO_Registers;

    /** @brief Input / Output Pin Modes */
    typedef enum
    {
        kINPUT	= 0,/*!<INPUT Mode*/
        kOUTPUT = 1/*!<OUTPUT Mode*/
    } Beagle_GPIO_Direction;

    /** @brief GPIO Pin Mapping */
    enum
    {
        P8_1,/*!<DGND*/  P8_2,/*!<DGND*/  P8_3,/*!<MMC1_DAT6*/  P8_4,/*!<MMC1_DAT7*/  P8_5,/*!<MMC1_DAT2*/
        P8_6,/*!<MMC1_DAT3*/  P8_7,/*!<GPIO_66*/  P8_8,/*!<GPIO_67*/  P8_9,/*!<GPIO_69*/  P8_10,/*!<GPIO_68*/
        P8_11,/*!<GPIO_45*/ P8_12,/*!<GPIO_44*/ P8_13,/*!<EHRPWM2B*/ P8_14,/*!<GPIO_26*/ P8_15,/*!<GPIO_47*/
        P8_16,/*!<GPIO_46*/ P8_17,/*!<GPIO_27*/ P8_18,/*!<GPIO_18*/ P8_19,/*!<EHRPWM2A*/ P8_20,/*!<MMC1_CMD*/
        P8_21,/*!<MMC1_CLK*/ P8_22,/*!<MMC1_DAT5*/ P8_23,/*!<MMC1_DAT4*/ P8_24,/*!<MMC1_DAT1*/ P8_25,/*!<MMC1_DAT0*/
        P8_26,/*!<GPIO_61*/ P8_27,/*!<LCD_VSYNC*/ P8_28,/*!<LCD_PCLK*/ P8_29,/*!<LCD_HSYNC*/ P8_30,/*!<LCD_AC_BIAS*/
        P8_31,/*!<LCD_DATA14*/ P8_32,/*!<LCD_DATA15*/ P8_33,/*!<LCD_DATA13*/ P8_34,/*!<LCD_DATA11*/ P8_35,/*!<LCD_DATA12*/
        P8_36,/*!<LCD_DATA10*/ P8_37,/*!<LCD_DATA8*/ P8_38,/*!<LCD_DATA9*/ P8_39,/*!<LCD_DATA6*/ P8_40,/*!<LCD_DATA7*/
        P8_41,/*!<LCD_DATA4*/ P8_42,/*!<LCD_DATA5*/ P8_43,/*!<LCD_DATA2*/ P8_44,/*!<LCD_DATA3*/ P8_45,/*!<LCD_DATA0*/
        P8_46,/*!<LCD_DATA1*/
        P9_1,/*!<DGND*/  P9_2,/*!<DGND*/  P9_3,/*!<VDD_3V3*/  P9_4,/*!<VDD_3V3*/  P9_5,/*!<VDD_5V*/
        P9_6,/*!<VDD_5V*/  P9_7,/*!<SYS_5V*/  P9_8,/*!<SYS_5V*/  P9_9,/*!<PWR_BUT*/  P9_10,/*!<SYS_RESETN*/
        P9_11,/*!<UARD4_RXD*/ P9_12,/*!<GPIO_60*/ P9_13,/*!<UART4_TXD*/ P9_14,/*!<EHRPWM1A*/ P9_15,/*!<GPIO_48*/
        P9_16,/*!<EHRPWM1B*/ P9_17,/*!<SPIO_CSO*/ P9_18,/*!<SPIO_D1*/ P9_19,/*!<I2C2_SCL*/ P9_20,/*!<I2C2_SDA*/
        P9_21,/*!<SPIO_D0*/ P9_22,/*!<SPIO_SCLK*/ P9_23,/*!<GPIO_49*/ P9_24,/*!<UART1_TXD*/ P9_25,/*!<GPIO_117*/
        P9_26,/*!<UART1_RXD*/ P9_27,/*!<GPIO_115*/ P9_28,/*!<SPI1_CS0*/ P9_29,/*!<SPI1_D0*/ P9_30,/*!<GPIO_112*/
        P9_31,/*!<SPI1_SCLK*/ P9_32,/*!<VDD_ADC*/ P9_33,/*!<AIN4*/ P9_34,/*!<GNDA_ADC*/ P9_35,/*!<AIN6*/
        P9_36,/*!<AIN5*/ P9_37,/*!<AIN2*/ P9_38,/*!<AIN3*/ P9_39,/*!<AIN0*/ P9_40,/*!<AIN1*/
        P9_41,/*!<GPIO_20*/ P9_42,/*!<ECAPPWM0*/ P9_43,/*!<DGND*/ P9_44,/*!<DGND*/ P9_45,/*!<DGND*/
        P9_46/*!<DGND*/
    } GPIO_Pins;

    /** @brief IO Banks for GPIOs */
    static const int GPIO_Pin_Bank[];

    /** @brief Pin ID for GPIOs */
    static const int GPIO_Pin_Id[];

    /** @brief Pad Control Register */
    static const unsigned long GPIO_Pad_Control[];

    /** @brief Base address of Control Module Registers */
    static const unsigned long GPIO_Control_Module_Registers;

    /** @brief Base addresses of GPIO Modules */
    static const unsigned long GPIO_Base[];

public:
    /**
     * @brief Beagle_GPIO Constructor
     */
    Beagle_GPIO();
    /**
     * @brief Beagle_GPIO Destructor
     */
    ~Beagle_GPIO();

public:
    /**
     * @brief Configure pin as input or output
     * @param _pin Pin number
     * @param _direction GPIO Directoion
     * @return GPIO Status
     */
    Beagle_GPIO_Status configurePin( unsigned short _pin, Beagle_GPIO_Direction _direction );

    /**
     * @brief Enable or disable pin interrupts
     * @param _pin Pin number
     * @param _enable True to enable, false to disable
     * @return GPIO Status
     */
    Beagle_GPIO_Status enablePinInterrupts( unsigned short _pin, bool _enable );

    /**
     * @brief Write a value to pin
     * @param _pin Pin number
     * @param _value Value to be written
     * @return GPIO Status
     */
    Beagle_GPIO_Status writePin( unsigned short _pin, unsigned char _value );

    /**
     * @brief readPin Read a value from a pin
     * @param _pin Pin number
     * @return Pin value
     */
    unsigned char readPin( unsigned short _pin );

    /**
     * @brief Open SPI Channel
     * @param _mode SPI Mode
     * @param _bits SPI Bits
     * @param _speed SPI Speed
     * @param _delay SPI Delay
     */
    void openSPI( unsigned char _mode=0,
                  unsigned char _bits=8,
                  unsigned long _speed=4800000,
                  unsigned short _delay=0 );

    /**
     * @brief Close SPI Channel
     */
    void closeSPI();

    /**
     * @brief Send SPI Buffer
     * @param buffer SPI Buffer
     * @param size SPI Buffer Size
     */
    void sendSPIBuffer( unsigned long buffer, int size );

    /**
     * @brief Check if the module is active
     * @return Returns true if the module is active otherwise false.
     */
    bool isActive() { return m_active; }

private:
    bool                m_active;
    int                 m_gpio_fd;
    unsigned long       *m_controlModule;
    unsigned long       *m_gpio[4];

    int                 m_spi_fd;
    unsigned char       *m_spi_buffer_rx;
    unsigned char       m_spi_mode;
    unsigned char       m_spi_bits;
    unsigned long       m_spi_speed;
    unsigned short      m_spi_delay;

    struct spi_ioc_transfer m_spi_ioc_tr;
};

#endif // BEAGLE_GPIO_H
