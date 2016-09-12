
--[[
    LuaJIT Binding to pigpio library:
    
    https://github.com/joan2937/pigpio
    
]]

local ffi = require("ffi")


ffi.cdef[[
typedef struct
{
   uint16_t func;
   uint16_t size;
} gpioHeader_t;

typedef struct
{
   size_t size;
   void *ptr;
   uint32_t data;
} gpioExtent_t;

typedef struct
{
   uint32_t tick;
   uint32_t level;
} gpioSample_t;

typedef struct
{
   uint16_t seqno;
   uint16_t flags;
   uint32_t tick;
   uint32_t level;
} gpioReport_t;

typedef struct
{
   uint32_t gpioOn;
   uint32_t gpioOff;
   uint32_t usDelay;
} gpioPulse_t;
]]

ffi.cdef[[
typedef struct
{
   uint32_t gpioOn;
   uint32_t gpioOff;
   uint32_t usDelay;
   uint32_t flags;
} rawWave_t;
]]

ffi.cdef[[
typedef struct
{
   uint16_t botCB;  /* first CB used by wave  */
   uint16_t topCB;  /* last CB used by wave   */
   uint16_t botOOL; /* first bottom OOL used by wave  */
                    /* botOOL to botOOL + numBOOL -1 are in use */
   uint16_t topOOL; /* last top OOL used by wave */
                    /* topOOL - numTOOL to topOOL are in use.*/
   uint16_t deleted;
   uint16_t numCB;
   uint16_t numBOOL;
   uint16_t numTOOL;
} rawWaveInfo_t;

typedef struct
{
   int clk;     /* GPIO for clock           */
   int mosi;    /* GPIO for MOSI            */
   int miso;    /* GPIO for MISO            */
   int ss_pol;  /* slave select off state   */
   int ss_us;   /* delay after slave select */
   int clk_pol; /* clock off state          */
   int clk_pha; /* clock phase              */
   int clk_us;  /* clock micros             */
} rawSPI_t;

typedef struct { /* linux/arch/arm/mach-bcm2708/include/mach/dma.h */
   uint32_t info;
   uint32_t src;
   uint32_t dst;
   uint32_t length;
   uint32_t stride;
   uint32_t next;
   uint32_t pad[2];
} rawCbs_t;

typedef struct
{
   uint16_t addr;  /* slave address       */
   uint16_t flags;
   uint16_t len;   /* msg length          */
   uint8_t  *buf;  /* pointer to msg data */
} pi_i2c_msg_t;
]]

ffi.cdef[[
typedef void (*gpioAlertFunc_t)    (int      gpio,
                                    int      level,
                                    uint32_t tick);

typedef void (*gpioAlertFuncEx_t)  (int      gpio,
                                    int      level,
                                    uint32_t tick,
                                    void    *userdata);

typedef void (*gpioISRFunc_t)      (int      gpio,
                                    int      level,
                                    uint32_t tick);

typedef void (*gpioISRFuncEx_t)    (int      gpio,
                                    int      level,
                                    uint32_t tick,
                                    void    *userdata);

typedef void (*gpioTimerFunc_t)    (void);

typedef void (*gpioTimerFuncEx_t)  (void *userdata);

typedef void (*gpioSignalFunc_t)   (int signum);

typedef void (*gpioSignalFuncEx_t) (int    signum,
                                    void  *userdata);

typedef void (*gpioGetSamplesFunc_t)   (const gpioSample_t *samples,
                                        int                 numSamples);

typedef void (*gpioGetSamplesFuncEx_t) (const gpioSample_t *samples,
                                        int                 numSamples,
                                        void               *userdata);

typedef void *(gpioThreadFunc_t) (void *);
]]

ffi.cdef[[
/* gpio: 0-53 */

static const int PI_MIN_GPIO     =  0;
static const int PI_MAX_GPIO     = 53;

/* user_gpio: 0-31 */

static const int PI_MAX_USER_GPIO =31;

/* level: 0-1 */

static const int PI_OFF   =0;
static const int PI_ON    =1;

static const int PI_CLEAR =0;
static const int PI_SET   =1;

static const int PI_LOW   =0;
static const int PI_HIGH  =1;

/* level: only reported for GPIO time-out, see gpioSetWatchdog */

static const int PI_TIMEOUT =2;

/* mode: 0-7 */

static const int PI_INPUT  =0;
static const int PI_OUTPUT =1;
static const int PI_ALT0   =4;
static const int PI_ALT1   =5;
static const int PI_ALT2   =6;
static const int PI_ALT3   =7;
static const int PI_ALT4   =3;
static const int PI_ALT5   =2;

/* pud: 0-2 */

static const int PI_PUD_OFF  =0;
static const int PI_PUD_DOWN =1;
static const int PI_PUD_UP   =2;

/* dutycycle: 0-range */

static const int PI_DEFAULT_DUTYCYCLE_RANGE   =255;

/* range: 25-40000 */

static const int PI_MIN_DUTYCYCLE_RANGE        =25;
static const int PI_MAX_DUTYCYCLE_RANGE     =40000;

/* pulsewidth: 0, 500-2500 */

static const int PI_SERVO_OFF =0;
static const int PI_MIN_SERVO_PULSEWIDTH =500;
static const int PI_MAX_SERVO_PULSEWIDTH =2500;

/* hardware PWM */

static const int PI_HW_PWM_MIN_FREQ =1;
static const int PI_HW_PWM_MAX_FREQ =125000000;
static const int PI_HW_PWM_RANGE =1000000;

/* hardware clock */

static const int PI_HW_CLK_MIN_FREQ =4689;
static const int PI_HW_CLK_MAX_FREQ =250000000;

static const int PI_NOTIFY_SLOTS  =32;

static const int PI_NTFY_FLAGS_ALIVE   = (1 <<6);
static const int PI_NTFY_FLAGS_WDOG    = (1 <<5);

static const int PI_WAVE_BLOCKS     =4;
static const int PI_WAVE_MAX_PULSES =(PI_WAVE_BLOCKS * 3000);
static const int PI_WAVE_MAX_CHARS  =(PI_WAVE_BLOCKS *  300);

static const int PI_BB_I2C_MIN_BAUD    = 50;
static const int PI_BB_I2C_MAX_BAUD =500000;

static const int PI_BB_SER_MIN_BAUD    = 50;
static const int PI_BB_SER_MAX_BAUD =250000;

static const int PI_BB_SER_NORMAL =0;
static const int PI_BB_SER_INVERT =1;

static const int PI_WAVE_MIN_BAUD     = 50;
static const int PI_WAVE_MAX_BAUD =1000000;

static const int PI_SPI_MIN_BAUD    = 32000;
static const int PI_SPI_MAX_BAUD =125000000;

static const int PI_MIN_WAVE_DATABITS =1;
static const int PI_MAX_WAVE_DATABITS =32;

static const int PI_MIN_WAVE_HALFSTOPBITS =2;
static const int PI_MAX_WAVE_HALFSTOPBITS =8;

static const int PI_WAVE_MAX_MICROS =(30 * 60 * 1000000); /* half an hour */

static const int PI_MAX_WAVES = 250;

static const int PI_MAX_WAVE_CYCLES =65535;
static const int PI_MAX_WAVE_DELAY  =65535;

static const int PI_WAVE_COUNT_PAGES =10;

/* wave tx mode */

static const int PI_WAVE_MODE_ONE_SHOT      =0;
static const int PI_WAVE_MODE_REPEAT        =1;
static const int PI_WAVE_MODE_ONE_SHOT_SYNC =2;
static const int PI_WAVE_MODE_REPEAT_SYNC   =3;

/* special wave at return values */

static const int PI_WAVE_NOT_FOUND  =9998; /* Transmitted wave not found. */
static const int PI_NO_TX_WAVE      =9999; /* No wave being transmitted. */

/* Files, I2C, SPI, SER */

static const int PI_FILE_SLOTS =8;
static const int PI_I2C_SLOTS =32;
static const int PI_SPI_SLOTS =16;
static const int PI_SER_SLOTS =8;

static const int PI_MAX_I2C_ADDR =0x7F;

static const int PI_NUM_AUX_SPI_CHANNEL =3;
static const int PI_NUM_STD_SPI_CHANNEL =2;

static const int PI_MAX_I2C_DEVICE_COUNT =(1<<16);
static const int PI_MAX_SPI_DEVICE_COUNT =(1<<16);

/* max pi_i2c_msg_t per transaction */

static const int  PI_I2C_RDRW_IOCTL_MAX_MSGS = 42;

/* flags for i2cTransaction, pi_i2c_msg_t */

static const int PI_I2C_M_WR           =0x0000; /* write data */
static const int PI_I2C_M_RD           =0x0001; /* read data */
static const int PI_I2C_M_TEN          =0x0010; /* ten bit chip address */
static const int PI_I2C_M_RECV_LEN     =0x0400; /* length will be first received byte */
static const int PI_I2C_M_NO_RD_ACK    =0x0800; /* if I2C_FUNC_PROTOCOL_MANGLING */
static const int PI_I2C_M_IGNORE_NAK   =0x1000; /* if I2C_FUNC_PROTOCOL_MANGLING */
static const int PI_I2C_M_REV_DIR_ADDR =0x2000; /* if I2C_FUNC_PROTOCOL_MANGLING */
static const int PI_I2C_M_NOSTART      =0x4000; /* if I2C_FUNC_PROTOCOL_MANGLING */
]]

ffi.cdef[[
/* bbI2CZip and i2cZip commands */

static const int PI_I2C_END         = 0;
static const int PI_I2C_ESC         = 1;
static const int PI_I2C_START       = 2;
static const int PI_I2C_COMBINED_ON = 2;
static const int PI_I2C_STOP        = 3;
static const int PI_I2C_COMBINED_OFF= 3;
static const int PI_I2C_ADDR        = 4;
static const int PI_I2C_FLAGS       = 5;
static const int PI_I2C_READ        = 6;
static const int PI_I2C_WRITE       = 7;
]]



ffi.cdef[[
/* Longest busy delay */

static const int PI_MAX_BUSY_DELAY =100;

/* timeout: 0-60000 */

static const int PI_MIN_WDOG_TIMEOUT =0;
static const int PI_MAX_WDOG_TIMEOUT =60000;

/* timer: 0-9 */

static const int PI_MIN_TIMER =0;
static const int PI_MAX_TIMER =9;

/* millis: 10-60000 */

static const int PI_MIN_MS =10;
static const int PI_MAX_MS =60000;

static const int PI_MAX_SCRIPTS       =32;

static const int PI_MAX_SCRIPT_TAGS   =50;
static const int PI_MAX_SCRIPT_VARS  =150;
static const int PI_MAX_SCRIPT_PARAMS =10;

/* script status */

static const int PI_SCRIPT_INITING =0;
static const int PI_SCRIPT_HALTED  =1;
static const int PI_SCRIPT_RUNNING =2;
static const int PI_SCRIPT_WAITING =3;
static const int PI_SCRIPT_FAILED  =4;

/* signum: 0-63 */

static const int PI_MIN_SIGNUM =0;
static const int PI_MAX_SIGNUM =63;

/* timetype: 0-1 */

static const int PI_TIME_RELATIVE =0;
static const int PI_TIME_ABSOLUTE =1;

static const int PI_MAX_MICS_DELAY =1000000; /* 1 second */
static const int PI_MAX_MILS_DELAY =60000;   /* 60 seconds */

/* cfgMillis */

static const int PI_BUF_MILLIS_MIN =100;
static const int PI_BUF_MILLIS_MAX =10000;

/* cfgMicros: 1, 2, 4, 5, 8, or 10 */

/* cfgPeripheral: 0-1 */

static const int PI_CLOCK_PWM =0;
static const int PI_CLOCK_PCM =1;

/* DMA channel: 0-14 */

static const int PI_MIN_DMA_CHANNEL =0;
static const int PI_MAX_DMA_CHANNEL =14;

/* port */

static const int PI_MIN_SOCKET_PORT =1024;
static const int PI_MAX_SOCKET_PORT =32000;


/* ifFlags: */

static const int PI_DISABLE_FIFO_IF   =1;
static const int PI_DISABLE_SOCK_IF   =2;
static const int PI_LOCALHOST_SOCK_IF =4;

/* memAllocMode */

static const int PI_MEM_ALLOC_AUTO    =0;
static const int PI_MEM_ALLOC_PAGEMAP =1;
static const int PI_MEM_ALLOC_MAILBOX =2;

/* filters */

static const int PI_MAX_STEADY  =300000;
static const int PI_MAX_ACTIVE =1000000;

/* gpioCfgInternals */

static const int PI_CFG_DBG_LEVEL         =0; /* bits 0-3 */
static const int PI_CFG_ALERT_FREQ        =4; /* bits 4-7 */
static const int PI_CFG_RT_PRIORITY       =(1<<8);
static const int PI_CFG_STATS             =(1<<9);

static const int PI_CFG_ILLEGAL_VAL       =(1<<10);

/* gpioISR */

static const int RISING_EDGE  =0;
static const int FALLING_EDGE =1;
static const int EITHER_EDGE  =2;


/* pads */

static const int PI_MAX_PAD =2;

static const int PI_MIN_PAD_STRENGTH =1;
static const int PI_MAX_PAD_STRENGTH =16;

/* files */

static const int PI_FILE_NONE   =0;
static const int PI_FILE_MIN    =1;
static const int PI_FILE_READ   =1;
static const int PI_FILE_WRITE  =2;
static const int PI_FILE_RW     =3;
static const int PI_FILE_APPEND =4;
static const int PI_FILE_CREATE =8;
static const int PI_FILE_TRUNC  =16;
static const int PI_FILE_MAX    =31;

static const int PI_FROM_START   =0;
static const int PI_FROM_CURRENT =1;
static const int PI_FROM_END     =2;
]]

ffi.cdef[[

int gpioInitialise(void);

void gpioTerminate(void);

int gpioSetMode(unsigned gpio, unsigned mode);

int gpioGetMode(unsigned gpio);

int gpioSetPullUpDown(unsigned gpio, unsigned pud);

int gpioRead (unsigned gpio);

int gpioWrite(unsigned gpio, unsigned level);
]]

ffi.cdef[[
int gpioPWM(unsigned user_gpio, unsigned dutycycle);

int gpioGetPWMdutycycle(unsigned user_gpio);

int gpioSetPWMrange(unsigned user_gpio, unsigned range);

int gpioGetPWMrange(unsigned user_gpio);

int gpioGetPWMrealRange(unsigned user_gpio);


int gpioSetPWMfrequency(unsigned user_gpio, unsigned frequency);



int gpioGetPWMfrequency(unsigned user_gpio);
]]

ffi.cdef[[
int gpioServo(unsigned user_gpio, unsigned pulsewidth);

int gpioGetServoPulsewidth(unsigned user_gpio);

int gpioSetAlertFunc(unsigned user_gpio, gpioAlertFunc_t f);

int gpioSetAlertFuncEx(
   unsigned user_gpio, gpioAlertFuncEx_t f, void *userdata);

int gpioSetISRFunc(
   unsigned user_gpio, unsigned edge, int timeout, gpioISRFunc_t f);

int gpioSetISRFuncEx(
   unsigned user_gpio,
   unsigned edge,
   int timeout,
   gpioISRFuncEx_t f,
   void *userdata);

int gpioNotifyOpen(void);

int gpioNotifyOpenWithSize(int bufSize);

int gpioNotifyBegin(unsigned handle, uint32_t bits);

int gpioNotifyPause(unsigned handle);

int gpioNotifyClose(unsigned handle);

int gpioWaveClear(void);

int gpioWaveAddNew(void);

int gpioWaveAddGeneric(unsigned numPulses, gpioPulse_t *pulses);
]]

ffi.cdef[[

int gpioWaveAddSerial
   (unsigned user_gpio,
    unsigned baud,
    unsigned data_bits,
    unsigned stop_bits,
    unsigned offset,
    unsigned numBytes,
    char     *str);




int gpioWaveCreate(void);

int gpioWaveDelete(unsigned wave_id);

int gpioWaveTxSend(unsigned wave_id, unsigned wave_mode);

int gpioWaveChain(char *buf, unsigned bufSize);

int gpioWaveTxAt(void);

int gpioWaveTxBusy(void);

int gpioWaveTxStop(void);

int gpioWaveGetMicros(void);

int gpioWaveGetHighMicros(void);

int gpioWaveGetMaxMicros(void);

int gpioWaveGetPulses(void);

int gpioWaveGetHighPulses(void);

int gpioWaveGetMaxPulses(void);

int gpioWaveGetCbs(void);

int gpioWaveGetHighCbs(void);

int gpioWaveGetMaxCbs(void);
]]

ffi.cdef[[
int gpioSerialReadOpen(unsigned user_gpio, unsigned baud, unsigned data_bits);

int gpioSerialReadInvert(unsigned user_gpio, unsigned invert);

int gpioSerialRead(unsigned user_gpio, void *buf, size_t bufSize);

int gpioSerialReadClose(unsigned user_gpio);
]]

ffi.cdef[[
int i2cOpen(unsigned i2cBus, unsigned i2cAddr, unsigned i2cFlags);

int i2cClose(unsigned handle);

int i2cWriteQuick(unsigned handle, unsigned bit);

int i2cWriteByte(unsigned handle, unsigned bVal);

int i2cReadByte(unsigned handle);

int i2cWriteByteData(unsigned handle, unsigned i2cReg, unsigned bVal);

int i2cWriteWordData(unsigned handle, unsigned i2cReg, unsigned wVal);

int i2cReadByteData(unsigned handle, unsigned i2cReg);

int i2cReadWordData(unsigned handle, unsigned i2cReg);

int i2cProcessCall(unsigned handle, unsigned i2cReg, unsigned wVal);

int i2cWriteBlockData(
unsigned handle, unsigned i2cReg, char *buf, unsigned count);

int i2cReadBlockData(unsigned handle, unsigned i2cReg, char *buf);

int i2cBlockProcessCall(
unsigned handle, unsigned i2cReg, char *buf, unsigned count);

int i2cReadI2CBlockData(
unsigned handle, unsigned i2cReg, char *buf, unsigned count);

int i2cWriteI2CBlockData(
unsigned handle, unsigned i2cReg, char *buf, unsigned count);

int i2cReadDevice(unsigned handle, char *buf, unsigned count);

int i2cWriteDevice(unsigned handle, char *buf, unsigned count);

void i2cSwitchCombined(int setting);

int i2cSegments(unsigned handle, pi_i2c_msg_t *segs, unsigned numSegs);

int i2cZip(
   unsigned handle,
   char    *inBuf,
   unsigned inLen,
   char    *outBuf,
   unsigned outLen);
]]

ffi.cdef[[
int bbI2COpen(unsigned SDA, unsigned SCL, unsigned baud);

int bbI2CClose(unsigned SDA);

int bbI2CZip(
   unsigned SDA,
   char    *inBuf,
   unsigned inLen,
   char    *outBuf,
   unsigned outLen);
]]

ffi.cdef[[
int spiOpen(unsigned spiChan, unsigned baud, unsigned spiFlags);

int spiClose(unsigned handle);

int spiRead(unsigned handle, char *buf, unsigned count);

int spiWrite(unsigned handle, char *buf, unsigned count);

int spiXfer(unsigned handle, char *txBuf, char *rxBuf, unsigned count);
]]

ffi.cdef[[
int serOpen(char *sertty, unsigned baud, unsigned serFlags);

int serClose(unsigned handle);

int serWriteByte(unsigned handle, unsigned bVal);

int serReadByte(unsigned handle);

int serWrite(unsigned handle, char *buf, unsigned count);

int serRead(unsigned handle, char *buf, unsigned count);

int serDataAvailable(unsigned handle);
]]

ffi.cdef[[
int gpioTrigger(unsigned user_gpio, unsigned pulseLen, unsigned level);

int gpioSetWatchdog(unsigned user_gpio, unsigned timeout);

int gpioNoiseFilter(unsigned user_gpio, unsigned steady, unsigned active);

int gpioGlitchFilter(unsigned user_gpio, unsigned steady);

int gpioSetGetSamplesFunc(gpioGetSamplesFunc_t f, uint32_t bits);

int gpioSetGetSamplesFuncEx(
   gpioGetSamplesFuncEx_t f, uint32_t bits, void *userdata);

int gpioSetTimerFunc(unsigned timer, unsigned millis, gpioTimerFunc_t f);

int gpioSetTimerFuncEx(
   unsigned timer, unsigned millis, gpioTimerFuncEx_t f, void *userdata);
]]

--pthread_t *gpioStartThread(gpioThreadFunc_t f, void *userdata);
--void gpioStopThread(pthread_t *pth);

ffi.cdef[[
int gpioStoreScript(char *script);

int gpioRunScript(unsigned script_id, unsigned numPar, uint32_t *param);


int gpioScriptStatus(unsigned script_id, uint32_t *param);

int gpioStopScript(unsigned script_id);


int gpioDeleteScript(unsigned script_id);

int gpioSetSignalFunc(unsigned signum, gpioSignalFunc_t f);

int gpioSetSignalFuncEx(
   unsigned signum, gpioSignalFuncEx_t f, void *userdata);

uint32_t gpioRead_Bits_0_31(void);

uint32_t gpioRead_Bits_32_53(void);

int gpioWrite_Bits_0_31_Clear(uint32_t bits);

int gpioWrite_Bits_32_53_Clear(uint32_t bits);

int gpioWrite_Bits_0_31_Set(uint32_t bits);

int gpioWrite_Bits_32_53_Set(uint32_t bits);

int gpioHardwareClock(unsigned gpio, unsigned clkfreq);

int gpioHardwarePWM(unsigned gpio, unsigned PWMfreq, unsigned PWMduty);

int gpioTime(unsigned timetype, int *seconds, int *micros);

int gpioSleep(unsigned timetype, int seconds, int micros);

uint32_t gpioDelay(uint32_t micros);

uint32_t gpioTick(void);

unsigned gpioHardwareRevision(void);

unsigned gpioVersion(void);

int gpioGetPad(unsigned pad);

int gpioSetPad(unsigned pad, unsigned padStrength);
]]


ffi.cdef[[
int shell(char *scriptName, char *scriptString);

int fileOpen(char *file, unsigned mode);

int fileClose(unsigned handle);

int fileWrite(unsigned handle, char *buf, unsigned count);

int fileRead(unsigned handle, char *buf, unsigned count);

int fileSeek(unsigned handle, int32_t seekOffset, int seekFrom);

int fileList(char *fpat,  char *buf, unsigned count);
]]




ffi.cdef[[
int gpioCfgBufferSize(unsigned cfgMillis);

int gpioCfgClock(
   unsigned cfgMicros, unsigned cfgPeripheral, unsigned cfgSource);

int gpioCfgDMAchannel(unsigned DMAchannel); /* DEPRECATED */

int gpioCfgDMAchannels(
   unsigned primaryChannel, unsigned secondaryChannel);

int gpioCfgPermissions(uint64_t updateMask);

int gpioCfgSocketPort(unsigned port);

int gpioCfgInterfaces(unsigned ifFlags);

int gpioCfgMemAlloc(unsigned memAllocMode);

int gpioCfgInternals(unsigned cfgWhat, unsigned cfgVal);

uint32_t gpioCfgGetInternals(void);

int gpioCfgSetInternals(uint32_t cfgVal);

int gpioCustom1(unsigned arg1, unsigned arg2, char *argx, unsigned argc);

int gpioCustom2(unsigned arg1, char *argx, unsigned argc,
                char *retBuf, unsigned retMax);
]]


ffi.cdef[[
int rawWaveAddSPI(
   rawSPI_t *spi,
   unsigned offset,
   unsigned spiSS,
   char *buf,
   unsigned spiTxBits,
   unsigned spiBitFirst,
   unsigned spiBitLast,
   unsigned spiBits);

int rawWaveAddGeneric(unsigned numPulses, rawWave_t *pulses);

unsigned rawWaveCB(void);

rawCbs_t *rawWaveCBAdr(int cbNum);

uint32_t rawWaveGetOut(int pos);

void rawWaveSetOut(int pos, uint32_t lVal);

uint32_t rawWaveGetIn(int pos);

void rawWaveSetIn(int pos, uint32_t lVal);

rawWaveInfo_t rawWaveInfo(int wave_id);

int getBitInBytes(int bitPos, char *buf, int numBits);

void putBitInBytes(int bitPos, char *buf, int bit);

double time_time(void);

void time_sleep(double seconds);

void rawDumpWave(void);

void rawDumpScript(unsigned script_id);
]]


ffi.cdef[[
/*DEF_S Socket Command Codes*/
static const int PI_CMD_MODES  =0;
static const int PI_CMD_MODEG  =1;
static const int PI_CMD_PUD    =2;
static const int PI_CMD_READ   =3;
static const int PI_CMD_WRITE  =4;
static const int PI_CMD_PWM    =5;
static const int PI_CMD_PRS    =6;
static const int PI_CMD_PFS    =7;
static const int PI_CMD_SERVO  =8;
static const int PI_CMD_WDOG   =9;
static const int PI_CMD_BR1   =10;
static const int PI_CMD_BR2   =11;
static const int PI_CMD_BC1   =12;
static const int PI_CMD_BC2   =13;
static const int PI_CMD_BS1   =14;
static const int PI_CMD_BS2   =15;
static const int PI_CMD_TICK  =16;
static const int PI_CMD_HWVER =17;
static const int PI_CMD_NO    =18;
static const int PI_CMD_NB    =19;
static const int PI_CMD_NP    =20;
static const int PI_CMD_NC    =21;
static const int PI_CMD_PRG   =22;
static const int PI_CMD_PFG   =23;
static const int PI_CMD_PRRG  =24;
static const int PI_CMD_HELP  =25;
static const int PI_CMD_PIGPV =26;
static const int PI_CMD_WVCLR =27;
static const int PI_CMD_WVAG  =28;
static const int PI_CMD_WVAS  =29;
static const int PI_CMD_WVGO  =30;
static const int PI_CMD_WVGOR =31;
static const int PI_CMD_WVBSY =32;
static const int PI_CMD_WVHLT =33;
static const int PI_CMD_WVSM  =34;
static const int PI_CMD_WVSP  =35;
static const int PI_CMD_WVSC  =36;
static const int PI_CMD_TRIG  =37;
static const int PI_CMD_PROC  =38;
static const int PI_CMD_PROCD =39;
static const int PI_CMD_PROCR =40;
static const int PI_CMD_PROCS =41;
static const int PI_CMD_SLRO  =42;
static const int PI_CMD_SLR   =43;
static const int PI_CMD_SLRC  =44;
static const int PI_CMD_PROCP =45;
static const int PI_CMD_MICS  =46;
static const int PI_CMD_MILS  =47;
static const int PI_CMD_PARSE =48;
static const int PI_CMD_WVCRE =49;
static const int PI_CMD_WVDEL =50;
static const int PI_CMD_WVTX  =51;
static const int PI_CMD_WVTXR =52;
static const int PI_CMD_WVNEW =53;

static const int PI_CMD_I2CO  =54;
static const int PI_CMD_I2CC  =55;
static const int PI_CMD_I2CRD =56;
static const int PI_CMD_I2CWD =57;
static const int PI_CMD_I2CWQ =58;
static const int PI_CMD_I2CRS =59;
static const int PI_CMD_I2CWS =60;
static const int PI_CMD_I2CRB =61;
static const int PI_CMD_I2CWB =62;
static const int PI_CMD_I2CRW =63;
static const int PI_CMD_I2CWW =64;
static const int PI_CMD_I2CRK =65;
static const int PI_CMD_I2CWK =66;
static const int PI_CMD_I2CRI =67;
static const int PI_CMD_I2CWI =68;
static const int PI_CMD_I2CPC =69;
static const int PI_CMD_I2CPK =70;

static const int PI_CMD_SPIO  =71;
static const int PI_CMD_SPIC  =72;
static const int PI_CMD_SPIR  =73;
static const int PI_CMD_SPIW  =74;
static const int PI_CMD_SPIX  =75;

static const int PI_CMD_SERO  =76;
static const int PI_CMD_SERC  =77;
static const int PI_CMD_SERRB =78;
static const int PI_CMD_SERWB =79;
static const int PI_CMD_SERR  =80;
static const int PI_CMD_SERW  =81;
static const int PI_CMD_SERDA =82;

static const int PI_CMD_GDC   =83;
static const int PI_CMD_GPW   =84;

static const int PI_CMD_HC    =85;
static const int PI_CMD_HP    =86;

static const int PI_CMD_CF1   =87;
static const int PI_CMD_CF2   =88;

static const int PI_CMD_BI2CC =89;
static const int PI_CMD_BI2CO =90;
static const int PI_CMD_BI2CZ =91;

static const int PI_CMD_I2CZ  =92;

static const int PI_CMD_WVCHA =93;

static const int PI_CMD_SLRI  =94;

static const int PI_CMD_CGI   =95;
static const int PI_CMD_CSI   =96;

static const int PI_CMD_FG    =97;
static const int PI_CMD_FN    =98;

static const int PI_CMD_NOIB  =99;

static const int PI_CMD_WVTXM =100;
static const int PI_CMD_WVTAT =101;

static const int PI_CMD_PADS  =102;
static const int PI_CMD_PADG  =103;

static const int PI_CMD_FO    =104;
static const int PI_CMD_FC    =105;
static const int PI_CMD_FR    =106;
static const int PI_CMD_FW    =107;
static const int PI_CMD_FS    =108;
static const int PI_CMD_FL    =109;

static const int PI_CMD_SHELL =110;
]]


ffi.cdef[[
/* pseudo commands */

static const int PI_CMD_SCRIPT =800;

static const int PI_CMD_ADD   =800;
static const int PI_CMD_AND   =801;
static const int PI_CMD_CALL  =802;
static const int PI_CMD_CMDR  =803;
static const int PI_CMD_CMDW  =804;
static const int PI_CMD_CMP   =805;
static const int PI_CMD_DCR   =806;
static const int PI_CMD_DCRA  =807;
static const int PI_CMD_DIV   =808;
static const int PI_CMD_HALT  =809;
static const int PI_CMD_INR   =810;
static const int PI_CMD_INRA  =811;
static const int PI_CMD_JM    =812;
static const int PI_CMD_JMP   =813;
static const int PI_CMD_JNZ   =814;
static const int PI_CMD_JP    =815;
static const int PI_CMD_JZ    =816;
static const int PI_CMD_TAG   =817;
static const int PI_CMD_LD    =818;
static const int PI_CMD_LDA   =819;
static const int PI_CMD_LDAB  =820;
static const int PI_CMD_MLT   =821;
static const int PI_CMD_MOD   =822;
static const int PI_CMD_NOP   =823;
static const int PI_CMD_OR    =824;
static const int PI_CMD_POP   =825;
static const int PI_CMD_POPA  =826;
static const int PI_CMD_PUSH  =827;
static const int PI_CMD_PUSHA =828;
static const int PI_CMD_RET   =829;
static const int PI_CMD_RL    =830;
static const int PI_CMD_RLA   =831;
static const int PI_CMD_RR    =832;
static const int PI_CMD_RRA   =833;
static const int PI_CMD_STA   =834;
static const int PI_CMD_STAB  =835;
static const int PI_CMD_SUB   =836;
static const int PI_CMD_SYS   =837;
static const int PI_CMD_WAIT  =838;
static const int PI_CMD_X     =839;
static const int PI_CMD_XA    =840;
static const int PI_CMD_XOR   =841;
]]

ffi.cdef[[
/*DEF_S Error Codes*/

static const int PI_INIT_FAILED      = -1; // gpioInitialise failed
static const int PI_BAD_USER_GPIO    = -2; // GPIO not 0-31
static const int PI_BAD_GPIO         = -3; // GPIO not 0-53
static const int PI_BAD_MODE         = -4; // mode not 0-7
static const int PI_BAD_LEVEL        = -5; // level not 0-1
static const int PI_BAD_PUD          = -6; // pud not 0-2
static const int PI_BAD_PULSEWIDTH   = -7; // pulsewidth not 0 or 500-2500
static const int PI_BAD_DUTYCYCLE    = -8; // dutycycle outside set range
static const int PI_BAD_TIMER        = -9; // timer not 0-9
static const int PI_BAD_MS           =-10; // ms not 10-60000
static const int PI_BAD_TIMETYPE     =-11; // timetype not 0-1
static const int PI_BAD_SECONDS      =-12; // seconds < 0
static const int PI_BAD_MICROS       =-13; // micros not 0-999999
static const int PI_TIMER_FAILED     =-14; // gpioSetTimerFunc failed
static const int PI_BAD_WDOG_TIMEOUT =-15; // timeout not 0-60000
static const int PI_NO_ALERT_FUNC    =-16; // DEPRECATED
static const int PI_BAD_CLK_PERIPH   =-17; // clock peripheral not 0-1
static const int PI_BAD_CLK_SOURCE   =-18; // DEPRECATED
static const int PI_BAD_CLK_MICROS   =-19; // clock micros not 1, 2, 4, 5, 8, or 10
static const int PI_BAD_BUF_MILLIS   =-20; // buf millis not 100-10000
static const int PI_BAD_DUTYRANGE    =-21; // dutycycle range not 25-40000
static const int PI_BAD_DUTY_RANGE   =-21; // DEPRECATED (use PI_BAD_DUTYRANGE)
static const int PI_BAD_SIGNUM       =-22; // signum not 0-63
static const int PI_BAD_PATHNAME     =-23; // can't open pathname
static const int PI_NO_HANDLE        =-24; // no handle available
static const int PI_BAD_HANDLE       =-25; // unknown handle
static const int PI_BAD_IF_FLAGS     =-26; // ifFlags > 3
static const int PI_BAD_CHANNEL      =-27; // DMA channel not 0-14
static const int PI_BAD_PRIM_CHANNEL =-27; // DMA primary channel not 0-14
static const int PI_BAD_SOCKET_PORT  =-28; // socket port not 1024-32000
static const int PI_BAD_FIFO_COMMAND =-29; // unrecognized fifo command
static const int PI_BAD_SECO_CHANNEL =-30; // DMA secondary channel not 0-6
static const int PI_NOT_INITIALISED  =-31; // function called before gpioInitialise
static const int PI_INITIALISED      =-32; // function called after gpioInitialise
static const int PI_BAD_WAVE_MODE    =-33; // waveform mode not 0-3
static const int PI_BAD_CFG_INTERNAL =-34; // bad parameter in gpioCfgInternals call
static const int PI_BAD_WAVE_BAUD    =-35; // baud rate not 50-250K(RX)/50-1M(TX)
static const int PI_TOO_MANY_PULSES  =-36; // waveform has too many pulses
static const int PI_TOO_MANY_CHARS   =-37; // waveform has too many chars
static const int PI_NOT_SERIAL_GPIO  =-38; // no bit bang serial read on GPIO
static const int PI_BAD_SERIAL_STRUC =-39; // bad (null) serial structure parameter
static const int PI_BAD_SERIAL_BUF   =-40; // bad (null) serial buf parameter
static const int PI_NOT_PERMITTED    =-41; // GPIO operation not permitted
static const int PI_SOME_PERMITTED   =-42; // one or more GPIO not permitted
static const int PI_BAD_WVSC_COMMND  =-43; // bad WVSC subcommand
static const int PI_BAD_WVSM_COMMND  =-44; // bad WVSM subcommand
static const int PI_BAD_WVSP_COMMND  =-45; // bad WVSP subcommand
static const int PI_BAD_PULSELEN     =-46; // trigger pulse length not 1-100
static const int PI_BAD_SCRIPT       =-47; // invalid script
static const int PI_BAD_SCRIPT_ID    =-48; // unknown script id
static const int PI_BAD_SER_OFFSET   =-49; // add serial data offset > 30 minutes
static const int PI_GPIO_IN_USE      =-50; // GPIO already in use
static const int PI_BAD_SERIAL_COUNT =-51; // must read at least a byte at a time
static const int PI_BAD_PARAM_NUM    =-52; // script parameter id not 0-9
static const int PI_DUP_TAG          =-53; // script has duplicate tag
static const int PI_TOO_MANY_TAGS    =-54; // script has too many tags
static const int PI_BAD_SCRIPT_CMD   =-55; // illegal script command
static const int PI_BAD_VAR_NUM      =-56; // script variable id not 0-149
static const int PI_NO_SCRIPT_ROOM   =-57; // no more room for scripts
static const int PI_NO_MEMORY        =-58; // can't allocate temporary memory
static const int PI_SOCK_READ_FAILED =-59; // socket read failed
static const int PI_SOCK_WRIT_FAILED =-60; // socket write failed
static const int PI_TOO_MANY_PARAM   =-61; // too many script parameters (> 10)
static const int PI_NOT_HALTED       =-62; // DEPRECATED
static const int PI_SCRIPT_NOT_READY =-62; // script initialising
static const int PI_BAD_TAG          =-63; // script has unresolved tag
static const int PI_BAD_MICS_DELAY   =-64; // bad MICS delay (too large)
static const int PI_BAD_MILS_DELAY   =-65; // bad MILS delay (too large)
static const int PI_BAD_WAVE_ID      =-66; // non existent wave id
static const int PI_TOO_MANY_CBS     =-67; // No more CBs for waveform
static const int PI_TOO_MANY_OOL     =-68; // No more OOL for waveform
static const int PI_EMPTY_WAVEFORM   =-69; // attempt to create an empty waveform
static const int PI_NO_WAVEFORM_ID   =-70; // no more waveforms
static const int PI_I2C_OPEN_FAILED  =-71; // can't open I2C device
static const int PI_SER_OPEN_FAILED  =-72; // can't open serial device
static const int PI_SPI_OPEN_FAILED  =-73; // can't open SPI device
static const int PI_BAD_I2C_BUS      =-74; // bad I2C bus
static const int PI_BAD_I2C_ADDR     =-75; // bad I2C address
static const int PI_BAD_SPI_CHANNEL  =-76; // bad SPI channel
static const int PI_BAD_FLAGS        =-77; // bad i2c/spi/ser open flags
static const int PI_BAD_SPI_SPEED    =-78; // bad SPI speed
static const int PI_BAD_SER_DEVICE   =-79; // bad serial device name
static const int PI_BAD_SER_SPEED    =-80; // bad serial baud rate
static const int PI_BAD_PARAM        =-81; // bad i2c/spi/ser parameter
static const int PI_I2C_WRITE_FAILED =-82; // i2c write failed
static const int PI_I2C_READ_FAILED  =-83; // i2c read failed
static const int PI_BAD_SPI_COUNT    =-84; // bad SPI count
static const int PI_SER_WRITE_FAILED =-85; // ser write failed
static const int PI_SER_READ_FAILED  =-86; // ser read failed
static const int PI_SER_READ_NO_DATA =-87; // ser read no data available
static const int PI_UNKNOWN_COMMAND  =-88; // unknown command
static const int PI_SPI_XFER_FAILED  =-89; // spi xfer/read/write failed
static const int PI_BAD_POINTER      =-90; // bad (NULL) pointer
static const int PI_NO_AUX_SPI       =-91; // no auxiliary SPI on Pi A or B
static const int PI_NOT_PWM_GPIO     =-92; // GPIO is not in use for PWM
static const int PI_NOT_SERVO_GPIO   =-93; // GPIO is not in use for servo pulses
static const int PI_NOT_HCLK_GPIO    =-94; // GPIO has no hardware clock
static const int PI_NOT_HPWM_GPIO    =-95; // GPIO has no hardware PWM
static const int PI_BAD_HPWM_FREQ    =-96; // hardware PWM frequency not 1-125M
static const int PI_BAD_HPWM_DUTY    =-97; // hardware PWM dutycycle not 0-1M
static const int PI_BAD_HCLK_FREQ    =-98; // hardware clock frequency not 4689-250M
static const int PI_BAD_HCLK_PASS    =-99; // need password to use hardware clock 1
static const int PI_HPWM_ILLEGAL    =-100; // illegal, PWM in use for main clock
static const int PI_BAD_DATABITS    =-101; // serial data bits not 1-32
static const int PI_BAD_STOPBITS    =-102; // serial (half) stop bits not 2-8
static const int PI_MSG_TOOBIG      =-103; // socket/pipe message too big
static const int PI_BAD_MALLOC_MODE =-104; // bad memory allocation mode
static const int PI_TOO_MANY_SEGS   =-105; // too many I2C transaction segments
static const int PI_BAD_I2C_SEG     =-106; // an I2C transaction segment failed
static const int PI_BAD_SMBUS_CMD   =-107; // SMBus command not supported by driver
static const int PI_NOT_I2C_GPIO    =-108; // no bit bang I2C in progress on GPIO
static const int PI_BAD_I2C_WLEN    =-109; // bad I2C write length
static const int PI_BAD_I2C_RLEN    =-110; // bad I2C read length
static const int PI_BAD_I2C_CMD     =-111; // bad I2C command
static const int PI_BAD_I2C_BAUD    =-112; // bad I2C baud rate, not 50-500k
static const int PI_CHAIN_LOOP_CNT  =-113; // bad chain loop count
static const int PI_BAD_CHAIN_LOOP  =-114; // empty chain loop
static const int PI_CHAIN_COUNTER   =-115; // too many chain counters
static const int PI_BAD_CHAIN_CMD   =-116; // bad chain command
static const int PI_BAD_CHAIN_DELAY =-117; // bad chain delay micros
static const int PI_CHAIN_NESTING   =-118; // chain counters nested too deeply
static const int PI_CHAIN_TOO_BIG   =-119; // chain is too long
static const int PI_DEPRECATED      =-120; // deprecated function removed
static const int PI_BAD_SER_INVERT  =-121; // bit bang serial invert not 0 or 1
static const int PI_BAD_EDGE        =-122; // bad ISR edge value, not 0-2
static const int PI_BAD_ISR_INIT    =-123; // bad ISR initialisation
static const int PI_BAD_FOREVER     =-124; // loop forever must be last command
static const int PI_BAD_FILTER      =-125; // bad filter parameter
static const int PI_BAD_PAD         =-126; // bad pad number
static const int PI_BAD_STRENGTH    =-127; // bad pad drive strength
static const int PI_FIL_OPEN_FAILED =-128; // file open failed
static const int PI_BAD_FILE_MODE   =-129; // bad file mode
static const int PI_BAD_FILE_FLAG   =-130; // bad file flag
static const int PI_BAD_FILE_READ   =-131; // bad file read
static const int PI_BAD_FILE_WRITE  =-132; // bad file write
static const int PI_FILE_NOT_ROPEN  =-133; // file not open for read
static const int PI_FILE_NOT_WOPEN  =-134; // file not open for write
static const int PI_BAD_FILE_SEEK   =-135; // bad file seek
static const int PI_NO_FILE_MATCH   =-136; // no files match pattern
static const int PI_NO_FILE_ACCESS  =-137; // no permission to access file
static const int PI_FILE_IS_A_DIR   =-138; // file is a directory
static const int PI_BAD_SHELL_STATUS =-139; // bad shell return status
static const int PI_BAD_SCRIPT_NAME =-140; // bad script name

static const int PI_PIGIF_ERR_0    =-2000;
static const int PI_PIGIF_ERR_99   =-2099;

static const int PI_CUSTOM_ERR_0   =-3000;
static const int PI_CUSTOM_ERR_999 =-3999;

/*DEF_E*/

/*DEF_S Defaults*/

static const int PI_DEFAULT_BUFFER_MILLIS           = 120;
static const int PI_DEFAULT_CLK_MICROS              = 5;
static const int PI_DEFAULT_CLK_PERIPHERAL          = PI_CLOCK_PCM;
static const int PI_DEFAULT_IF_FLAGS                = 0;
static const int PI_DEFAULT_DMA_CHANNEL             = 14;
static const int PI_DEFAULT_DMA_PRIMARY_CHANNEL     = 14;
static const int PI_DEFAULT_DMA_SECONDARY_CHANNEL   = 6;
static const int PI_DEFAULT_SOCKET_PORT             = 8888;

static const int PI_DEFAULT_UPDATE_MASK_UNKNOWN     = 0xFFFFFFFF;
static const int PI_DEFAULT_UPDATE_MASK_B1          = 0x03E7CF93;
static const int PI_DEFAULT_UPDATE_MASK_A_B2        = 0xFBC7CF9C;
]]



ffi.cdef[[
static const int PI_DEFAULT_MEM_ALLOC_MODE          = PI_MEM_ALLOC_AUTO;

static const int PI_DEFAULT_CFG_INTERNALS           = 0;
]]


--return ffi.load("pigpio")
