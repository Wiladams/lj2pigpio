local bit = require("bit")
local lshift, band = bit.lshift, bit.band

local exports = {
    Lib = require("libpigpio");
    
  PIGPIO_VERSION = 55;


  PI_INPFIFO ="/dev/pigpio";
  PI_OUTFIFO ="/dev/pigout";
  PI_ERRFIFO ="/dev/pigerr";

  PI_ENVPORT ="PIGPIO_PORT";
  PI_ENVADDR ="PIGPIO_ADDR";

  PI_LOCKFILE ="/var/run/pigpio.pid";

  PI_I2C_COMBINED ="/sys/module/i2c_bcm2708/parameters/combined";

    PI_DEFAULT_SOCKET_PORT_STR        = "8888";
    PI_DEFAULT_SOCKET_ADDR_STR        = "127.0.0.1";
    
    PI_DEFAULT_UPDATE_MASK_APLUS_BPLUS = 0x0080480FFFFFFCLL;
    PI_DEFAULT_UPDATE_MASK_ZERO        = 0x0080000FFFFFFCLL;
    PI_DEFAULT_UPDATE_MASK_PI2B        = 0x0080480FFFFFFCLL;
    PI_DEFAULT_UPDATE_MASK_PI3B        = 0x0000000FFFFFFCLL;
    PI_DEFAULT_UPDATE_MASK_COMPUTE     = 0x00FFFFFFFFFFFFLL;

}

exports.WAVE_FLAG_READ  = 1;
exports.WAVE_FLAG_TICK  = 2;


-- SPI

local function PI_SPI_FLAGS_BITLEN(x) return lshift(band(x,63),16) end
local function PI_SPI_FLAGS_RX_LSB(x)  return lshift(band(x,1),15) end
local function PI_SPI_FLAGS_TX_LSB(x)  return lshift(band(x,1),14) end
local function PI_SPI_FLAGS_3WREN(x)  return lshift(band(x,15),10) end
local function PI_SPI_FLAGS_3WIRE(x)   return lshift(band(x,1),9) end
local function PI_SPI_FLAGS_AUX_SPI(x) return lshift(band(x,1),8) end
local function PI_SPI_FLAGS_RESVD(x)   return lshift(band(x,7),5) end
local function PI_SPI_FLAGS_CSPOLS(x)  return lshift(band(x,7),2) end
local function PI_SPI_FLAGS_MODE(x)    return band(x,3) end

local function PI_NTFY_FLAGS_BIT(x) return band(lshift(x,0),31) end


return exports
