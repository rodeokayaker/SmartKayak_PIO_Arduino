#pragma once
#include <Arduino_GFX_Library.h>

class CustomILI9341 : public Arduino_TFT
{
public:
  CustomILI9341(Arduino_DataBus *bus, int8_t rst = GFX_NOT_DEFINED, uint8_t r = 0, bool ips = false)
    : Arduino_TFT(bus, rst, r, ips, ILI9341_TFTHEIGHT, ILI9341_TFTWIDTH, 0, 0, 0, 0) {}

  bool begin(int32_t speed = GFX_NOT_DEFINED) override
  {
    _override_datamode = SPI_MODE0; // always use SPI_MODE0

    return Arduino_TFT::begin(speed);
  }

  void setRotation(uint8_t r) override
  {
    Arduino_TFT::setRotation(r);
    switch (_rotation)
    {
    case 1:
      r = (ILI9341_MADCTL_MV | ILI9341_MADCTL_RGB);
      break;
    case 2:
      r = (ILI9341_MADCTL_MY | ILI9341_MADCTL_RGB);
      break;
    case 3:
      r = (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_RGB);
      break;
    default: // case 0:
      r = (ILI9341_MADCTL_MX | ILI9341_MADCTL_RGB);
      break;
    }
    _bus->beginWrite();
    _bus->writeC8D8(ILI9341_MADCTL, r);
    _bus->endWrite();
  };

  void writeAddrWindow(int16_t x, int16_t y, uint16_t w, uint16_t h) override
  {
    if ((x != _currentX) || (w != _currentW))
    {
      _currentX = x;
      _currentW = w;
      x += _xStart;
      _bus->writeC8D16D16(ILI9341_CASET, x, x + w - 1);
    }
  
    if ((y != _currentY) || (h != _currentH))
    {
      _currentY = y;
      _currentH = h;
      y += _yStart;
      _bus->writeC8D16D16(ILI9341_PASET, y, y + h - 1);
    }
  
    _bus->writeCommand(ILI9341_RAMWR); // write to RAM
  };

  void invertDisplay(bool i) override
  {
    _bus->sendCommand((_ips ^ i) ? ILI9341_INVON : ILI9341_INVOFF);
  };
  void displayOn() override
  {
    _bus->sendCommand(ILI9341_SLPOUT);
    delay(ILI9341_SLPOUT_DELAY);
  };
  void displayOff() override
  {
    _bus->sendCommand(ILI9341_SLPIN);
    delay(ILI9341_SLPIN_DELAY);
  };

protected:
  void tftInit() override
  {
    if (_rst != GFX_NOT_DEFINED)
    {
      pinMode(_rst, OUTPUT);
      digitalWrite(_rst, HIGH);
      delay(100);
      digitalWrite(_rst, LOW);
      delay(ILI9341_RST_DELAY);
      digitalWrite(_rst, HIGH);
      delay(ILI9341_RST_DELAY);
    }
    else
    {
      // Software Rest
      _bus->sendCommand(ILI9341_SWRESET);
      delay(ILI9341_RST_DELAY);
    }
  
    // Исправленная версия массива инициализации (исправлена строка с 0xCB командой - должно быть 5 байт, а не 3)
    static const uint8_t fixed_init_ops[] = {
      BEGIN_WRITE,
      WRITE_COMMAND_8, 0xEF,
      WRITE_BYTES, 3, 0x03, 0x80, 0x02,
      WRITE_COMMAND_8, 0xCF,
      WRITE_BYTES, 3, 0x00, 0xC1, 0x30,
      WRITE_COMMAND_8, 0xED,
      WRITE_BYTES, 4, 0x64, 0x03, 0x12, 0x81,
      WRITE_COMMAND_8, 0xE8,
      WRITE_BYTES, 3, 0x85, 0x00, 0x78,
      WRITE_COMMAND_8, 0xCB,
      WRITE_BYTES, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
      WRITE_C8_D8, 0xF7, 0x20,
      WRITE_C8_D16, 0xEA, 0x00, 0x00,
      WRITE_C8_D8, ILI9341_PWCTR1, 0x23,
      WRITE_C8_D8, ILI9341_PWCTR2, 0x10,
      WRITE_C8_D16, ILI9341_VMCTR1, 0x3e, 0x28,
      WRITE_C8_D8, ILI9341_VMCTR2, 0x86,
      WRITE_C8_D8, ILI9341_VSCRSADD, 0x00,
      WRITE_C8_D8, ILI9341_PIXFMT, 0x55,
      WRITE_C8_D16, ILI9341_FRMCTR1, 0x00, 0x18,
      WRITE_COMMAND_8, ILI9341_DFUNCTR,
      WRITE_BYTES, 3, 0x08, 0x82, 0x27,
      WRITE_C8_D8, 0xF2, 0x00,
      WRITE_C8_D8, ILI9341_GAMMASET, 0x01,
      WRITE_COMMAND_8, ILI9341_GMCTRP1,
      WRITE_BYTES, 15,
      0x0F, 0x31, 0x2B, 0x0C,
      0x0E, 0x08, 0x4E, 0xF1,
      0x37, 0x07, 0x10, 0x03,
      0x0E, 0x09, 0x00,
      WRITE_COMMAND_8, ILI9341_GMCTRN1,
      WRITE_BYTES, 15,
      0x00, 0x0E, 0x14, 0x03,
      0x11, 0x07, 0x31, 0xC1,
      0x48, 0x08, 0x0F, 0x0C,
      0x31, 0x36, 0x0F,
      WRITE_COMMAND_8, ILI9341_SLPOUT,
      WRITE_COMMAND_8, ILI9341_DISPON,
      END_WRITE
    };
  
    _bus->batchOperation(fixed_init_ops, sizeof(fixed_init_ops));
  
    invertDisplay(false);
  };

private:
};
