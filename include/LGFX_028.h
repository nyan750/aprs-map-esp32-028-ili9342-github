#pragma once
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// LGFX config for the NEW 2432S028 unit (arrived 2026-11-15),
// CONFIRMED by diag v4/v5 (2026-11-15): this unit's panel is ILI9342
// (320x240 native landscape GRAM, as in the M5Stack Basic), NOT the
// ILI9341 of the old unit. Inversion ON, rotation 2, and pushImage
// shows the same (g,b,r) colour rotation as the old unit, so the
// standard pre-rotated palettes apply. Bus/light pins as the 028.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9342 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;

public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host    = HSPI_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 40000000;
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 14;
      cfg.pin_mosi = 13;
      cfg.pin_miso = 12;
      cfg.pin_dc   = 2;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs   = 15;
      cfg.pin_rst  = -1;
      cfg.pin_busy = -1;
      cfg.panel_width   = 320;
      cfg.panel_height  = 240;
      cfg.memory_width  = 320;
      cfg.memory_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.readable  = false;   // SDO reads back 0 on this unit (diag v5)
      cfg.invert    = true;    // diag v5: W rendered black at v4 -> inversion ON (as on the M5 ILI9342C)
      cfg.rgb_order = false;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    {
      auto cfg = _light.config();
      cfg.pin_bl      = 21;
      cfg.invert      = false;
      cfg.freq        = 44100;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};
