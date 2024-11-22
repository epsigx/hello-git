template <class T>
struct gpu
{ // most of the implementation of gpu is referenced from the source code of vba-m
  struct plat
  {
    struct cell
    {  
      uintptr_t tile_id[2]; // [0]:*32^h_flip[1]:*64^h_flip
      uint32_t *palette;    // 4bit palette bank.

      uintptr_t filp_x; // != 0   
    };
    cell cells[32 * 32];
  };

  struct sprite
  {
    struct cell
    {
      uint8_t *character;
      uint32_t *palette;
      uint32_t scale_mask;
      uint32_t _8bit_mask; // 0:4bit
      uint32_t win_mask;
      uint32_t mosaic_mask;
      uint32_t pos_x;   // (adjusted).
      uint32_t pos_y;   // (adjusted).
      uint32_t pri;     // without 1st mask, 2nd mask.
      uint32_t tile_sft;
      uint32_t size_x;  // 8/16/32/64 
      uint32_t size_y;  // 8/16/32/64
      uint32_t address; // base tile id address. 
      uint32_t shape2; // 

      union
      {
        struct
        {
          int32_t gst_x;
          int32_t gst_y; 
          int32_t org_x;
          int32_t org_y;

          int32_t ref_dx;
          int32_t ref_dmx;
          int32_t ref_dy;
          int32_t ref_dmy;

          int16_t *transform;
        };

        struct
        {
          uint32_t enable_mask;

          uint32_t vflip_mask; // size_y - 1 or 0
          uint32_t hflip_mask;
        };
      };
    };
    uint32_t pri;

    int16_t transforms[128];

    cell cells[128];
  };

  struct channel
  {
    uint32_t control;  // reg - BG?CNT

    uint32_t hofs; // reg - BG?HOFS
    uint32_t vofs; // reg - BG?VOFS

    uint16_t scale_params[4]; // reg- dx, dmx, dy, dmy...
    uint16_t scale_start[4]; // reg- dx, dmx, dy, dmy...
    uint32_t scale_params_rt[4]; // dx, dmx, dy, dmy...
    uint32_t scale_start_rt[2]; //  dx, dmx, dy, dmy...

    uint16_t win_h;
    uint16_t win_v;

    uint32_t mosaic_mask;

    uint32_t pri; // adjusted (pri, 1st, 2nd mask).
    uint32_t scroll_y; // (& 511 done)

    uint8_t *character;

    plat *nt_banks[4]; // nametable bank
    typename plat::cell *nt_maps[4]; 

    uint32_t iterates;      // 
    uint32_t *buffer_off_x; //  = & chan.buffer[8 - mini_x];
    uint32_t *buffer_end; // switch for nam,etable...
    uint32_t *buffer; // 8 for pos 0
  }; 

  gpu (void)
  {

  }

  uint32_t user_display_mask; 
  uint32_t mode_display_mask;  // & control ...

  channel channels[4];

  uint32_t bg_pal[256 + 256];
  uint32_t sp_pal[256 + 256];
  uint32_t sp_pri; // sp base pri lock.

  uint32_t mosaic; // reg-mosaic
  uint32_t mosaic_bg_x; // +1 done
  uint32_t mosaic_bg_y; // +1 done
  uint32_t mosaic_sp_x; // +1 done
  uint32_t mosaic_sp_y; // +1 done
  uint32_t vcount; // reg-vcount.
  uint32_t control;// reg-master control.
  uint32_t bldctl; // reg-bldcnt
  uint32_t bldalpha; 
  uint32_t bldy; 
  uint32_t state;  // reg-state
  uint32_t win_in_out[2]; // reg-win_in, win_out
  uint16_t win_h[2];
  uint16_t win_v[2];
  uint32_t win_dim_h[4]; // left, right, llef
  uint32_t win_dim_v[4];
  
  uint32_t irq_hbl_stat_mask;
  uint32_t irq_vbl_stat_mask;
  uint32_t irq_line_stat_mask;

  uint8_t memory[MEM_32K * 6]; // bg 32k chunk0, bg 32k chunk1, bg 32k chunk0 mirror, bg 32k chunk1 mirror, sp 32k, sp 32k mirror.
  uint8_t trace[MEM_32K * 3];

  uint8_t oam[MEM_1K];
  uint8_t pal[MEM_1K];
  uint8_t io[256];

  sprite sprite_;

  plat plats[32]; // 64k nametable map.


  // General Internal Memory
  //   00000000-00003FFF   BIOS - System ROM         (16 KBytes)
  //   00004000-01FFFFFF   Not used
  //   02000000-0203FFFF   WRAM - On-board Work RAM  (256 KBytes) 2 Wait
  //   02040000-02FFFFFF   Not used
  //   03000000-03007FFF   WRAM - On-chip Work RAM   (32 KBytes)
  //   03008000-03FFFFFF   Not used
  //   04000000-040003FE   I/O Registers
  //   04000400-04FFFFFF   Not used
  // Internal Display Memory
  //   05000000-050003FF   BG/OBJ Palette RAM        (1 Kbyte)
  //   05000400-05FFFFFF   Not used
  //   06000000-06017FFF   VRAM - Video RAM          (96 KBytes)
  //   06018000-06FFFFFF   Not used
  //   07000000-070003FF   OAM - OBJ Attributes      (1 Kbyte)
  //   07000400-07FFFFFF   Not used
  // External Memory (Game Pak)
  //   08000000-09FFFFFF   Game Pak ROM/FlashROM (max 32MB) - Wait State 0
  //   0A000000-0BFFFFFF   Game Pak ROM/FlashROM (max 32MB) - Wait State 1
  //   0C000000-0DFFFFFF   Game Pak ROM/FlashROM (max 32MB) - Wait State 2
  //   0E000000-0E00FFFF   Game Pak SRAM    (max 64 KBytes) - 8bit Bus width
  //   0E010000-0FFFFFFF   Not used
  // Unused Memory Area
  //   10000000-FFFFFFFF   Not used (upper 4bits of address bus unused)

  // 31 30 29 28 27 26  25  24
  // mp mp sp sp sp 1st 2nd obm
  //
  // channel 0 buffer
  // channel 1 buffer
  // channel 2 buffer
  // channel 3 buffer
  // sprite buffer
  // backdrop buffer 
  // window buffer 
  // mixer buffer 
  uint32_t *sp_buffer;       // 31 30 29 28 27 26  25  24
  uint32_t *win_buffer;      // d2~d0:win0, win1, obj
  uint32_t *graphics_buffer;
  uint32_t *graphics_pitch; // uint32_t pitch
  uint32_t backdrop;

  uint32_t *(gpu<T>::*sp_render) (void);
  uint32_t *(gpu<T>::*bg_renders[4])(channel &);
  void (gpu<T>::*composite) (void);

  // hash table  
  void (gpu<T>::*hash_composites[6 * 8])(void); // d2:sprite mask d1-d0:effect mode

  uint32_t *(gpu<T>::*hash_txt_renders[2])(channel &); 
  uint32_t *(gpu<T>::*hash_rot_renders[8])(channel &); 
  uint32_t *(gpu<T>::*hash_sp_renders[8])(void);
  void (gpu<T>::*hash_sp_wins[8])(void); 

  void (*sp_win) (void);

#ifdef VSP_VECTOR_PROCESS    //               fedcba9876543210fedcba9876543210 | fedcba9876543210fedcba9876543210
  uint32_t win_mask[8];      // channel 5x    cC00eeeeeEaaaaaAdD00bbbbbB000000 | 0000000dddddD0000eeeeeE000000Wws 
                             // channel 4x    cC00000000aaaaaAdD00bbbbbB000000 | 00000000000000000000000000000Wws  
                             // channel 3x    cC00000000aaaaaA0000bbbbbB000000 | 00000000000000000000000000000Wws
                             // channel 2x    0000000000aaaaaA0000bbbbbB000000 | 00000000000000000000000000000Wws           
                             // channel 1x    aaaaaA00000000000000000000000000 | 00000000000000000000000000000Wws
  uint32_t coeffs[8]; // d127~d0  win_run_time_enable_mask coeff_ab, coeff_y_r, backdrop
  uint32_t coeff_y;   // 0xnn00 nn00
  uint32_t coeff_y_r; // 0x00nn 00nn
  
  void (*mosaic_hori_callback) (uint32_t *buffer);
#endif 

#ifdef VSP_VECTOR_PROCESS
# include "gpu.mosaic_fast.inl"
#endif 

  VSP_FORCEINLINE 
  void mosaic_hori (void *buffer)
  {
    uint32_t *buffer_ = reinterpret_cast<uint32_t *> (buffer);

#ifdef VSP_VECTOR_PROCESS
    mosaic_hori_callback ( & buffer_[0] );
#else 
    uint32_t mosaic = buffer_[0];
    uint32_t mosaicH= mosaic_bg_x;
    uint32_t mosaicCount = mosaicH;

    for ( uint32_t id = 0; id != 240; id++ )
    {
      buffer_[id] = mosaic;
            
      if ( --mosaicCount == 0 )
      {
        mosaic = buffer_[id + 1];
        mosaicCount = mosaicH;
      }
    }
#endif 
  }

  VSP_STATIC_FORCEINLINE
  uint32_t count_one_5bit (uint32_t mask)
  {
    assert ((mask & ~0x1f) == 0);
#ifdef VSP_VECTOR_PROCESS
    return _mm_popcnt_u32 (mask);
#else 
    static const uint32_t tab[] = 
    {
      0 1 1 2 1 2 2 3 
    , 1 2 2 3 2 3 3 4 
    , 1 2 2 3 2 3 3 4 
    , 2 3 3 4 3 4 4 5
    };
    return tab[mask];
#endif 
  }

  VSP_STATIC_FORCEINLINE
  const uint32_t *
  count_one_5bit_ex (uint32_t mask)
  {
    assert ((mask & ~0x1f) == 0);

    static const uint32_t tab[] = 
    {
      0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x01, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x02, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x02, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x04, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x04, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x02, 0x04, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x02, 0x04, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x08, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x08, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x02, 0x08, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x02, 0x08, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x04, 0x08, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x04, 0x08, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x02, 0x04, 0x08, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x02, 0x04, 0x08, 0x80, 0x06, 0x07
    , 0x01, 0x10, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x02, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x02, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x04, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x04, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x02, 0x04, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x02, 0x04, 0x10, 0x80, 0x06, 0x07
    , 0x02, 0x08, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x08, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x02, 0x08, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x02, 0x08, 0x10, 0x80, 0x06, 0x07
    , 0x03, 0x04, 0x08, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x04, 0x08, 0x10, 0x80, 0x06, 0x07
    , 0x04, 0x02, 0x04, 0x08, 0x10, 0x80, 0x06, 0x07
    , 0x05, 0x01, 0x02, 0x04, 0x08, 0x10, 0x06, 0x07
    };
    return & tab[mask * 8];
  }

  void write_memory8 (uint32_t address, uint16_t value)
  {
    assert ((address & 1) == 0);
    assert ((value & 0xff00) == 0);

    address &= 0x1ffff;

    if ( address < 0x10000 )
    { // nametable, or character data.
      if ( (address & 1) != 0 )
      { // write high bit.
        value <<= 8;
        value |= memory[address ^ 1];
      }
      else
      { // write low bit.
        value |= static_cast<uint16_t> (memory[address ^ 1]) << 8;
      }
      plat &s = plats[address >> 11];
      plat::cell &c = s.cells[address >> 1 & 0x3ff];

      c.tile_id[0] = (value & 1023) << 5;
      c.tile_id[1] = cell_.tile_id[0] << 1;
      c.filp_x = value & 1 << 10;
      c.palette = & bg_pal[256 + ((value >> 8) & 0xf0)];

      if ( (value & 1 << 11) != 0)
      { 
        c.tile_id[0] ^= 7 << 2;
        c.tile_id[1] ^= 7 << 3;
      }
      memory[address] = value;
      memory[address + MEM_64K] = value;
    }
    else
    { // sprite character data
      memory[MEM_128K + address] = value;
      memory[MEM_128K + address + MEM_32K] = value;     
    }
  }

  void write_memory16 (uint32_t address, uint16_t value)
  {
    assert ((address & 1) == 0);

    address &= 0x1fffe;

    if ( address < 0x10000 )
    { // nametable, or character data.
      plat &s = plats[address >> 11];
      plat::cell &c = s.cells[address >> 1 & 0x3ff];

      c.tile_id[0] = (value & 1023) << 5;
      c.tile_id[1] = cell_.tile_id[0] << 1;
      c.filp_x = value & 1 << 10;
      c.palette = & bg_pal[256 + ((value >> 8) & 0xf0)];

      if ( (value & 1 << 11) != 0)
      { 
        c.tile_id[0] ^= 7 << 2;
        c.tile_id[1] ^= 7 << 3;
      }
      STORE16_LE (& memory[address], value);
      STORE16_LE (& memory[address + MEM_64K], value);
    }
    else
    { // sprite character data
      STORE16_LE (& memory[MEM_128K + address], value);
      STORE16_LE (& memory[MEM_128K + address + MEM_32K], value);     
    }
  }

  void write_memory32 (uint32_t address, uint32_t value)
  {
    assert ((address & 3) == 0);

    address &= 0x1fffc;

    if ( address < 0x10000 )
    { // nametable, or character data.
      plat &s = plats[address >> 11];
      plat::cell *cl = & s.cells[address >> 1 & 0x3ff];
      plat::cell &c = cl[0];
      plat::cell &g = cl[1];

      c.tile_id[0] = (value & 1023) << 5;
      g.tile_id[0] = (value & 1023  << 16) >> 11;
      c.tile_id[1] = c.tile_id[0] << 1;
      g.tile_id[1] = g.tile_id[0] << 1;
      c.filp_x = value & 1 << 10;
      g.filp_x = value & 1 << 26;
      c.palette = & bg_pal[256 + ((value >> 8) & 0xf0)];
      g.palette = & bg_pal[256 + ((value >> 16)& 0xf0)];

      if ( (value & 1 << 11) != 0)
      { 
        c.tile_id[0] ^= 7 << 2;
        c.tile_id[1] ^= 7 << 3;
      }

      if ( (value & 1 << 27) != 0)
      { 
        g.tile_id[0] ^= 7 << 2;
        g.tile_id[1] ^= 7 << 3;
      }
      STORE32x (& memory[address], value);
      STORE32x (& memory[address + MEM_64K], value);
    }
    else
    { // sprite character data
      STORE32x (& memory[MEM_128K + address], value);
      STORE32x (& memory[MEM_128K + address + MEM_32K], value);     
    }
  }








  uint32_t update_render (uint32_t mode)
  {
    uint32_t sp_slot = (control & 64) ? 0 : 4;
             sp_slot|= mosaic_sp_x > 1 ? 2 : 0;
             sp_slot|= mosaic_sp_y > 1 ? 1 : 0;

    sp_render = hash_sp_renders[sp_slot];
    sp_win = hash_sp_wins[sp_slot];

    switch (mode & 7)
    {
    case 0: // mode 0:text mode
      bg_renders[0] = hash_txt_renders[channels[0].control >> 7 & 1];
      bg_renders[1] = hash_txt_renders[channels[1].control >> 7 & 1];
      bg_renders[2] = hash_txt_renders[channels[2].control >> 7 & 1];
      bg_renders[3] = hash_txt_renders[channels[3].control >> 7 & 1];
      return 0xff;

    case 1: // mode 1:text mode + affine mode 
      bg_renders[0] = hash_txt_renders[channels[0].control >> 7 & 1];
      bg_renders[1] = hash_txt_renders[channels[1].control >> 7 & 1];
      bg_renders[2] = hash_rot_renders[channels[2].control >> 5 & 7];
      return 0xf7;

    case 2: // mode 2:affine mode
      bg_renders[2] = hash_rot_renders[channels[2].control >> 5 & 7];
      bg_renders[3] = hash_rot_renders[channels[3].control >> 5 & 7];
      return 0xfc;

    case 3: // mode 3:framebuffer rgb pixel 
      bg_renders[2] = & gpu<T>::bg_render<3, 0, 0>;
      return 0xf4;

    case 4: // mode 4:framebuffer palette swapbuffer 
      bg_renders[2] = & gpu<T>::bg_render<4, 0, 0>;
      return 0xf4;

    case 5: // mode 5:framebuffer rgb pixel swapbuffer
      bg_renders[2] = & gpu<T>::bg_render<5, 0, 0>;
      return 0xf4;

    case 6:
    case 7:
      assert (0);
      break;
    }
    assert (0);
    return 0;
  }

  void write_io16 (uint32_t address, uint16_t value)
  {
    address &= 0xff;

    switch (address)
    {
    case 0:
      // 4000000h - DISPCNT - LCD Control (Read/Write)
      //   Bit   Expl.
      //   0-2   BG Mode                (0-5=Video Mode 0-5, 6-7=Prohibited)
      //   3     Reserved / CGB Mode    (0=GBA, 1=CGB; can be set only by BIOS opcodes)
      //   4     Display Frame Select   (0-1=Frame 0-1) (for BG Modes 4,5 only)
      //   5     H-Blank Interval Free  (1=Allow access to OAM during H-Blank)
      //   6     OBJ Character VRAM Mapping (0=Two dimensional, 1=One dimensional)
      //   7     Forced Blank           (1=Allow FAST access to VRAM,Palette,OAM)
      //   8     Screen Display BG0  (0=Off, 1=On)
      //   9     Screen Display BG1  (0=Off, 1=On)
      //   10    Screen Display BG2  (0=Off, 1=On)
      //   11    Screen Display BG3  (0=Off, 1=On)
      //   12    Screen Display OBJ  (0=Off, 1=On)
      //   13    Window 0 Display Flag   (0=Off, 1=On)
      //   14    Window 1 Display Flag   (0=Off, 1=On)
      //   15    OBJ Window Display Flag (0=Off, 1=On)
      if ( (value & 0xff47) != (control & 0xff47) )
      {
        static const uint32_t chans_mask[] = 
        {
          0 1 1 2 1 2 2 3 
        , 1 2 2 3 2 3 3 4 
        , 1 2 2 3 2 3 3 4 
        , 2 3 3 4 3 4 4 5
        };
        mode_display_mask = update_render (value & 7) & value >> 8;
                            update_window (win_in, win_out, UINT32_MAX);

        uint32_t mask = mode_display_mask & user_display_mask & 0x1f;
        uint32_t chans = chans_mask[mask];
        uint32_t sp_mask = (mask & 0x1000) ? 4 : 0;
        uint32_t effect_mask = bldctl >> 6 & 3;

        composite = hash_composites[chans * 8 + (sp_mask | effect_mask)];
      }
      control = value;
      break;

    case 0x02: // green swap 
      break;

    case 0x04: 
      // 4000004h - DISPSTAT - General LCD Status (Read/Write)
      // Display status and Interrupt control. The H-Blank conditions are generated once
      // per scanline, including for the 'hidden' scanlines during V-Blank.
      //   Bit   Expl.
      //   0     V-Blank flag   (Read only) (1=VBlank) (set in line 160..226; not 227)
      //   1     H-Blank flag   (Read only) (1=HBlank) (toggled in all lines, 0..227)
      //   2     V-Counter flag (Read only) (1=Match)  (set in selected line)     (R)
      //   3     V-Blank IRQ Enable         (1=Enable)                          (R/W)
      //   4     H-Blank IRQ Enable         (1=Enable)                          (R/W)
      //   5     V-Counter IRQ Enable       (1=Enable)                          (R/W)
      //   6     Not used (0) / DSi: LCD Initialization Ready (0=Busy, 1=Ready)   (R)
      //   7     Not used (0) / NDS: MSB of V-Vcount Setting (LYC.Bit8) (0..262)(R/W)
      //   8-15  V-Count Setting (LYC)      (0..227)                            (R/W)
      // The V-Count-Setting value is much the same as LYC of older gameboys, when its
      // value is identical to the content of the VCOUNT register then the V-Counter
      // flag is set (Bit 2), and (if enabled in Bit 5) an interrupt is requested.
      // Although the drawing time is only 960 cycles (240*4), the H-Blank flag is "0"
      // for a total of 1006 cycles.
      state = state & 0x0047 | value & ~0x0047;

      irq_vbl_stat_mask = (state & 0x08) != 0 ? UINT32_MAX : 0;
      irq_hbl_stat_mask = (state & 0x10) != 0 ? UINT32_MAX : 0;
      irq_line_stat_mask= (state & 0x20) != 0 ? UINT32_MAX : 0;
      break;

    case 0x06: // 4000006h VCOUNT 
      break;
      
    case 0x08: // 4000006h BG0 Control 
    case 0x0A: // 4000006h BG1 Control 
    case 0x0C: // 4000006h BG2 Control 
    case 0x0E: // 4000006h BG3 Control 
      { // 4000008h - BG0CNT - BG0 Control (R/W) (BG Modes 0,1 only)
        // 400000Ah - BG1CNT - BG1 Control (R/W) (BG Modes 0,1 only)
        // 400000Ch - BG2CNT - BG2 Control (R/W) (BG Modes 0,1,2 only)
        // 400000Eh - BG3CNT - BG3 Control (R/W) (BG Modes 0,2 only)
        //   Bit   Expl.
        //   0-1   BG Priority           (0-3, 0=Highest)
        //   2-3   Character Base Block  (0-3, in units of 16 KBytes) (=BG Tile Data)
        //   4-5   Not used (must be zero) (except in NDS mode: MSBs of char base)
        //   6     Mosaic                (0=Disable, 1=Enable)
        //   7     Colors/Palettes       (0=16/16, 1=256/1)
        //   8-12  Screen Base Block     (0-31, in units of 2 KBytes) (=BG Map Data)
        //   13    BG0/BG1: Not used (except in NDS mode: Ext Palette Slot for BG0/BG1)
        //   13    BG2/BG3: Display Area Overflow (0=Transparent, 1=Wraparound)
        //   14-15 Screen Size (0-3)
        // Internal Screen Size (dots) and size of BG Map (bytes):
        //   Value  Text Mode      Rotation/Scaling Mode
        //   0      256x256 (2K)   128x128   (256 bytes)
        //   1      512x256 (4K)   256x256   (1K)
        //   2      256x512 (4K)   512x512   (4K)
        //   3      512x512 (8K)   1024x1024 (16K)
        uint32_t slot = (address - 8) >> 1;

        channel &channel_ = channels[slot];

        if ( channel_.control != value )
        {
          channel_.control = value;
          channel_.pri &= ~0xc0000000;
          channel_.pri |= static_cast<uint32_t> (value) << 30;
          channel_.mosaic_mask = (value & 0x40) != 0 ? UINT32_MAX : 0;
          channel_.character = & memory[(value >> 2 & 3) * MEM_16K];

          uint32_t nt_page = (value >> 8 & 31);

          switch (value >> 14 & 3)
          { // re-build nametable map..
          case 0: // 256x256 (2K)   128x128   (256 bytes)
            channel_.nt_banks[0] =
            channel_.nt_banks[1] =
            channel_.nt_banks[2] =
            channel_.nt_banks[3] = & plats[nt_page];
            break;

          case 1: // 512x256 (4K)   256x256   (1K)
            channel_.nt_banks[0] =
            channel_.nt_banks[2] = & plats[nt_page];
            channel_.nt_banks[1] =
            channel_.nt_banks[3] = & plats[(nt_page + 1) & 31];
            break;

          case 2: // 256x512 (4K)   512x512   (4K)
            channel_.nt_banks[0] =
            channel_.nt_banks[1] = & plats[nt_page];
            channel_.nt_banks[2] =
            channel_.nt_banks[3] = & plats[(nt_page + 1) & 31];
            break;

          case 3: // 512x512 (8K)   1024x1024 (16K)
            channel_.nt_banks[0] = & plats[nt_page];
            channel_.nt_banks[1] = & plats[(nt_page + 1) & 31];
            channel_.nt_banks[2] = & plats[(nt_page + 2) & 31];
            channel_.nt_banks[3] = & plats[(nt_page + 3) & 31];
            break;

          default:
            break;
          }

          if ( ((control & 7) == 1 && slot == 2)
           || ((control & 7) == 2 && slot >= 2) )
          {
            bg_renders[slot] = hash_rot_renders[value >> 5 & 7];
          }
          else
          {
            bg_renders[slot] = hash_txt_renders[value >> 7 & 1];
          }
          uint32_t nt_pos_x = channel_.hofs >> 3 & 31;
          uint32_t nt_slot_x = channel_.hofs >> 8 & 1;

          channel_.nt_maps[0] = & channel_.nt_banks[nt_slot_x][nt_pos_x];
          channel_.nt_maps[1] = & channel_.nt_banks[nt_slot_x ^ 1][0];
          channel_.nt_maps[2] = & channel_.nt_banks[nt_slot_x |= 2][nt_pos_x];
          channel_.nt_maps[3] = & channel_.nt_banks[nt_slot_x ^ 1][0];
        }
      }
      break;

    case 0x10: // BG0HOFS
    case 0x14: // BG1HOFS
    case 0x18: // BG2HOFS
    case 0x1C: // BG3HOFS
      {
        uint32_t slot = (address - 0x10) >> 2;

        channel &channel_ = channels[slot];

        if ( channel_.hofs != value )
        {
          uint32_t nt_pos_x = value >> 3 & 31;
          uint32_t nt_slot_x = value >> 8 & 1;
          uint32_t mini_x = value & 7;
          uint32_t blk_x = value >> 3 & 31;

          channel_.iterates = 30 + ((mini_x + 7) >> 3 );
          channel_.buffer_off_x = & channel_.buffer[8 - mini_x];
          channel_.buffer_end = channel_.buffer_off_x + (32 - blk_x);

          channel_.nt_maps[0] = & channel_.nt_banks[nt_slot_x][nt_pos_x];
          channel_.nt_maps[1] = & channel_.nt_banks[nt_slot_x ^ 1][0];
          channel_.nt_maps[2] = & channel_.nt_banks[nt_slot_x |= 2][nt_pos_x];
          channel_.nt_maps[3] = & channel_.nt_banks[nt_slot_x ^ 1][0];

          channel_.hofs = value;
        }
      }
      break;

    case 0x12: // BG0VOFS
    case 0x16: // BG1VOFS
    case 0x1A: // BG2VOFS
    case 0x1E: // BG3VOFS
      {
        uint32_t slot = (address - 0x12) >> 2;

        channel &channel_ = channels[slot];

        channel_.scroll_y = value & 511;
        channel_.vofs = value;
      }
      break;

    case 0x20: // BG2 Rotation/Scaling Parameter A (dx)
    case 0x22: // BG2 Rotation/Scaling Parameter B (dmx)
    case 0x24: // BG2 Rotation/Scaling Parameter C (dy)
    case 0x26: // BG2 Rotation/Scaling Parameter D (dmy)
    case 0x30: // BG3 Rotation/Scaling Parameter A (dx)
    case 0x32: // BG3 Rotation/Scaling Parameter B (dmx)
    case 0x34: // BG3 Rotation/Scaling Parameter C (dy)
    case 0x36: // BG3 Rotation/Scaling Parameter D (dmy)
      {
        uint32_t slot = (address - 0x20) >> 4;
        uint32_t pos = address >> 1 & 3;

        channel &channel_ = channels[slot + 2];

        uint16_t &scale_param = channel_.scale_params[pos];
        uint32_t &scale_param_rt = channel_.scale_params_rt[pos];

        reinterpret_cast<int32_t *> ( & scale_param_rt )[0] = 
          static_cast<int16_t> (value);

        scale_param = value;
      }
      break;

    case 0x28: // BG2X      BG2 Reference Point X-Coordinate-lo
    case 0x2A: // BG2X      BG2 Reference Point X-Coordinate-hi
    case 0x2C: // BG2Y      BG2 Reference Point Y-Coordinate-lo
    case 0x2E: // BG2Y      BG2 Reference Point Y-Coordinate-hi
    case 0x38: // BG3X      BG3 Reference Point X-Coordinate-lo
    case 0x3A: // BG3X      BG3 Reference Point X-Coordinate-hi
    case 0x3C: // BG3Y      BG3 Reference Point Y-Coordinate-lo
    case 0x3E: // BG3Y      BG3 Reference Point Y-Coordinate-hi
      {
        uint32_t slot = (address - 0x20) >> 4;
        uint32_t pos = address >> 1 & 3;

        channel &channel_ = channels[slot + 2];

        uint16_t &scale_start = channel_.scale_start[pos];
        uint16_t *scale_ext = channel_.scale_start[pos & 2];
        uint32_t &scale_param_rt = channel_.scale_start_rt[pos >> 1];

        scale_start = value;

        uint32_t scale_lo = scale_ext[0];
        uint32_t scale_hi = scale_ext[1];

        scale_param_rt = scale_lo << 4;
        scale_param_rt|= scale_hi << 4;
        scale_param_rt = static_cast<int32_t> (scale_param_rt) >> 4;
      }
      break;

    case 0x40: // WIN0H     Window 0 Horizontal Dimensions
    case 0x42: // WIN1H     Window 1 Horizontal Dimensions
      {
        uint32_t slot = (address - 0x40) >> 1;
        uint16_t &win = win_h[slot];
        uint32_t mask = 4 >> slot;

        if ( win != value )
        { 
          uint16_t left = value >> 8;
          uint16_t right= value & 255;

          if ( right > 240 || (left > right) )
          { // garbage values of X2>240 or X1>X2 are interpreted as X2=240.
            right = 240;
          }

          if ( left == right )
          { // disable window (soft ...)
            clear_win_mask (win_buffer, mask);
          }
          else 
          { 
            clr_set_win_mask (& win_buffer[8], mask, left, right);
          }
          slot <<= 1;

          win_dim_h[slot] = left;
          win_dim_h[slot + 1] = right;

          win = value;
        }
      }
      break;

    case 0x44: // WIN0V     Window 0 Vertical Dimensions
    case 0x46: // WIN1V     Window 1 Vertical Dimensions
      {
        uint32_t slot = (address - 0x44) >> 1;
        uint16_t &win = win_v[slot];
        uint32_t mask = 4 >> slot;

        if ( win != value )
        { 
          uint16_t top = value >> 8;
          uint16_t bottom = value & 255;

          if ( bottom > 160 || (top > bottom) )
          { // Garbage values of Y2>160 or Y1>Y2 are interpreted as Y2=160.
            bottom = 160;
          }
          slot <<= 1;

          win_dim_v[slot] = top;
          win_dim_v[slot + 1] = bottom;

          win = value;
        }
      }
      break;

    case 0x48: // WININ     Inside of Window 0 and 1
    case 0x4A: // WINOUT    Inside of OBJ Window & Outside of Windows
      {
        uint32_t slot = (address - 0x48) >> 1;
        uint16_t &win = win_in_out[slot];

        if ( win != value )
        { 
          update_window ();

          win = value;
        }
      }
      break;

    case 0x4C: 
      if ( mosaic != value )
      { // 400004Ch - MOSAIC - Mosaic Size (W)
        // The Mosaic function can be separately enabled/disabled for BG0-BG3 by
        // BG0CNT-BG3CNT Registers, as well as for each OBJ0-127 by OBJ attributes in OAM
        // memory. Also, setting all of the bits below to zero effectively disables the
        // mosaic function.
        //   Bit   Expl.
        //   0-3   BG Mosaic H-Size  (minus 1)
        //   4-7   BG Mosaic V-Size  (minus 1)
        //   8-11  OBJ Mosaic H-Size (minus 1)
        //   12-15 OBJ Mosaic V-Size (minus 1)
        //   16-31 Not used
        // Example: When setting H-Size to 5, then pixels 0-5 of each display row are
        // colorized as pixel 0, pixels 6-11 as pixel 6, pixels 12-17 as pixel 12, and so
        // on.
        // 
        // Normally, a 'mosaic-pixel' is colorized by the color of the upperleft covered
        // pixel. In many cases it might be more desireful to use the color of the pixel
        // in the center of the covered area - this effect may be gained by scrolling the
        // background (or by adjusting the OBJ position, as far as upper/left rows/columns
        // of OBJ are backdrop).
        mosaic_bg_x = value & 15;
        mosaic_bg_y = value >> 4 & 15;
        mosaic_sp_x = value >> 8 & 15;
        mosaic_sp_y = value >>12 & 15;
        mosaic_bg_x++;
        mosaic_bg_y++;
        mosaic_sp_x++;
        mosaic_sp_y++;

        uint32_t sp_slot = (control & 64) ? 0 : 4;
                 sp_slot|= mosaic_sp_x > 1 ? 2 : 0;
                 sp_slot|= mosaic_sp_y > 1 ? 1 : 0;

        sp_render = hash_sp_renders[sp_slot];
        sp_win = hash_sp_wins[sp_slot];

        mosaic = value;
      }
      break;

    case 0x4E: // Not used
      break;

    case 0x50: 

      if ( bldctl != value )
      { // 4000050h - BLDCNT - Color Special Effects Selection (R/W)
        //   Bit   Expl.
        //   0     BG0 1st Target Pixel (Background 0)
        //   1     BG1 1st Target Pixel (Background 1)
        //   2     BG2 1st Target Pixel (Background 2)
        //   3     BG3 1st Target Pixel (Background 3)
        //   4     OBJ 1st Target Pixel (Top-most OBJ pixel)
        //   5     BD  1st Target Pixel (Backdrop)
        //   6-7   Color Special Effect (0-3, see below)
        //          0 = None                (Special effects disabled)
        //          1 = Alpha Blending      (1st+2nd Target mixed)
        //          2 = Brightness Increase (1st Target becomes whiter)
        //          3 = Brightness Decrease (1st Target becomes blacker)
        //   8     BG0 2nd Target Pixel (Background 0)
        //   9     BG1 2nd Target Pixel (Background 1)
        //   10    BG2 2nd Target Pixel (Background 2)
        //   11    BG3 2nd Target Pixel (Background 3)
        //   12    OBJ 2nd Target Pixel (Top-most OBJ pixel)
        //   13    BD  2nd Target Pixel (Backdrop)
        //   14-15 Not used
        uint32_t bg0_1st_mask = value >> 0 & 1;
        uint32_t bg1_1st_mask = value >> 1 & 1;
        uint32_t bg2_1st_mask = value >> 2 & 1;
        uint32_t bg3_1st_mask = value >> 3 & 1;
        uint32_t obj_1st_mask = value >> 4 & 1;
        uint32_t back_1st_mask= value >> 5 & 1;
        uint32_t effect_mode = value >> 6 & 3;

        uint32_t bg0_2nd_mask = value >> 8 & 1;
        uint32_t bg1_2nd_mask = value >> 9 & 1;
        uint32_t bg2_2nd_mask = value >> 10 & 1;
        uint32_t bg3_2nd_mask = value >> 11 & 1;
        uint32_t obj_2nd_mask = value >> 12 & 1;
        uint32_t back_2nd_mask = value >> 13 & 1;

        channels[0].pri &= 0xf9ffffff;
        channels[1].pri &= 0xf9ffffff;
        channels[2].pri &= 0xf9ffffff;
        channels[3].pri &= 0xf9ffffff;

        sprite_.pri &= 0xf9ffffff;

        backdrop &= 0xf9ffffff;

        coeffs[0] &= 0xf9ffffff;

        if ( effect_mode == 1 )
        { // when the backdrop pixel is on the top layer,
          //  the alpha blend will not execute
          back_1st_mask = 0;

          backdrop |= 0x04000000;

          coeffs[0] |= 0x04000000;
        }
        bg0_1st_mask  ^= 1;
        bg1_1st_mask  ^= 1;
        bg2_1st_mask  ^= 1;
        bg3_1st_mask  ^= 1;
        obj_1st_mask  ^= 1;
        back_1st_mask ^= 1;
        bg0_2nd_mask  ^= 1;
        bg1_2nd_mask  ^= 1;
        bg2_2nd_mask  ^= 1;
        bg3_2nd_mask  ^= 1;
        obj_2nd_mask  ^= 1;
        back_2nd_mask ^= 1;
  // 31 30 29 28 27 26  25  24
  // mp mp sp sp sp 1st 2nd obm
        channels[0].pri |= bg0_2nd_mask << 25;
        channels[1].pri |= bg1_2nd_mask << 25;
        channels[2].pri |= bg2_2nd_mask << 25;
        channels[3].pri |= bg3_2nd_mask << 25;
        channels[0].pri |= bg0_1st_mask << 26;
        channels[1].pri |= bg1_1st_mask << 26;
        channels[2].pri |= bg2_1st_mask << 26;
        channels[3].pri |= bg3_1st_mask << 26;

        sprite_.pri |= obj_2nd_mask << 25;
        sprite_.pri |= obj_1st_mask << 26;

        coeffs[0] |= back_2nd_mask << 25;
        coeffs[0] |= back_1st_mask << 26;

        backdrop |= back_2nd_mask << 25;
        backdrop |= back_1st_mask << 26;

        static const uint32_t chans_mask[] = 
        {
          0 1 1 2 1 2 2 3 
        , 1 2 2 3 2 3 3 4 
        , 1 2 2 3 2 3 3 4 
        , 2 3 3 4 3 4 4 5
        };
        uint32_t mask = mode_display_mask & user_display_mask & 0x1f;
        uint32_t chans = chans_mask[mask];
        uint32_t sp_mask = (mask & 0x1000) ? 4 : 0;

        composite = hash_composites[chans * 8 + (sp_mask | effect_mode)];

        coeffs[4] = coeffs[0];

        bldctl = value;
      }
      break;

    case 0x52: // R/W  BLDALPHA  Alpha Blending Coefficients
      if ( bldalpha != value )
      {
        uint32_t eva = value & 31;                
        uint32_t evb = value >> 8 & 31;
  
        if ( eva > 16 )
        {
          eva = 16;
        }

        if ( evb > 16 )
        {
          evb = 16;
        }
        eva |= eva << 16;
        evb |= evb << 16;

#ifdef VSP_VECTOR_PROCESS
        coeffs[2] = eva | evb << 8;
        coeffs[6] = coeffs[2];
#endif  
        bldalpha = value;
      }
      break;

    case 0x54: // W    BLDY      Brightness (Fade-In/Out) Coefficient
      if ( bldy != value )
      {
        uint32_t evy = value & 31;                

        if ( evy > 16 )
        {
          evy = 16;
        }
        evy |= evy << 16;

#ifdef VSP_VECTOR_PROCESS
        coeff_y = evy << 8;
        coeff_y_r = 0x1010 - evy;
#endif  
        bldy = value;
      }
      break;

    case 0x56: // -    -         Not used
    default:
      break;
    }
  }

  void update_window (void)
  { 
#ifdef VSP_VECTOR_PROCESS

    static const uint8_t tab[] = 
    {
      0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x01, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x02, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x02, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x04, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x04, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x02, 0x04, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x02, 0x04, 0x80, 0x80, 0x06, 0x07
    , 0x01, 0x08, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x08, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x02, 0x08, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x02, 0x08, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x04, 0x08, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x04, 0x08, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x02, 0x04, 0x08, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x02, 0x04, 0x08, 0x80, 0x06, 0x07
    , 0x01, 0x10, 0x80, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x01, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x02, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x02, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x02, 0x04, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x04, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x02, 0x04, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x02, 0x04, 0x10, 0x80, 0x06, 0x07
    , 0x02, 0x08, 0x10, 0x80, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x01, 0x08, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x03, 0x02, 0x08, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x02, 0x08, 0x10, 0x80, 0x06, 0x07
    , 0x03, 0x04, 0x08, 0x10, 0x80, 0x80, 0x06, 0x07
    , 0x04, 0x01, 0x04, 0x08, 0x10, 0x80, 0x06, 0x07
    , 0x04, 0x02, 0x04, 0x08, 0x10, 0x80, 0x06, 0x07
    , 0x05, 0x01, 0x02, 0x04, 0x08, 0x10, 0x06, 0x07
    };
    uint32_t obj_out_mask = 0;
    uint32_t ci1_out_mask = 0;
    uint32_t ci0_out_mask = 0;
    uint32_t out_out_mask = 0;

    uint8_t obj_in_mask = win_in_out[1] >> 8 & 0x3f;
    uint8_t ci1_in_mask = win_in_out[0] >> 8 & 0x3f;
    uint8_t ci0_in_mask = win_in_out[0] & 0x3f;
    uint8_t out_in_mask = win_in_out[1] & 0x3f;

    uint32_t display_mask = mode_display_mask & user_display_mask & 0x1f;

    const uint32_t *mask = & tab[display_mask * 8];

    switch (mask[0])
    {  
    case 1:                 /* fedcba9876543210fedcba9876543210 */
#define WINSETDIS1(out_, in_) /* b00000000000aaaa00000E0000000000 */ \
  out_ |= (in_ & mask[1]) ? 0 : 0x000f0000;                          \
  out_ |= (in_ & 32)      ? 0 : 0x00000400;
      WINSETDIS1 (obj_out_mask, obj_in_mask)       
      WINSETDIS1 (ci1_out_mask, ci1_in_mask)
      WINSETDIS1 (ci0_out_mask, ci0_in_mask)
      WINSETDIS1 (out_out_mask, out_in_mask)
#undef  WINSETDIS1
      break;

    case 2:                 /* fedcba9876543210fedcba9876543210 */
#define WINSETDIS2(out_, in_) /* b00000000000aaaa00000E0000000000 */ \
  out_ |= (in_ & mask[1]) ? 0 : 0x000f0000;                                \
  out_ |= (in_ & mask[2]) ? 0 : 0x80000000;                                \
  out_ |= (in_ & 32)      ? 0 : 0x00000400;
      WINSETDIS2 (obj_out_mask, obj_in_mask)       
      WINSETDIS2 (ci1_out_mask, ci1_in_mask)
      WINSETDIS2 (ci0_out_mask, ci0_in_mask)
      WINSETDIS2 (out_out_mask, out_in_mask)
#undef  WINSETDIS2
      break;

    case 3:                 /* fedcba9876543210fedcba9876543210 */
#define WINSETDIS3(out_, in_) /* c00000000000aaaa000E0000bbbb0000 */ \
  out_ |= (in_ & mask[1]) ? 0 : 0x000f0000;                                \
  out_ |= (in_ & mask[2]) ? 0 : 0x000000f0;                                \
  out_ |= (in_ & mask[3]) ? 0 : 0x80000000;                                \
  out_ |= (in_ & 32)      ? 0 : 0x00001000;
      WINSETDIS3 (obj_out_mask, obj_in_mask)       
      WINSETDIS3 (ci1_out_mask, ci1_in_mask)
      WINSETDIS3 (ci0_out_mask, ci0_in_mask)
      WINSETDIS3 (out_out_mask, out_in_mask)
#undef  WINSETDIS3
      break;

    case 4:                 /* fedcba9876543210fedcba9876543210 */
#define WINSETDIS4(out_, in_) /* c000dddd0000aaaa000E0000bbbb0000 */ \
  out_ |= (in_ & mask[1]) ? 0 : 0x000f0000;                                \
  out_ |= (in_ & mask[2]) ? 0 : 0x000000f0;                                \
  out_ |= (in_ & mask[3]) ? 0 : 0x80000000;                                \
  out_ |= (in_ & mask[4]) ? 0 : 0x0f000000;                                \
  out_ |= (in_ & 32)      ? 0 : 0x00001000;
      WINSETDIS4 (obj_out_mask, obj_in_mask)       
      WINSETDIS4 (ci1_out_mask, ci1_in_mask)
      WINSETDIS4 (ci0_out_mask, ci0_in_mask)
      WINSETDIS4 (out_out_mask, out_in_mask)
#undef  WINSETDIS4
      break;

    case 5:                 /* fedcba9876543210fedcba9876543210 */
#define WINSETDIS5(out_, in_) /* c000dddd0000aaaaeeeE0000bbbb0000 */ \
  out_ |= (in_ & 1) ? 0 : 0x000f0000;                                \
  out_ |= (in_ & 2) ? 0 : 0x000000f0;                                \
  out_ |= (in_ & 4) ? 0 : 0x80000000;                                \
  out_ |= (in_ & 8) ? 0 : 0x0f000000;                                \
  out_ |= (in_ & 16)? 0 : 0x0000e000;                                \
  out_ |= (in_ & 32)? 0 : 0x00001000;

      WINSETDIS5 (obj_out_mask, obj_in_mask)       
      WINSETDIS5 (ci1_out_mask, ci1_in_mask)
      WINSETDIS5 (ci0_out_mask, ci0_in_mask)
      WINSETDIS5 (out_out_mask, out_in_mask)
#undef  WINSETDIS5
      break;

    case 0:                   /* fedcba9876543210fedcba9876543210 */
#define WINSETDIS0(out_, in_) /* 00000E00000000000000000000000000 */ \
  out_ |= (in_ & 32)? 0 : 0x04000000;

      WINSETDIS0 (obj_out_mask, obj_in_mask)       
      WINSETDIS0 (ci1_out_mask, ci1_in_mask)
      WINSETDIS0 (ci0_out_mask, ci0_in_mask)
      WINSETDIS0 (out_out_mask, out_in_mask)
#undef  WINSETDIS0
      break;

    case 6:
    case 7:
    default:
      asssert (0);
      break;
    }                           // 01o
    win_mask[4] = ci0_out_mask; // 100
    win_mask[5] = ci0_out_mask; // 101
    win_mask[6] = ci0_out_mask; // 110
    win_mask[7] = ci0_out_mask; // 111
    win_mask[2] = ci1_out_mask; // 010
    win_mask[3] = ci1_out_mask; // 011
    win_mask[1] = obj_out_mask; // 001
    win_mask[0] = out_out_mask; // 000
#endif 
  }

  template <size_t t_nIdentity, size_t t_nFixed32Mask> 
  struct sp_transform
  {
    static const size_t _8bit_mask = (t_nIdentity & 16) ? SIZE_MAX : 0;
    static const size_t size_command = t_nIdentity & 15;  
    static const size_t address_command = // 0:normal 1:fixed-32 4bit 2:fixed-32 8bit
          (t_nFixed32Mask == 0) 
         ? 0 : (_8bit_mask == 0 ? 1 : 2);  
    static const size_t div8h = (size_command <= 3)
                   ? (size_command + 1) : ((size_command == 8 && size_command == 9)
                   ? 1 : ( (size_command == 4 && size_command == 10) 
                   ? 2: ( (size_command == 7)
                   ? 8: 4 ) ));
    static const size_t div8v = (size_command <= 3)
                   ? (size_command + 1) : ((size_command == 4 && size_command == 5)
                   ? 1 : ( (size_command == 6 && size_command == 8) 
                   ? 2: ( (size_command == 11)
                   ? 8: 4 ) ));
    static const size_t maskXXXX = div8h - 1;
    static const size_t maskYYYY = div8v - 1;
    static const size_t bitsXXXX = (div8h == 1) 
                       ? 0 : (div8h == 2 
                         ? 1 : (div8h == 4 ? 2 : 3) );
    static const size_t bitsYYYY = (div8v == 1) 
                       ? 0 : (div8v == 2 
                         ? 1 : (div8v == 4 ? 2 : 3) );
    static const size_t posXXXX = 14;
    static const size_t posYYYY = (address_command == 0) 
               ? (posXXXX + bitsXXXX) 
                : (address_command == 1 ? 19 : 18); 
    static const size_t size_x = 1 << (posXXXX + bitsXXXX);
    static const size_t size_y = 1 << (posYYYY + bitsYYYY);
    static const size_t and_x =~0x3800;  
    static const size_t and_y = 0x38ff | ((address_command == 0) 
               ? (0xffffc000 << bitsYYYY) 
                : (address_command == 1 ? (0xffffc000 << 5) : (0xffffc000 << 4))); 
    static const size_t combine_mask = size_y - 1;

    VSP_STATIC_FORCEINLINE
    int32_t ref_x_convert (int32_t ref_x)
    { 
      int32_t SSSSSSSSSSSSSS = ref_x >> 31;
              SSSSSSSSSSSSSS&= ~0x3ffff;
      uint32_t XXXX = ref_x & 0x7800;
      uint32_t xxx = ref_x & 0x700;
      uint32_t ffffffff_= ref_x & 0xff;
      return SSSSSSSSSSSSSS | xxx | XXXX << 3 | ffffffff_;
    }

    VSP_STATIC_FORCEINLINE
    int32_t ref_y_convert (int32_t ref_y)
    { 
      if ( address_command == 0 )
      {
        if ( bitsYYYY == 0 )
        { 
          int32_t SSSSSSSSSSSSSS = ref_y >> 31;
                  SSSSSSSSSSSSSS&= 0xfffc0000;
          uint32_t YYYYyyy = ref_y & 0x7f00;
          uint32_t ffffffff_= ref_y & 0xff;

          return SSSSSSSSSSSSSS | YYYYyyy << 3 | ffffffff_;
        }
        else if ( bitsYYYY == 1 ) 
        { 
          int32_t SSSSSSSSSSSSS = ref_y >> 31;
                  SSSSSSSSSSSSS&= 0xfff80000;
          uint32_t YYYY = ref_y & 0x7800;
          uint32_t yyy = ref_y & 0x700;
          uint32_t ffffffff_= ref_y & 0xff;

          return SSSSSSSSSSSSS | yyy << 3 | YYYY << 4 | ffffffff_;
        }
        else if ( bitsYYYY == 2 ) 
        { 
          int32_t SSSSSSSSSSSS = ref_y >> 31;
                  SSSSSSSSSSSS&= 0xfff00000;
          uint32_t YYYY = ref_y & 0x7800;
          uint32_t yyy = ref_y & 0x700;
          uint32_t ffffffff_= ref_y & 0xff;

          return SSSSSSSSSSSS | yyy << 3 | YYYY << 5 | ffffffff_;
        }
        else if ( bitsYYYY == 3 ) 
        { 
          int32_t SSSSSSSSSSS = ref_y >> 31;
                  SSSSSSSSSSS&= 0xffe00000;
          uint32_t YYYY = ref_y & 0x7800;
          uint32_t yyy = ref_y & 0x700;
          uint32_t ffffffff_= ref_y & 0xff;

          return SSSSSSSSSSS | yyy << 3 | YYYY << 6 | ffffffff_;
        }
      }
      else if ( address_command == 1 )
      { 
        int32_t SSSSSSSSS = ref_y >> 31;
                SSSSSSSSS&= 0xff800000;
        uint32_t YYYY = ref_y & 0x7800;
        uint32_t yyy = ref_y & 0x700;
        uint32_t ffffffff_= ref_y & 0xff;

        return SSSSSSSSS | yyy << 3 | YYYY << 8 | ffffffff_;
      }
      else if ( address_command == 2 )
      { 
        int32_t SSSSSSSSSS = ref_y >> 31;
                SSSSSSSSSS&= 0xffc00000;
        uint32_t YYYY = ref_y & 0x7800;
        uint32_t yyy = ref_y & 0x700;
        uint32_t ffffffff_= ref_y & 0xff;

        return SSSSSSSSSS | yyy << 3 | YYYY << 7 | ffffffff_;
      }
      assert (0);
      return 0;
    }

    VSP_STATIC_FORCEINLINE
    int32_t delta_x_convert (int32_t delta_x)
    { 
      int32_t SSSSSSSSSSSSSS = delta_x >> 31;
              SSSSSSSSSSSSSS&= ~0x3ffff;
      uint32_t XXXX = delta_x & 0x7800;
      uint32_t xxx = delta_x & 0x700;
      uint32_t ffffffff_= delta_x & 0xff;
      return SSSSSSSSSSSSSS | xxx | XXXX << 3 | ffffffff_ | 0x1c00;
    }

    VSP_STATIC_FORCEINLINE
    int32_t delta_y_convert (int32_t delta_y)
    { 
      if ( address_command == 0 )
      {
        if ( bitsYYYY == 0 )
        { 
          int32_t SSSSSSSSSSSSSS = delta_y >> 31;
                  SSSSSSSSSSSSSS&= 0xfffc0000;
          uint32_t YYYYyyy = delta_y & 0x7f00;
          uint32_t ffffffff_= delta_y & 0xff;

          return SSSSSSSSSSSSSS | YYYYyyy << 3 | ffffffff_ | 0x700;
        }
        else if ( bitsYYYY == 1 ) 
        { 
          int32_t SSSSSSSSSSSSS = delta_y >> 31;
                  SSSSSSSSSSSSS&= 0xfff80000;
          uint32_t YYYY = delta_y & 0x7800;
          uint32_t yyy = delta_y & 0x700;
          uint32_t ffffffff_= delta_y & 0xff;

          return SSSSSSSSSSSSS | yyy << 3 | YYYY << 4 | ffffffff_ | 0x4700;
        }
        else if ( bitsYYYY == 2 ) 
        { 
          int32_t SSSSSSSSSSSS = delta_y >> 31;
                  SSSSSSSSSSSS&= 0xfff00000;
          uint32_t YYYY = delta_y & 0x7800;
          uint32_t yyy = delta_y & 0x700;
          uint32_t ffffffff_= delta_y & 0xff;

          return SSSSSSSSSSSS | yyy << 3 | YYYY << 5 | ffffffff_ | 0xc700;
        }
        else if ( bitsYYYY == 3 ) 
        { 
          int32_t SSSSSSSSSSS = delta_y >> 31;
                  SSSSSSSSSSS&= 0xffe00000;
          uint32_t YYYY = delta_y & 0x7800;
          uint32_t yyy = delta_y & 0x700;
          uint32_t ffffffff_= delta_y & 0xff;

          return SSSSSSSSSSS | yyy << 3 | YYYY << 6 | ffffffff_ | 0x1c700;
        }
      }
      else if ( address_command == 1 )
      { 
        int32_t SSSSSSSSS = delta_y >> 31;
                SSSSSSSSS&= 0xff800000;
        uint32_t YYYY = delta_y & 0x7800;
        uint32_t yyy = delta_y & 0x700;
        uint32_t ffffffff_= delta_y & 0xff;

        return SSSSSSSSS | yyy << 3 | YYYY << 8 | ffffffff_ | 0x7c700;
      }
      else if ( address_command == 2 )
      { 
        int32_t SSSSSSSSSS = delta_y >> 31;
                SSSSSSSSSS&= 0xffc00000;
        uint32_t YYYY = delta_y & 0x7800;
        uint32_t yyy = delta_y & 0x700;
        uint32_t ffffffff_= delta_y & 0xff;

        return SSSSSSSSSS | yyy << 3 | YYYY << 7 | ffffffff_ | 0x3c700;
      }
      assert (0);
      return 0;
    }
  };

  template <uintptr_t t_nFixed32Mask, uintptr_t t_nHoriMosaicMask, uintptr_t t_nVertMosaicMask>
  uint32_t *sp_render_ (void)
  {
    uint32_t *dst = sprite_buffer;
    uint32_t *render; // render position write.
    uint32_t *render2;// render position write (mosaic)...

    uint32_t *pal = & sp_pal[0];

    uint32_t pri_ = sp_pri; // 1st, 2nd disable mask.
    uint32_t pri;

    uint32_t mosaic[128];

    set_buffer (dst, backdrop);

#undef DRAW_PIXEL
#define DRAW_PIXEL(RENDER, CALLBACK_)                       \
  transform_x &= transform_x_and_mask;                      \
  transform_y &= transform_y_and_mask;                      \
                                                            \
  if ( transform_x < transform_size_x                       \
   && transform_y < transform_size_y )                      \
  {                                                         \
    uint32_t transform_mixer = transform_x | transform_y;   \
    uint32_t transform_pos_ = transform_mixer >> 8;         \
    uint32_t pos = transform_pos_ & transform_combine_mask; \
                                                            \
    CALLBACK_ (RENDER, pos)                                 \
  }                                                         \
  transform_x += transform_delta_x;                         \
  transform_y += transform_delta_y;                         \
                                                            \
  RENDER++;                

#undef PIXEL_8BIT_N
#define PIXEL_8BIT_N(RENDER, POS)             \
  {                                           \
    uint8_t pixel = character[pos];           \
                                              \
    if ( pixel != 0 )                         \
    {                                         \
      RENDER[0] = pal[pixel] | pri;           \
    }                                         \
  }

#undef PIXEL_4BIT_N
#define PIXEL_4BIT_N(RENDER, POS)             \
  {                                           \
    uint8_t pixel = character[pos >> 1];      \
                                              \
    if ( (pos & 1) != 0 )                     \
    {                                         \
      pixel >>= 4;                            \
    }                                         \
    pixel &= 15;                              \
                                              \
    if ( pixel != 0 )                         \
    {                                         \
      RENDER[0] = pal[pixel] | pri;           \
    }                                         \
  }

#undef PIXEL_8BIT_M
#define PIXEL_8BIT_M(RENDER, POS)             \
  {                                           \
    RENDER[0] = pal[character[pos]];          \
  }

#undef PIXEL_4BIT_M
#define PIXEL_4BIT_M(RENDER, POS)             \
  {                                           \
    uint8_t pixel = character[pos >> 1];      \
                                              \
    if ( (pos & 1) != 0 )                     \
    {                                         \
      pixel >>= 4;                            \
    }                                         \
    pixel &= 15;                              \
                                              \
    RENDER[0] = pal[pixel];                   \
  }

#undef TRANSFORM_SP_RENDER
#define TRANSFORM_SP_RENDER(id, RENDER, CALLBACK_)                                                   \
  case (id):                                                                                         \
    {                                                                                                \
      static const uint32_t transform_x_and_mask = sp_transform<id, t_nFixed32Mask>::and_x;          \
      static const uint32_t transform_y_and_mask = sp_transform<id, t_nFixed32Mask>::and_y;          \
      static const uint32_t transform_size_x = sp_transform<id, t_nFixed32Mask>::size_x;             \
      static const uint32_t transform_size_y = sp_transform<id, t_nFixed32Mask>::size_y;             \
      static const uint32_t transform_combine_mask = sp_transform<id, t_nFixed32Mask>::combine_mask; \
                                                                                                     \
      transform_x = sp_transform<id, t_nFixed32Mask>::ref_x_convert (org_x);                         \
      transform_y = sp_transform<id, t_nFixed32Mask>::ref_y_convert (org_y);                         \
      transform_delta_x = sp_transform<id, t_nFixed32Mask>::delta_x_convert (delta_x);               \
      transform_delta_y = sp_transform<id, t_nFixed32Mask>::delta_y_convert (delta_y);               \
                                                                                                     \
      counts = sp_transform<id, t_nFixed32Mask>::div8h;                                              \
                                                                                                     \
      if ( (id & 16) != 0 )                                                                          \
      { /* 8bit palette */                                                                           \
        do                                                                                           \
        {                                                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
        } while (--counts != 0);                                                                     \
      }                                                                                              \
      else                                                                                           \
      { /* 4bit palette */                                                                           \
        uint32_t *pal = c.palette;                                                                   \
                                                                                                     \
        do                                                                                           \
        {                                                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
          DRAW_PIXEL (RENDER, CALLBACK_);                                                            \
        } while (--counts != 0);                                                                     \
      }                                                                                              \
    }                                                                                      

    for (uint32_t id = 127; static_cast<int32_t> (id) >= 0; id++ )
    {
      sprite::cell &c = sprite_.cells[id];

      if (c.win_mask != 0)
      {
        continue ;
      }
      else if (c.scale_mask != 0)
      { // scale sprite stuff..
        uint32_t cy = vcount - c.pos_y;

        if ( cy < c.gst_y && ( c.pos_x < 240 || c.pos_x > 0x7fffffffu ) )
        {
          if ( t_nVertMosaicMask != 0)
          {
            cy -= (cy % mosaic_sp_y) & c.mosaic_mask;
          }
          pri = c.pri | pri_;

          if ( c.ref_dx != c.transform[0]
           || c.ref_dmx != c.transform[1] )
          {
            c.ref_dx = c.transform[0];
            c.ref_dmx= c.transform[1];

            c.org_x = c.size_x << 7;
            c.org_x-= (c.gst_x >> 1) * c.ref_dx;
            c.org_x-= (c.gst_y >> 1) * c.ref_dmx;
          }

          if ( c.ref_dy != c.transform[2]
           || c.ref_dmy != c.transform[3] )
          {
            c.ref_dy = c.transform[2];
            c.ref_dmy= c.transform[3];

            c.org_y = c.size_y << 7;
            c.org_y-= (c.gst_x >> 1) * c.ref_dy;
            c.org_y-= (c.gst_y >> 1) * c.ref_dmy;
          }
          int32_t org_x = c.org_x;
          int32_t org_y = c.org_y;
          int32_t delta_x = c.ref_dx;
          int32_t delta_y = c.ref_dy;

          uint32_t transform_x;
          uint32_t transform_y;
          uint32_t transform_delta_x;
          uint32_t transform_delta_y;
          
          uint32_t counts;

          uint8_t *character = c.character;

          org_x += static_cast<int32_t> (cy) * c.ref_dmx;
          org_y += static_cast<int32_t> (cy) * c.ref_dmy;

          render = & dst[static_cast<int32_t> (c.pos_x)];

          if ( t_nHoriMosaicMask == 0 || c.mosaic_mask == 0)
          {
            switch (c.shape2)
            {
            TRANSFORM_SP_RENDER (0,  render, PIXEL_4BIT_N) continue ;/* 8*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (1,  render, PIXEL_4BIT_N) continue ;/* 16*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (2,  render, PIXEL_4BIT_N) continue ;/* 32*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (3,  render, PIXEL_4BIT_N) continue ;/* 64*64 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (4,  render, PIXEL_4BIT_N) continue ;/* 16*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (5,  render, PIXEL_4BIT_N) continue ;/* 32*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (6,  render, PIXEL_4BIT_N) continue ;/* 32*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (7,  render, PIXEL_4BIT_N) continue ;/* 64*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (8,  render, PIXEL_4BIT_N) continue ;/* 8*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (9,  render, PIXEL_4BIT_N) continue ;/* 8*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (10, render, PIXEL_4BIT_N) continue ;/* 16*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (11, render, PIXEL_4BIT_N) continue ;/* 32*64 (sstandard cale mode 4bit) */
            TRANSFORM_SP_RENDER (16, render, PIXEL_8BIT_N) continue ;/* 8*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (17, render, PIXEL_8BIT_N) continue ;/* 16*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (18, render, PIXEL_8BIT_N) continue ;/* 32*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (19, render, PIXEL_8BIT_N) continue ;/* 64*64 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (20, render, PIXEL_8BIT_N) continue ;/* 16*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (21, render, PIXEL_8BIT_N) continue ;/* 32*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (22, render, PIXEL_8BIT_N) continue ;/* 32*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (23, render, PIXEL_8BIT_N) continue ;/* 64*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (24, render, PIXEL_8BIT_N) continue ;/* 8*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (25, render, PIXEL_8BIT_N) continue ;/* 8*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (26, render, PIXEL_8BIT_N) continue ;/* 16*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (27, render, PIXEL_8BIT_N) continue ;/* 32*64 (sstandard cale mode 8bit) */

            case 12: /* unused */
            case 13: /* unused */
            case 14: /* unused */
            case 15: /* unused */
            case 28: /* unused */
            case 29: /* unused */
            case 30: /* unused */
            case 31: /* unused */
              break;
            }
          }
          else
          {
#undef DRAW_PIXEL
#define DRAW_PIXEL(RENDER, CALLBACK_)                       \
  transform_x &= transform_x_and_mask;                      \
  transform_y &= transform_y_and_mask;                      \
                                                            \
  if ( transform_x < transform_size_x                       \
   && transform_y < transform_size_y )                      \
  {                                                         \
    uint32_t transform_mixer = transform_x | transform_y;   \
    uint32_t transform_pos_ = transform_mixer >> 8;         \
    uint32_t pos = transform_pos_ & transform_combine_mask; \
                                                            \
    CALLBACK_ (RENDER, pos)                                 \
  }                                                         \
  else                                                      \
  {                                                         \
    RENDER[0] = 0xff000000;                                 \
  }                                                         \
  transform_x += transform_delta_x;                         \
  transform_y += transform_delta_y;                         \
                                                            \
  RENDER++; 
            render2 = mosaic;

            switch (c.shape2)
            {
            TRANSFORM_SP_RENDER (0,  render2, PIXEL_4BIT_M) break ;/* 8*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (1,  render2, PIXEL_4BIT_M) break ;/* 16*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (2,  render2, PIXEL_4BIT_M) break ;/* 32*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (3,  render2, PIXEL_4BIT_M) break ;/* 64*64 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (4,  render2, PIXEL_4BIT_M) break ;/* 16*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (5,  render2, PIXEL_4BIT_M) break ;/* 32*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (6,  render2, PIXEL_4BIT_M) break ;/* 32*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (7,  render2, PIXEL_4BIT_M) break ;/* 64*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (8,  render2, PIXEL_4BIT_M) break ;/* 8*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (9,  render2, PIXEL_4BIT_M) break ;/* 8*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (10, render2, PIXEL_4BIT_M) break ;/* 16*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (11, render2, PIXEL_4BIT_M) break ;/* 32*64 (sstandard cale mode 4bit) */
            TRANSFORM_SP_RENDER (16, render2, PIXEL_8BIT_M) break ;/* 8*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (17, render2, PIXEL_8BIT_M) break ;/* 16*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (18, render2, PIXEL_8BIT_M) break ;/* 32*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (19, render2, PIXEL_8BIT_M) break ;/* 64*64 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (20, render2, PIXEL_8BIT_M) break ;/* 16*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (21, render2, PIXEL_8BIT_M) break ;/* 32*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (22, render2, PIXEL_8BIT_M) break ;/* 32*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (23, render2, PIXEL_8BIT_M) break ;/* 64*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (24, render2, PIXEL_8BIT_M) break ;/* 8*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (25, render2, PIXEL_8BIT_M) break ;/* 8*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (26, render2, PIXEL_8BIT_M) break ;/* 16*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (27, render2, PIXEL_8BIT_M) break ;/* 32*64 (sstandard cale mode 8bit) */

            case 12: /* unused */
            case 13: /* unused */
            case 14: /* unused */
            case 15: /* unused */
            case 28: /* unused */
            case 29: /* unused */
            case 30: /* unused */
            case 31: /* unused */
              break;
            }

            uint32_t mosaic_= mosaic[0];
            uint32_t mosaicH= mosaic_sp_x;
            uint32_t mosaicCount = mosaicH;

            for ( uint32_t id = 0; id != c.gst_x; id++ )
            {
              if ( static_cast<int32_t> (mosaic_) >= 0 )
              {
                render[id] = mosaic_ | pri;
              }
            
              if ( --mosaicCount == 0 )
              {
                mosaic_ = mosaic[id + 1];
                mosaicCount = mosaicH;
              }
            }
          }
        }
      }
      else if (c.enable_mask != 0)
      { // standard sprite stuff..
        uint32_t cy = vcount - c.pos_y;

        if ( cy < c.size_y && ( c.pos_x < 240 || c.pos_x > 0x7fffffffu ) )
        { // draw oam sprite.
          cy ^= c.vflip_mask; // switch v-flip.

          if ( t_nVertMosaicMask != 0)
          {
            cy -= (cy % mosaic_sp_y) & c.mosaic_mask;
          }
          uint32_t YYY = cy >> 3;
          uint32_t yyy = cy & 7;

          uint32_t t;
          uint32_t pri = c.pri | pri_;

          uint32_t counts = c.size_x >> 3;

          render = & dst[static_cast<int32_t> (c.pos_x)];

          if ( t_nHoriMosaicMask == 0 || c.mosaic_mask == 0)
          {
            if ( c._8bit_mask != 0 )
            { // draw color 256 sprite.
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                            + (t_nFixed32Mask != 0 ) 
                          ? (YYY << 10)
                            : (YYY << c.tile_sft);
                         character+= yyy << 3;

                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  if ( (t = c & 255) != 0 ) { render[0] = pal[t] | pri; } c >>= 8;
                  if ( (t = c & 255) != 0 ) { render[1] = pal[t] | pri; } c >>= 8;
                  if ( (t = c & 255) != 0 ) { render[2] = pal[t] | pri; } c >>= 8;
                  if ( c != 0 )             { render[3] = pal[c] | pri; } 
                  if ( (t = c2& 255) != 0 ) { render[4] = pal[t] | pri; } c2>>= 8;
                  if ( (t = c2& 255) != 0 ) { render[5] = pal[t] | pri; } c2>>= 8;
                  if ( (t = c2& 255) != 0 ) { render[6] = pal[t] | pri; } c2>>= 8;
                  if ( c2 != 0 )            { render[7] = pal[c2]| pri; }
 
                  render += 8;
                  character += 64;
                } while (--counts != 0);
              }
              else
              { // hori filp.
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                         character+= yyy << 3;
                         character+= counts << 6;
                         character-= 8;
 
                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  if ( (t = c & 255) != 0 ) { render[7] = pal[t] | pri; } c >>= 8;
                  if ( (t = c & 255) != 0 ) { render[6] = pal[t] | pri; } c >>= 8;
                  if ( (t = c & 255) != 0 ) { render[5] = pal[t] | pri; } c >>= 8;
                  if ( c != 0 )             { render[4] = pal[c] | pri; } 
                  if ( (t = c2 &255) != 0 ) { render[3] = pal[t] | pri; } c2>>= 8;
                  if ( (t = c2 &255) != 0 ) { render[2] = pal[t] | pri; } c2>>= 8;
                  if ( (t = c2 &255) != 0 ) { render[1] = pal[t] | pri; } c2>>= 8;
                  if ( c2 != 0 )            { render[0] = pal[c2]| pri; }
 
                  render += 8;
                  character -= 64;
                } while (--counts != 0);
              }
            }
            else
            { // draw 4bit color + palette bank sprite.
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                        character+= yyy << 2;
                uint32_t *pal = c.palette;
     
                do
                {
                  uint32_t c = LOAD32x (character);

                  if ( (t = c & 15) != 0 ) { render[0] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[1] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[2] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[3] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[4] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[5] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[6] = pal[t] | pri; } c >>= 4;
                  if ( c != 0 )            { render[7] = pal[c] | pri; }

                  character += 32;
                  render += 8;
                } while (--counts != 0);
              }
              else
              { // hori filp.
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);                
                         character+= yyy << 2;
                         character+= counts << 5;
                         character-= 4;
                uint32_t *pal = c.palette;

                do
                {
                  uint32_t c = LOAD32x (character);

                  if ( (t = c & 15) != 0 ) { render[7] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[6] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[5] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[4] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[3] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[2] = pal[t] | pri; } c >>= 4;
                  if ( (t = c & 15) != 0 ) { render[1] = pal[t] | pri; } c >>= 4;
                  if ( c != 0 )            { render[0] = pal[c] | pri; }

                  character -= 32;
                  render += 8;
                } while (--counts != 0);
              }
            }
          }
          else
          { // mosaic
            render2 = mosaic;

            if ( c._8bit_mask != 0 )
            { // draw color 256 sprite (mosaic version)
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                            + (t_nFixed32Mask != 0 ) 
                          ? (YYY << 10)
                            : (YYY << c.tile_sft);
                          character+= yyy << 3;
                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  render2[0] = pal[c & 255]; c >>= 8;
                  render2[1] = pal[c & 255]; c >>= 8;
                  render2[2] = pal[c & 255]; c >>= 8;
                  render2[3] = pal[c]; 
                  render2[4] = pal[c2& 255]; c2>>= 8;
                  render2[5] = pal[c2& 255]; c2>>= 8;
                  render2[6] = pal[c2& 255]; c2>>= 8;
                  render2[7] = pal[c2];
 
                  render2 += 8;
                  character += 64;
                } while (--counts != 0);
              }
              else
              { // hori filp. (mosaic version)
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                         character+= yyy << 3;
                         character+= counts << 6;
                         character-= 8;
                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  render2[7] = pal[c & 255]; c >>= 8;
                  render2[6] = pal[c & 255]; c >>= 8;
                  render2[5] = pal[c & 255]; c >>= 8;
                  render2[4] = pal[c]; 
                  render2[3] = pal[c2& 255]; c2>>= 8;
                  render2[2] = pal[c2& 255]; c2>>= 8;
                  render2[1] = pal[c2& 255]; c2>>= 8;
                  render2[0] = pal[c2];
 
                  render2 += 8;
                  character -= 64;
                } while (--counts != 0);
              }
            }
            else
            { // draw 4bit color + palette bank sprite. (mosaic version)
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                         character+= yyy << 2;
                uint32_t *pal = c.palette;
 
                do
                {
                  uint32_t c = LOAD32x (character);

                  render2[0] = pal[c & 15]; c >>= 4;
                  render2[1] = pal[c & 15]; c >>= 4;
                  render2[2] = pal[c & 15]; c >>= 4;
                  render2[3] = pal[c & 15]; c >>= 4;
                  render2[4] = pal[c & 15]; c >>= 4;
                  render2[5] = pal[c & 15]; c >>= 4;
                  render2[6] = pal[c & 15]; c >>= 4;
                  render2[7] = pal[c]; 

                  character += 32;
                  render2 += 8;            
                } while (--counts != 0);
              }
              else
              { // hori filp. (mosaic version)
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);  
                         character+= yyy << 2;
                         character+= counts << 5;
                         character-= 4;
                uint32_t *pal = c.palette;

                do
                {
                  uint32_t c = LOAD32x (character);

                  render2[7] = pal[c & 15]; c >>= 4; // and, sib-index-read, sib-index-write, reg-shift.
                  render2[6] = pal[c & 15]; c >>= 4;
                  render2[5] = pal[c & 15]; c >>= 4;
                  render2[4] = pal[c & 15]; c >>= 4;
                  render2[3] = pal[c & 15]; c >>= 4;
                  render2[2] = pal[c & 15]; c >>= 4;
                  render2[1] = pal[c & 15]; c >>= 4;
                  render2[0] = pal[c];

                  character -= 32;
                  render2 += 8;             
                } while (--counts != 0);
              }
            }
          }

          uint32_t mosaic_ = mosaic[0];
          uint32_t mosaicH = mosaic_sp_x;
          uint32_t mosaicCount = mosaicH;

          for ( uint32_t id = 0; id != c.size_x; id++ )
          {
            if ( static_cast<int32_t> (mosaic_) >= 0 )
            {
              render[id] = mosaic_ | pri;
            }
            
            if ( --mosaicCount == 0 )
            {
              mosaic_ = mosaic[id + 1];
              mosaicCount = mosaicH;
            }
          }
        }
      }
    }
  }

  template <uintptr_t t_nFixed32Mask, uintptr_t t_nHoriMosaicMask, uintptr_t t_nVertMosaicMask>
  void sp_win_ (void)
  {
    uint32_t *dst = sprite_buffer;
    uint32_t *render; // render position write.
    uint32_t *render2;// render position write (mosaic)...

    uint32_t pri_ = sp_pri; // 1st, 2nd disable mask.
    uint32_t pri;

    uint32_t mosaic[128];

    clear_win_mask<1> (dst);

#undef DRAW_PIXEL
#define DRAW_PIXEL(RENDER, CALLBACK_)                       \
  transform_x &= transform_x_and_mask;                      \
  transform_y &= transform_y_and_mask;                      \
                                                            \
  if ( transform_x < transform_size_x                       \
   && transform_y < transform_size_y )                      \
  {                                                         \
    uint32_t transform_mixer = transform_x | transform_y;   \
    uint32_t transform_pos_ = transform_mixer >> 8;         \
    uint32_t pos = transform_pos_ & transform_combine_mask; \
                                                            \
    CALLBACK_ (RENDER, pos)                                 \
  }                                                         \
  transform_x += transform_delta_x;                         \
  transform_y += transform_delta_y;                         \
                                                            \
  RENDER++;                

#undef PIXEL_8BIT_N
#define PIXEL_8BIT_N(RENDER, POS)    \
  {                                  \
    uint8_t pixel = character[pos];  \
                                     \
    if ( pixel != 0 )                \
    {                                \
      RENDER[0] = 1;                 \
    }                                \
  }

#undef PIXEL_4BIT_N
#define PIXEL_4BIT_N(RENDER, POS)        \
  {                                      \
    uint8_t pixel = character[pos >> 1]; \
                                         \
    if ( (pos & 1) != 0 )                \
    {                                    \
      pixel >>= 4;                       \
    }                                    \
    pixel &= 15;                         \
                                         \
    if ( pixel != 0 )                    \
    {                                    \
      RENDER[0] = 1;                     \
    }                                    \
  }

#undef PIXEL_8BIT_M
#define PIXEL_8BIT_M(RENDER, POS)             \
  {                                           \
    RENDER[0] = character[pos];               \
  }

#undef PIXEL_4BIT_M
#define PIXEL_4BIT_M(RENDER, POS)             \
  {                                           \
    uint8_t pixel = character[pos >> 1];      \
                                              \
    if ( (pos & 1) != 0 )                     \
    {                                         \
      pixel >>= 4;                            \
    }                                         \
    RENDER[0] = pixel & 15;                   \
  }

#undef TRANSFORM_SP_RENDER
#define TRANSFORM_SP_RENDER(id, RENDER, CALLBACK_)                                                   \
  case (id):                                                                                         \
    {                                                                                                \
      static const uint32_t transform_x_and_mask = sp_transform<id, t_nFixed32Mask>::and_x;          \
      static const uint32_t transform_y_and_mask = sp_transform<id, t_nFixed32Mask>::and_y;          \
      static const uint32_t transform_size_x = sp_transform<id, t_nFixed32Mask>::size_x;             \
      static const uint32_t transform_size_y = sp_transform<id, t_nFixed32Mask>::size_y;             \
      static const uint32_t transform_combine_mask = sp_transform<id, t_nFixed32Mask>::combine_mask; \
                                                                                                     \
      transform_x = sp_transform<id, t_nFixed32Mask>::ref_x_convert (org_x);                         \
      transform_y = sp_transform<id, t_nFixed32Mask>::ref_y_convert (org_y);                         \
      transform_delta_x = sp_transform<id, t_nFixed32Mask>::delta_x_convert (delta_x);               \
      transform_delta_y = sp_transform<id, t_nFixed32Mask>::delta_y_convert (delta_y);               \
                                                                                                     \
      counts = sp_transform<id, t_nFixed32Mask>::div8h;                                              \
                                                                                                     \
      do                                                                                             \
      {                                                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
        DRAW_PIXEL (RENDER, CALLBACK_);                                                              \
      } while (--counts != 0);                                                                       \
    }                                                                                      

    for (uint32_t id = 127; static_cast<int32_t> (id) >= 0; id++ )
    {
      sprite::cell &c = sprite_.cells[id];

      if (c.win_mask == 0)
      {
        continue ;
      }
      else if (c.scale_mask != 0)
      { // scale sprite stuff..
        uint32_t cy = vcount - c.pos_y;

        if ( cy < c.gst_y && ( c.pos_x < 240 || c.pos_x > 0x7fffffffu ) )
        {
          if ( t_nVertMosaicMask != 0)
          {
            cy -= (cy % mosaic_sp_y) & c.mosaic_mask;
          }
          pri = c.pri | pri_;

          if ( c.ref_dx != c.transform[0]
           || c.ref_dmx != c.transform[1] )
          {
            c.ref_dx = c.transform[0];
            c.ref_dmx= c.transform[1];

            c.org_x = c.size_x << 7;
            c.org_x-= (c.gst_x >> 1) * c.ref_dx;
            c.org_x-= (c.gst_y >> 1) * c.ref_dmx;
          }

          if ( c.ref_dy != c.transform[2]
           || c.ref_dmy != c.transform[3] )
          {
            c.ref_dy = c.transform[2];
            c.ref_dmy= c.transform[3];

            c.org_y = c.size_y << 7;
            c.org_y-= (c.gst_x >> 1) * c.ref_dy;
            c.org_y-= (c.gst_y >> 1) * c.ref_dmy;
          }
          int32_t org_x = c.org_x;
          int32_t org_y = c.org_y;
          int32_t delta_x = c.ref_dx;
          int32_t delta_y = c.ref_dy;

          uint32_t transform_x;
          uint32_t transform_y;
          uint32_t transform_delta_x;
          uint32_t transform_delta_y;
          
          uint32_t counts;

          uint8_t *character = c.character;

          org_x += static_cast<int32_t> (cy) * c.ref_dmx;
          org_y += static_cast<int32_t> (cy) * c.ref_dmy;

          render = & dst[static_cast<int32_t> (c.pos_x)];

          if ( t_nHoriMosaicMask == 0 || c.mosaic_mask == 0)
          {
            switch (c.shape2)
            {
            TRANSFORM_SP_RENDER (0,  render, PIXEL_4BIT_N) continue ;/* 8*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (1,  render, PIXEL_4BIT_N) continue ;/* 16*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (2,  render, PIXEL_4BIT_N) continue ;/* 32*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (3,  render, PIXEL_4BIT_N) continue ;/* 64*64 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (4,  render, PIXEL_4BIT_N) continue ;/* 16*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (5,  render, PIXEL_4BIT_N) continue ;/* 32*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (6,  render, PIXEL_4BIT_N) continue ;/* 32*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (7,  render, PIXEL_4BIT_N) continue ;/* 64*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (8,  render, PIXEL_4BIT_N) continue ;/* 8*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (9,  render, PIXEL_4BIT_N) continue ;/* 8*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (10, render, PIXEL_4BIT_N) continue ;/* 16*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (11, render, PIXEL_4BIT_N) continue ;/* 32*64 (sstandard cale mode 4bit) */
            TRANSFORM_SP_RENDER (16, render, PIXEL_8BIT_N) continue ;/* 8*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (17, render, PIXEL_8BIT_N) continue ;/* 16*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (18, render, PIXEL_8BIT_N) continue ;/* 32*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (19, render, PIXEL_8BIT_N) continue ;/* 64*64 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (20, render, PIXEL_8BIT_N) continue ;/* 16*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (21, render, PIXEL_8BIT_N) continue ;/* 32*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (22, render, PIXEL_8BIT_N) continue ;/* 32*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (23, render, PIXEL_8BIT_N) continue ;/* 64*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (24, render, PIXEL_8BIT_N) continue ;/* 8*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (25, render, PIXEL_8BIT_N) continue ;/* 8*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (26, render, PIXEL_8BIT_N) continue ;/* 16*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (27, render, PIXEL_8BIT_N) continue ;/* 32*64 (sstandard cale mode 8bit) */

            case 12: /* unused */
            case 13: /* unused */
            case 14: /* unused */
            case 15: /* unused */
            case 28: /* unused */
            case 29: /* unused */
            case 30: /* unused */
            case 31: /* unused */
              break;
            }
          }
          else
          {
#undef DRAW_PIXEL
#define DRAW_PIXEL(RENDER, CALLBACK_)                       \
  transform_x &= transform_x_and_mask;                      \
  transform_y &= transform_y_and_mask;                      \
                                                            \
  if ( transform_x < transform_size_x                       \
   && transform_y < transform_size_y )                      \
  {                                                         \
    uint32_t transform_mixer = transform_x | transform_y;   \
    uint32_t transform_pos_ = transform_mixer >> 8;         \
    uint32_t pos = transform_pos_ & transform_combine_mask; \
                                                            \
    CALLBACK_ (RENDER, pos)                                 \
  }                                                         \
  else                                                      \
  {                                                         \
    RENDER[0] = 0;                                          \
  }                                                         \
  transform_x += transform_delta_x;                         \
  transform_y += transform_delta_y;                         \
                                                            \
  RENDER++; 
            render2 = mosaic;

            switch (c.shape2)
            {
            TRANSFORM_SP_RENDER (0,  render2, PIXEL_4BIT_M) break ;/* 8*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (1,  render2, PIXEL_4BIT_M) break ;/* 16*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (2,  render2, PIXEL_4BIT_M) break ;/* 32*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (3,  render2, PIXEL_4BIT_M) break ;/* 64*64 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (4,  render2, PIXEL_4BIT_M) break ;/* 16*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (5,  render2, PIXEL_4BIT_M) break ;/* 32*8 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (6,  render2, PIXEL_4BIT_M) break ;/* 32*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (7,  render2, PIXEL_4BIT_M) break ;/* 64*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (8,  render2, PIXEL_4BIT_M) break ;/* 8*16 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (9,  render2, PIXEL_4BIT_M) break ;/* 8*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (10, render2, PIXEL_4BIT_M) break ;/* 16*32 (standard scale mode 4bit) */
            TRANSFORM_SP_RENDER (11, render2, PIXEL_4BIT_M) break ;/* 32*64 (sstandard cale mode 4bit) */
            TRANSFORM_SP_RENDER (16, render2, PIXEL_8BIT_M) break ;/* 8*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (17, render2, PIXEL_8BIT_M) break ;/* 16*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (18, render2, PIXEL_8BIT_M) break ;/* 32*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (19, render2, PIXEL_8BIT_M) break ;/* 64*64 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (20, render2, PIXEL_8BIT_M) break ;/* 16*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (21, render2, PIXEL_8BIT_M) break ;/* 32*8 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (22, render2, PIXEL_8BIT_M) break ;/* 32*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (23, render2, PIXEL_8BIT_M) break ;/* 64*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (24, render2, PIXEL_8BIT_M) break ;/* 8*16 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (25, render2, PIXEL_8BIT_M) break ;/* 8*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (26, render2, PIXEL_8BIT_M) break ;/* 16*32 (standard scale mode 8bit) */
            TRANSFORM_SP_RENDER (27, render2, PIXEL_8BIT_M) break ;/* 32*64 (sstandard cale mode 8bit) */

            case 12: /* unused */
            case 13: /* unused */
            case 14: /* unused */
            case 15: /* unused */
            case 28: /* unused */
            case 29: /* unused */
            case 30: /* unused */
            case 31: /* unused */
              break;
            }
            uint32_t mosaic_= mosaic[0];
            uint32_t mosaicH= mosaic_sp_x;
            uint32_t mosaicCount = mosaicH;

            for ( uint32_t id = 0; id != c.gst_x; id++ )
            {
              if ( mosaic_ != 0 )
              {
                render[id] |= 1;
              }
            
              if ( --mosaicCount == 0 )
              {
                mosaic_ = mosaic[id + 1];
                mosaicCount = mosaicH;
              }
            }
          }
        }
      }
      else
      { // standard sprite stuff..
        uint32_t cy = vcount - c.pos_y;

        if ( cy < c.size_y && ( c.pos_x < 240 || c.pos_x > 0x7fffffffu ) )
        { // draw oam sprite.
          cy ^= c.vflip_mask; // switch v-flip.

          if ( t_nVertMosaicMask != 0)
          {
            cy -= (cy % mosaic_sp_y) & c.mosaic_mask;
          }
          uint32_t YYY = cy >> 3;
          uint32_t yyy = cy & 7;

          uint32_t counts = c.size_x >> 3;

          render = & dst[static_cast<int32_t> (c.pos_x)];

          if ( t_nHoriMosaicMask == 0 || c.mosaic_mask == 0)
          {
            if ( c._8bit_mask != 0 )
            { // draw color 256 sprite.
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                            + (t_nFixed32Mask != 0 ) 
                          ? (YYY << 10)
                            : (YYY << c.tile_sft);
                         character+= yyy << 3;

                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  if ( c2> 0xffffffu ) render[7] |= 1; c2<<= 8;
                  if ( c2> 0xffffffu ) render[6] |= 1; c2<<= 8;
                  if ( c2> 0xffffffu ) render[5] |= 1; c2<<= 8;
                  if ( c2> 0xffffffu ) render[4] |= 1; 
                  if ( c > 0xffffffu ) render[3] |= 1; c <<= 8;
                  if ( c > 0xffffffu ) render[2] |= 1; c <<= 8;
                  if ( c > 0xffffffu ) render[1] |= 1; c <<= 8;
                  if ( c > 0xffffffu ) render[0] |= 1; 

                  render += 8;
                  character += 64;
                } while (--counts != 0);
              }
              else
              { // hori filp.
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                         character+= yyy << 3;
                         character+= counts << 6;
                         character-= 8;
 
                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  if ( c2> 0xffffffu ) render[0] |= 1; c2<<= 8;
                  if ( c2> 0xffffffu ) render[1] |= 1; c2<<= 8;
                  if ( c2> 0xffffffu ) render[2] |= 1; c2<<= 8;
                  if ( c2> 0xffffffu ) render[3] |= 1; 
                  if ( c > 0xffffffu ) render[4] |= 1; c <<= 8;
                  if ( c > 0xffffffu ) render[5] |= 1; c <<= 8;
                  if ( c > 0xffffffu ) render[6] |= 1; c <<= 8;
                  if ( c > 0xffffffu ) render[7] |= 1; 
 
                  render += 8;
                  character -= 64;
                } while (--counts != 0);
              }
            }
            else
            { // draw 4bit color + palette bank sprite.
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                        character+= yyy << 2;
                do
                {
                  uint32_t c = LOAD32x (character);

                  if ( c > 0xfffffffu ) render[7] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[6] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[5] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[4] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[3] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[2] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[1] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[0] |= 1;

                  character += 32;
                  render += 8;
                } while (--counts != 0);
              }
              else
              { // hori filp.
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);                
                         character+= yyy << 2;
                         character+= counts << 5;
                         character-= 4;

                do
                {
                  uint32_t c = LOAD32x (character);

                  if ( c > 0xfffffffu ) render[0] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[1] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[2] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[3] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[4] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[5] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[6] |= 1; c <<= 4;
                  if ( c > 0xfffffffu ) render[7] |= 1;

                  character -= 32;
                  render += 8;
                } while (--counts != 0);
              }
            }
          }
          else
          { // mosaic
            render2 = mosaic;

            if ( c._8bit_mask != 0 )
            { // draw color 256 sprite (mosaic version)
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                            + (t_nFixed32Mask != 0 ) 
                          ? (YYY << 10)
                            : (YYY << c.tile_sft);
                          character+= yyy << 3;
                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  render2[0] = c & 255; c >>= 8;
                  render2[1] = c & 255; c >>= 8;
                  render2[2] = c & 255; c >>= 8;
                  render2[3] = c; 
                  render2[4] = c2& 255; c2>>= 8;
                  render2[5] = c2& 255; c2>>= 8;
                  render2[6] = c2& 255; c2>>= 8;
                  render2[7] = c2;
 
                  render2 += 8;
                  character += 64;
                } while (--counts != 0);
              }
              else
              { // hori filp. (mosaic version)
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                         character+= yyy << 3;
                         character+= counts << 6;
                         character-= 8;
                do
                {
                  uint32_t c = LOAD32x (character);
                  uint32_t c2= LOAD32x (character + 4);

                  render2[7] = c & 255; c >>= 8;
                  render2[6] = c & 255; c >>= 8;
                  render2[5] = c & 255; c >>= 8;
                  render2[4] = c; 
                  render2[3] = c2& 255; c2>>= 8;
                  render2[2] = c2& 255; c2>>= 8;
                  render2[1] = c2& 255; c2>>= 8;
                  render2[0] = c2;
 
                  render2 += 8;
                  character -= 64;
                } while (--counts != 0);
              }
            }
            else
            { // draw 4bit color + palette bank sprite. (mosaic version)
              if ( c.hflip_mask == 0 )
              { 
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);
                         character+= yyy << 2;
                do
                {
                  uint32_t c = LOAD32x (character);

                  render2[0] = c & 15; c >>= 4;
                  render2[1] = c & 15; c >>= 4;
                  render2[2] = c & 15; c >>= 4;
                  render2[3] = c & 15; c >>= 4;
                  render2[4] = c & 15; c >>= 4;
                  render2[5] = c & 15; c >>= 4;
                  render2[6] = c & 15; c >>= 4;
                  render2[7] = c; 

                  character += 32;
                  render2 += 8;            
                } while (--counts != 0);
              }
              else
              { // hori filp. (mosaic version)
                uint8_t *character = c.character 
                          + (t_nFixed32Mask != 0 ) 
                            ? (YYY << 10)
                           : (YYY << c.tile_sft);  
                         character+= yyy << 2;
                         character+= counts << 5;
                         character-= 4;

                do
                {
                  uint32_t c = LOAD32x (character);

                  render2[7] = c & 15; c >>= 4;
                  render2[6] = c & 15; c >>= 4;
                  render2[5] = c & 15; c >>= 4;
                  render2[4] = c & 15; c >>= 4;
                  render2[3] = c & 15; c >>= 4;
                  render2[2] = c & 15; c >>= 4;
                  render2[1] = c & 15; c >>= 4;
                  render2[0] = c; 

                  character -= 32;
                  render2 += 8;             
                } while (--counts != 0);
              }
            }
          }

          uint32_t mosaic_ = mosaic[0];
          uint32_t mosaicH = mosaic_sp_x;
          uint32_t mosaicCount = mosaicH;

          for ( uint32_t id = 0; id != c.size_x; id++ )
          {
            if ( mosaic_ != 0 )
            {
              render[id] |= 1;
            }
            
            if ( --mosaicCount == 0 )
            {
              mosaic_ = mosaic[id + 1];
              mosaicCount = mosaicH;
            }
          }
        }
      }
    }
  }

  

#ifdef VSP_VECTOR_PROCESS

#define SHUFFLE_TRANSPARENT_MASK 0x00 
#define SHUFFLE_EVAB_MASK 0xaa
#define SHUFFLE_EVYR_MASK 0x55
#define SHUFFLE_WIN_MASK 0xff
      
#define LSFT_OBJA 7
#define LSFT_2ND 6
#define LSFT_1ST 5

#undef CHANNEL_PIXEL_NEXT
#define CHANNEL_PIXEL_NEXT \
  if ( CHANNELS > 0 )      \
  {                        \
    channels[0] += 8;      \
  }                        \
                           \
  if ( CHANNELS > 1 )      \
  {                        \
    channels[1] += 8;      \
  }                        \
                           \
  if ( CHANNELS > 2 )      \
  {                        \
    channels[2] += 8;      \
  }                        \
                           \
  if ( CHANNELS > 3 )      \
  {                        \
    channels[3] += 8;      \
  }                        \
                           \
  if ( CHANNELS > 4 )      \
  {                        \
    channels[4] += 8;      \
  }

#undef CHANNEL_LOAD
#define CHANNEL_LOAD                          \
  if ( CHANNELS > 0 )                         \
  {                                           \
    ymm0x = _mm256_load_si256x (channels[0]); \
  }                                           \
                                              \
  if ( CHANNELS > 1 )                         \
  {                                           \
    ymm1x = _mm256_load_si256x (channels[1]); \
  }                                           \
                                              \
  if ( CHANNELS > 2 )                         \
  {                                           \
    ymm2x = _mm256_load_si256x (channels[2]); \
  }                                           \
                                              \
  if ( CHANNELS > 3 )                         \
  {                                           \
    ymm3x = _mm256_load_si256x (channels[3]); \
  }                                           \
                                              \
  if ( CHANNELS > 4 )                         \
  {                                           \
    ymm4x = _mm256_load_si256x (channels[4]); \
  }

#define BLEND_PIXEL_ /* ymm0x <= 1st, ymm1x <= 2nd (also output), ymm3x:alpha ymm4x <= temporary */  \
  ymm4x = _mm256_unpacklo_epi8 (ymm0x, ymm1x);                                                       \
  ymm1x = _mm256_unpackhi_epi8 (ymm0x, ymm1x);                                                       \
  ymm4x = _mm256_maddubs_epi16 (ymm4x, ymm3x);                                                       \
  ymm1x = _mm256_maddubs_epi16 (ymm1x, ymm3x);                                                       \
  ymm4x = _mm256_srli_epi16 (ymm4x, 4);                                                              \
  ymm1x = _mm256_srli_epi16 (ymm1x, 4);                                                              \
  ymm1x = _mm256_packus_epi16 (ymm4x, ymm1x);

#define BLEND_PIXEL /* ymm0x <= 1st, ymm1x <= 2nd (also output), ymm3x~ymm4x <= temporary */  \
  ymm3x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVAB_MASK);                                    \
  BLEND_PIXEL_

#define CHANNEL_PIXEL_NORMAL                                                                      \
                                                                                                                     \
  if ( t_nSpriteMask != 0 )                                                                                          \
  {                                                                                                                  \
    ymm2x = _mm256_slli_epi32 (ymm0x, LSFT_OBJA);  /* @ymm2x <= obj-blend mask to msb */                             \
    ymm2x = _mm256_srai_epi32 (ymm2x, 31);         /* @ymm2x <= obj-blend mask */                                    \
                                                                                                                     \
    if (_mm256_testz_si256 (ymm2x, ymm2x) == 0)                                                                      \
    {                                                                                                                \
      ymm4x = _mm256_slli_epi32 (ymm1x, LSFT_2ND); /* @ymm4x <= 2nd mask to msb */                                   \
      ymm2x = _mm256_andnot_si256 (ymm4x, ymm2x);  /* @ymm2x <= (obj-sprite && obj-alpha && 2nd set) */              \
      ymm2x = _mm256_srai_epi32 (ymm2x, 31);       /* @ymm2x <= (obj-sprite && obj-alpha && 2nd set) */              \
                                                                                                                     \
      BLEND_PIXEL                                                                                                    \
                                                                                                                     \
      ymm0x = _mm256_blendv_epi8 (ymm0x, ymm1x, ymm2x);                                                              \
    }                                                                                                                \
  }                                                                                                                  \
  _mm256_storeu_si256 ( reinterpret_cast<__m256i *> (dst), ymm0x ); /* @ymm0x:1st target pixel */                    \
                                                                                                                     \
  CHANNEL_PIXEL_NEXT                                                                                                    

#define CHANNEL_PIXEL_ALPHA                                                                           \
                                                                                                                       \
  if ( t_nSpriteMask != 0 )                                                                                            \
  {                                                                                                                    \
    ymm2x = _mm256_slli_epi32 (ymm0x, LSFT_OBJA) /* @ymm2x <= obj-blend mask to msb */                                 \
    ymm3x = _mm256_slli_epi32 (ymm0x, LSFT_1ST); /* @ymm3x <= 1st mask to msb  */                                      \
    ymm4x = _mm256_slli_epi32 (ymm1x, LSFT_2ND); /* @ymm4x <= 2nd mask to msb   */                                     \
    ymm2x = _mm256_andnot_si256 (ymm2x, ymm3x);  /* @ymm2x <= (obj-sprite && obj-alpha) ? 1st always set */            \
    ymm3x = _mm256_cmpeq_epi32 (ymm3x, ymm3x);   /* @ymm3x <= all one  */                                              \
    ymm2x = _mm256_or_si256 (ymm2x, ymm4x);      /* @ymm2x <= == 0 ? ( 1st && 2nd ) : ....  */                         \
    ymm2x = _mm256_srai_epi32 (ymm2x, 31);       /* @ymm2x <= 1 ? sel source : blend ...  */                           \
                                                                                                                       \
    if ( _mm256_testc_si256 ( ymm2x, ymm3x ) == 0 )                                                                    \
    {                                                                                                                  \
      BLEND_PIXEL                                                                                                      \
                                                                                                                       \
      ymm0x = _mm256_blendv_epi8 (ymm1x, ymm0x, ymm2x); /* @ymm0x <= blend select alpha-blend pixel or source pixel */ \
    }                                                                                                                  \
  }                                                                                                                    \
  else                                                                                                                 \
  {                                                                                                                    \
    ymm3x = _mm256_slli_epi32 (ymm0x, LSFT_1ST); /* @ymm3x <= 1st mask to msb  */                                      \
    ymm4x = _mm256_slli_epi32 (ymm1x, LSFT_2ND); /* @ymm4x <= 2nd mask to msb   */                                     \
    ymm3x = _mm256_cmpeq_epi32 (ymm3x, ymm3x);   /* @ymm3x <= all one  */                                              \
    ymm2x = _mm256_or_si256 (ymm2x, ymm4x);      /* @ymm2x <= == 0 ? ( 1st && 2nd ) : ....  */                         \
    ymm2x = _mm256_srai_epi32 (ymm2x, 31);       /* @ymm2x <= 1 ? sel source : blend ...  */                           \
                                                                                                                       \
    if ( _mm256_testc_si256 ( ymm2x, ymm3x ) == 0 )                                                                    \
    {                                                                                                                  \
      BLEND_PIXEL                                                                                                      \
                                                                                                                       \
      ymm0x = _mm256_blendv_epi8 (ymm1x, ymm0x, ymm2x); /* @ymm0x <= blend select alpha-blend pixel or source pixel */ \
    }                                                                                                                  \
  }                                                                                                                    \
  _mm256_store_si256_ux ( dst, ymm0x ); /* @ymm0x:1st target pixel  */                     \
                                                                                                                       \
  CHANNEL_PIXEL_NEXT                                                                                                      

#define CHANNEL_PIXEL_BRIGHTNESS_INC                                                                  \
                                                                                                                       \
  if ( t_nSpriteMask != 0 )                                                                                            \
  {                                                                                                                    \
    ymm3x = _mm256_slli_epi32 (ymm0x, LSFT_OBJA); /* @ymm3x <= obj-blend mask to msb */                                \
    ymm4x = _mm256_slli_epi32 (ymm0x, LSFT_1ST); /* @ymm4x <= 1st target effect mask to msb */                         \
    ymm5x = _mm256_slli_epi32 (ymm1x, LSFT_2ND); /* @ymm5x <= 2nd target effect mask to msb */                         \
    ymm5x = _mm256_andnot_si256 (ymm5x, ymm3x);  /* @ymm5x <= 1 (2nd sel && obj-blend set) */                          \
    ymm3x = _mm256_or_si256 (ymm4x, ymm5x);      /* @ymm3x <= 0 (no alpha && 1st set) */                               \
    ymm4x = _mm256_srai_epi32 (ymm3x, 31);       /* @ymm4x <= 0 (no alpha && 1st set)  */                              \
    ymm2x = _mm256_srai_epi32 (ymm5x, 31);       /* @ymm2x <= 1 (2nd sel && obj-blend set) */                          \
    ymm5x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVYR_MASK);                                                           \
    ymm3x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVAB_MASK);                                                           \
    ymm3x = _mm256_blendv_epi8 (ymm5x, ymm3x, ymm2x);                                                                  \
    ymm5x = _mm256_cmpeq_epi32 (ymm2x, ymm2x);                                                                         \
    ymm1x = _mm256_blendv_epi8 (ymm5x, ymm1x, ymm2x);                                                                  \
    ymm2x = _mm256_andnot_si256 (ymm2x, ymm4x);               /* @ymm2x <= 1 ? normal : effect */                      \
                                                                                                                       \
    if ( _mm256_testc_si256 ( ymm2x, ymm5x ) == 0 )                                                                    \
    {                                                                                                                  \
      BLEND_PIXEL_                                                                                                     \
                                                                                                                       \
      ymm0x = _mm256_blendv_epi8 (ymm1x, ymm0x, ymm2x); /* @ymm0x <= blend select alpha-blend pixel or source pixel */ \
    }                                                                                                                  \
  }                                                                                                                    \
  else                                                                                                                 \
  {                                                                                                                    \
    ymm2x = _mm256_slli_epi32 (ymm0x, LSFT_1ST); /* @ymm4x <= 1st target effect mask to msb */                         \
    ymm2x = _mm256_srai_epi32 (ymm2x, 31);                     /* @ymm4x <= 0 (no alpha && 1st set)  */                \
    ymm1x = _mm256_cmpeq_epi32 (ymm1x, ymm1x);                                                                         \
    ymm3x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVYR_MASK);                                                           \
                                                                                                                       \
    BLEND_PIXEL_                                                                                                       \
                                                                                                                       \
    ymm0x = _mm256_blendv_epi8 (ymm1x, ymm0x, ymm2x); /* @ymm0x <= blend select alpha-blend pixel or source pixel */   \
  }                                                                                                                    \
  _mm256_storeu_si256x ( dst, ymm0x ); /* @ymm0x <= 1st target pixel */                  \
                                                                                                                       \
  CHANNEL_PIXEL_NEXT                                                                                                   
                                                                                                                       
#define CHANNEL_PIXEL_BRIGHTNESS_DEC                                                                  \
                                                                                                                       \
  if (t_nSpriteMask != 0 )                                                                                             \
  {                                                                                                                    \
    ymm3x = _mm256_slli_epi32 (ymm0x, LSFT_OBJA);  /* @ymm3x <= obj-blend mask to msb */                               \
    ymm4x = _mm256_slli_epi32 (ymm0x, LSFT_1ST);   /* @ymm4x <= 1st target effect mask to msb */                       \
    ymm5x = _mm256_slli_epi32 (ymm1x, LSFT_2ND);   /* @ymm5x <= 2nd target effect mask to msb */                       \
    ymm5x = _mm256_andnot_si256 (ymm5x, ymm3x);    /* @ymm5x <= 1 (2nd sel && obj-blend set) */                        \
    ymm3x = _mm256_or_si256 (ymm4x, ymm5x);        /* @ymm3x <= 0 (no alpha && 1st set) */                             \
    ymm4x = _mm256_srai_epi32 (ymm3x, 31);         /* @ymm4x <= 0 (no alpha && 1st set) */                             \
    ymm2x = _mm256_srai_epi32 (ymm5x, 31);         /* @ymm2x <= 1 (2nd sel && obj-blend set) */                        \
    ymm5x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVYR_MASK);                                                           \
    ymm3x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVAB_MASK);                                                           \
    ymm3x = _mm256_blendv_epi8 (ymm5x, ymm3x, ymm2x);                                                                  \
    ymm5x = _mm256_cmpeq_epi32 (ymm2x, ymm2x);                                                                         \
    ymm2x = _mm256_andnot_si256 (ymm2x, ymm4x);    /* @ymm2x <= 1 ? normal : effect */                                 \
                                                                                                                       \
    if ( _mm256_testc_si256 ( ymm2x, ymm2x ) == 0 )                                                                    \
    {                                                                                                                  \
      BLEND_PIXEL_                                                                                                     \
                                                                                                                       \
      ymm0x = _mm256_blendv_epi8 (ymm1x, ymm0x, ymm2x); /* @ymm0x <= blend select alpha-blend pixel or source pixel */ \
    }                                                                                                                  \
  }                                                                                                                    \
  else                                                                                                                 \
  {                                                                                                                    \
    ymm2x = _mm256_slli_epi32 (ymm0x, LSFT_1ST); /* @ymm4x <= 1st target effect mask to msb */                         \
    ymm2x = _mm256_srai_epi32 (ymm2x, 31);                     /* @ymm2x <= 0 (no alpha && 1st set) */                 \
    ymm3x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_EVYR_MASK);                                                           \
                                                                                                                       \
    BLEND_PIXEL_                                                                                                       \
                                                                                                                       \
    ymm0x = _mm256_blendv_epi8 (ymm1x, ymm0x, ymm2x); /* @ymm0x <= blend select alpha-blend pixel or source pixel */   \
  }                                                                                                                    \
  _mm256_storeu_si256 ( reinterpret_cast<__m256i *> (dst), ymm0x ); /* @ymm0x <= 1st target pixel */                   \
                                                                                                                       \
  CHANNEL_PIXEL_NEXT                                                                                                

#undef CHANNEL5_LOAD_SORT                                         
#define CHANNEL5_LOAD_SORT                                                  \
  ymm0x = _mm256_load_si256x (channels[0]);                                 \
  ymm1x = _mm256_load_si256x (channels[1]);                                 \
  ymm2x = _mm256_load_si256x (channels[2]);                                 \
  ymm3x = _mm256_load_si256x (channels[3]);                                 \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop    */ \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)       */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= min (2, 3)       */ \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm4x); /* @ymm1x <= min (0, 1, 2, 3) */ \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm6x); /* @ymm1x <= min (0, 1, 2, 3) */ \
  ymm0x = _mm256_load_si256x (channels[4]);                                 \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm0x <= 1st */               

#undef CHANNEL5_LOAD_SORT_ALPHA                                         
#define CHANNEL5_LOAD_SORT_ALPHA                                               \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_load_si256x (channels[3]);                                    \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop       */ \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)          */ \
  ymm0x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm0x <= max (0, 1)          */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= min (2, 3)          */ \
  ymm3x = _mm256_max_epu32 (ymm2x, ymm3x); /* @ymm3x <= max (2, 3)          */ \
  ymm2x = _mm256_max_epu32 (ymm1x, ymm4x); /* @ymm2x <= =>                  */ \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm4x); /* @ymm1x <= min (0, 1, 2, 3)    */ \
  ymm3x = _mm256_min_epu32 (ymm0x, ymm3x); /* @ymm3x <= <=                  */ \
  ymm3x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm3x <= min2nd (0, 1, 2, 3) */ \
  ymm0x = _mm256_load_si256x (channels[4]);                                    \
  ymm4x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm4x <= => */                  \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm0x <= 1st */                 \
  ymm1x = _mm256_min_epu32 (ymm4x, ymm3x); /* @ymm1x <= 2nd */                 

#undef CHANNEL5_LOAD_CLIP_SORT  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL5_LOAD_CLIP_SORT /* c000dddd0000aaaaeeeE0000bbbb0000 */         \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_slli_epi32 (ymm5x, 20);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_or_si256 (ymm1x, ymm4x);                                      \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= cccc */                 \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm5x = _mm256_srai_epi16 (ymm5x, 8);    /* ymm5x <= dddd eeee to pos */     \
  ymm3x = _mm256_load_si256x (channels[3]);                                    \
  ymm4x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm4x <= dddd */                 \
  ymm3x = _mm256_or_si256 (ymm3x, ymm4x);                                      \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)          */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= min (2, 3)          */ \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm4x); /* @ymm1x <= min (0, 1, 2, 3)    */ \
  ymm0x = _mm256_load_si256x (channels[4]);                                    \
  ymm2x = _mm256_slli_epi32 (ymm5x, 22);   /* @ymm2x <= eeeeeE              */ \
  ymm0x = _mm256_or_si256 (ymm0x, ymm2x);  /* @ymm0x <= channel 4 */           \
  ymm5x = _mm256_slli_epi32 (ymm5x, 27);   /* @ymm5x <= win 1st mask to msb */ \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm0x <= 1st */                 \
  ymm5x = _mm256_srli_epi32 (ymm5x, 5);                                        \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

#undef CHANNEL5_LOAD_CLIP_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL5_LOAD_CLIP_SORT_ALPHA /* c000dddd0000aaaaeeeE0000bbbb0000 */   \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_slli_epi32 (ymm5x, 20);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_or_si256 (ymm1x, ymm4x);                                      \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= cccc */                 \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm5x = _mm256_srai_epi16 (ymm5x, 8);    /* ymm5x <= dddd eeee to pos */     \
  ymm3x = _mm256_load_si256x (channels[3]);                                    \
  ymm4x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm4x <= dddd */                 \
  ymm3x = _mm256_or_si256 (ymm3x, ymm4x);                                      \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)          */ \
  ymm0x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm0x <= max (0, 1)          */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= min (2, 3)          */ \
  ymm3x = _mm256_max_epu32 (ymm2x, ymm3x); /* @ymm3x <= max (2, 3)          */ \
  ymm2x = _mm256_max_epu32 (ymm1x, ymm4x); /* @ymm2x <= =>                  */ \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm4x); /* @ymm1x <= min (0, 1, 2, 3)    */ \
  ymm3x = _mm256_min_epu32 (ymm0x, ymm3x); /* @ymm3x <= <=                  */ \
  ymm3x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm3x <= min2nd (0, 1, 2, 3) */ \
  ymm0x = _mm256_load_si256x (channels[4]);                                    \
  ymm2x = _mm256_slli_epi32 (ymm5x, 22);   /* @ymm2x <= eeeeeE              */ \
  ymm0x = _mm256_or_si256 (ymm0x, ymm2x);  /* @ymm0x <= channel 4 */           \
  ymm5x = _mm256_slli_epi32 (ymm5x, 27);   /* @ymm5x <= win 1st mask to msb */ \
  ymm4x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm4x <= => */                  \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm0x <= 1st */                 \
  ymm5x = _mm256_srli_epi32 (ymm5x, 5);                                        \
  ymm1x = _mm256_min_epu32 (ymm4x, ymm3x); /* @ymm1x <= 2nd */                 \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

// five channel mixing
#undef CHANNELS 
#define CHANNELS 5
#undef CHANNEL_LOAD_SORT
#define CHANNEL_LOAD_SORT CHANNEL5_LOAD_SORT
#undef CHANNEL_LOAD_SORT_ALPHA
#define CHANNEL_LOAD_SORT_ALPHA CHANNEL5_LOAD_SORT_ALPHA 
#undef CHANNEL_LOAD_CLIP_SORT
#define CHANNEL_LOAD_CLIP_SORT CHANNEL5_LOAD_CLIP_SORT 
#undef CHANNEL_LOAD_CLIP_SORT_ALPHA
#define CHANNEL_LOAD_CLIP_SORT_ALPHA CHANNEL5_LOAD_CLIP_SORT_ALPHA 
#undef COMPOSITE
#define COMPOSITE composite5x

#include "gpu.composite_fast.inl"

#undef CHANNEL4_LOAD_SORT                                          
#define CHANNEL4_LOAD_SORT                                               \
  ymm0x = _mm256_load_si256x (channels[0]);                              \
  ymm1x = _mm256_load_si256x (channels[1]);                              \
  ymm2x = _mm256_load_si256x (channels[2]);                              \
  ymm3x = _mm256_load_si256x (channels[3]);                              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop */ \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)    */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm4x); /* @ymm1x <= 2nd           */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm3x); /* @ymm0x <= 1st           */  

#undef CHANNEL4_LOAD_SORT_ALPHA                                         
#define CHANNEL4_LOAD_SORT_ALPHA                                         \
  ymm0x = _mm256_load_si256x (channels[0]);                              \
  ymm1x = _mm256_load_si256x (channels[1]);                              \
  ymm2x = _mm256_load_si256x (channels[2]);                              \
  ymm3x = _mm256_load_si256x (channels[3]);                              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop */ \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)    */ \
  ymm0x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm0x <= max (0, 1)    */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= min (2, 3)    */ \
  ymm3x = _mm256_max_epu32 (ymm2x, ymm3x); /* @ymm3x <= max (2, 3)    */ \
  ymm3x = _mm256_min_epu32 (ymm0x, ymm3x); /* @ymm3x <= <=            */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm4x); /* @ymm0x <= 1st           */ \
  ymm2x = _mm256_max_epu32 (ymm1x, ymm4x); /* @ymm2x <= =>            */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= 2nd           */ 

#undef CHANNEL4_LOAD_CLIP_SORT  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL4_LOAD_CLIP_SORT /* c000dddd0000aaaa000E0000bbbb0000 */         \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_slli_epi32 (ymm5x, 20);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_or_si256 (ymm1x, ymm4x);                                      \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= cccc */                 \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm5x = _mm256_srai_epi16 (ymm5x, 8);    /* ymm5x <= dddd eeee to pos */     \
  ymm3x = _mm256_load_si256x (channels[3]);                                    \
  ymm4x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm4x <= dddd */                 \
  ymm3x = _mm256_or_si256 (ymm3x, ymm4x);                                      \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)          */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm4x); /* @ymm1x <= 2nd                 */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm3x); /* @ymm0x <= 1st                 */ \
  ymm5x = _mm256_slli_epi32 (ymm5x, 27);   /* @ymm5x <= win 1st mask to msb */ \
  ymm5x = _mm256_srli_epi32 (ymm5x, 5);                                        \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

#undef CHANNEL4_LOAD_CLIP_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL4_LOAD_CLIP_SORT_ALPHA /* c000dddd0000aaaa000E0000bbbb0000 */   \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_slli_epi32 (ymm5x, 20);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_or_si256 (ymm1x, ymm4x);                                      \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= cccc */                 \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm5x = _mm256_srai_epi16 (ymm5x, 8);    /* ymm5x <= dddd eeee to pos */     \
  ymm3x = _mm256_load_si256x (channels[3]);                                    \
  ymm4x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm4x <= dddd */                 \
  ymm3x = _mm256_or_si256 (ymm3x, ymm4x);                                      \
  ymm4x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm4x <= min (0, 1)          */ \
  ymm0x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm0x <= max (0, 1)          */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= min (2, 3)          */ \
  ymm3x = _mm256_max_epu32 (ymm2x, ymm3x); /* @ymm3x <= max (2, 3)          */ \
  ymm3x = _mm256_min_epu32 (ymm0x, ymm3x); /* @ymm3x <= <=                  */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm4x); /* @ymm0x <= 1st                 */ \
  ymm2x = _mm256_max_epu32 (ymm1x, ymm4x); /* @ymm2x <= =>                  */ \
  ymm1x = _mm256_min_epu32 (ymm2x, ymm3x); /* @ymm1x <= 2nd                 */ \
  ymm5x = _mm256_slli_epi32 (ymm5x, 27);   /* @ymm5x <= win 1st mask to msb */ \
  ymm5x = _mm256_srli_epi32 (ymm5x, 5);                                        \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

// four channel mixing
#undef CHANNELS 
#define CHANNELS 4
#undef CHANNEL_LOAD_SORT
#define CHANNEL_LOAD_SORT CHANNEL4_LOAD_SORT
#undef CHANNEL_LOAD_SORT_ALPHA
#define CHANNEL_LOAD_SORT_ALPHA CHANNEL4_LOAD_SORT_ALPHA 
#undef CHANNEL_LOAD_CLIP_SORT
#define CHANNEL_LOAD_CLIP_SORT CHANNEL4_LOAD_CLIP_SORT 
#undef CHANNEL_LOAD_CLIP_SORT_ALPHA
#define CHANNEL_LOAD_CLIP_SORT_ALPHA CHANNEL4_LOAD_CLIP_SORT_ALPHA 
#undef COMPOSITE
#define COMPOSITE composite4x

#include "gpu.composite_fast.inl"

#undef CHANNEL3_LOAD_SORT  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL3_LOAD_SORT /* c00000000000aaaa000E0000bbbb0000 */        \
  ymm0x = _mm256_load_si256x (channels[0]);                              \
  ymm1x = _mm256_load_si256x (channels[1]);                              \
  ymm2x = _mm256_load_si256x (channels[2]);                              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop */ \
  ymm1x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm1x <= 2nd */           \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm0x <= 1st */                              

#undef CHANNEL3_LOAD_SORT_ALPHA  /* fedcba9876543210fedcba9876543210  */
#define CHANNEL3_LOAD_SORT_ALPHA /* c00000000000aaaa000E0000bbbb0000  */ \
  ymm0x = _mm256_load_si256x (channels[0]);                              \
  ymm1x = _mm256_load_si256x (channels[1]);                              \
  ymm2x = _mm256_load_si256x (channels[2]);                              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop */ \
  ymm3x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm3x <= max (0, 1)    */ \
  ymm1x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm1x <= min (0, 1)    */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm0x <= 1st           */ \
  ymm1x = _mm256_max_epu32 (ymm1x, ymm2x); /* @ymm1x <= >=            */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm3x); /* @ymm0x <= 2nd           */ 
  
#undef CHANNEL3_LOAD_CLIP_SORT  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL3_LOAD_CLIP_SORT /* c00000000000aaaa000E0000bbbb0000 */         \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_slli_epi32 (ymm5x, 20);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_or_si256 (ymm1x, ymm4x);                                      \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= cccc */                 \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm1x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm1x <= 2nd */                 \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm0x <= 1st */                 \
  ymm5x = _mm256_slli_epi32 (ymm5x, 27);   /* @ymm5x <= win 1st mask to msb */ \
  ymm5x = _mm256_srli_epi32 (ymm5x, 5);                                        \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

#undef CHANNEL3_LOAD_CLIP_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL3_LOAD_CLIP_SORT_ALPHA /* c00000000000aaaa000E0000bbbb0000 */   \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm1x = _mm256_load_si256x (channels[1]);                                    \
  ymm2x = _mm256_load_si256x (channels[2]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_slli_epi32 (ymm5x, 20);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_or_si256 (ymm1x, ymm4x);                                      \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= cccc */                 \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm3x = _mm256_max_epu32 (ymm0x, ymm1x); /* @ymm3x <= max (0, 1)          */ \
  ymm1x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm1x <= min (0, 1)          */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm0x <= 1st                 */ \
  ymm1x = _mm256_max_epu32 (ymm1x, ymm2x); /* @ymm1x <= >=                  */ \
  ymm0x = _mm256_min_epu32 (ymm1x, ymm3x); /* @ymm0x <= 2nd                 */ \
  ymm5x = _mm256_slli_epi32 (ymm5x, 27);   /* @ymm5x <= win 1st mask to msb */ \
  ymm5x = _mm256_srli_epi32 (ymm5x, 5);                                        \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

// three channel mixing
#undef CHANNELS 
#define CHANNELS 3
#undef CHANNEL_SORT
#define CHANNEL_SORT CHANNEL3_SORT 
#undef CHANNEL_SORT2
#define CHANNEL_SORT2 CHANNEL3_SORT2 
#undef CHANNEL_LOAD_CLIP
#define CHANNEL_LOAD_CLIP CHANNEL3_LOAD_CLIP 
#undef COMPOSITE
#define COMPOSITE composite3x

#include "gpu.composite_fast.inl"

#undef CHANNEL2_LOAD_SORT 
#define CHANNEL2_LOAD_SORT                                               \
  ymm1x = _mm256_load_si256x (channels[0]);                              \
  ymm0x = _mm256_load_si256x (channels[1]);                              \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm6x); /* @ymm1x <= 1 or backdrop */ \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm0x <= 1st */   

#undef CHANNEL2_LOAD_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL2_LOAD_SORT_ALPHA /* b00000000000aaaa00000E0000000000 */   \
  ymm0x = _mm256_load_si256x (channels[0]);                               \
  ymm2x = _mm256_load_si256x (channels[1]);                               \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm6x); /* @ymm0x <= 0 or backdrop */  \
  ymm1x = _mm256_max_epu32 (ymm0x, ymm2x); /* @ymm1x <= 2nd           */  \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x); /* @ymm0x <= 1st           */ 

#undef CHANNEL2_LOAD_CLIP_SORT  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL2_LOAD_CLIP_SORT /* b00000000000aaaa00000E0000000000 */         \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm1x = _mm256_load_si256x (channels[0]);                                    \
  ymm0x = _mm256_load_si256x (channels[1]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= bbbb */                 \
  ymm1x = _mm256_or_si256 (ymm1x, ymm3x);                                      \
  ymm0x = _mm256_or_si256 (ymm0x, ymm4x);                                      \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x); /* @ymm0x <= 1st                 */ \
  ymm5x = _mm256_slli_epi32 (ymm5x, 16);   /* @ymm5x <= win 1st mask to pos */ \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */
  
#undef CHANNEL2_LOAD_CLIP_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL2_LOAD_CLIP_SORT_ALPHA /* b00000000000aaaa00000E0000000000 */   \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm2x = _mm256_load_si256x (channels[1]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm4x = _mm256_srai_epi32 (ymm5x, 31);   /* ymm4x <= bbbb */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm2x = _mm256_or_si256 (ymm2x, ymm4x);                                      \
  ymm1x = _mm256_max_epu32 (ymm0x, ymm2x); /* @ymm1x <= 2nd                 */ \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x); /* @ymm0x <= 1st                 */ \
  ymm5x = _mm256_slli_epi32 (ymm5x, 16);   /* @ymm5x <= win 1st mask to pos */ \
  ymm2x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

// second channel mixing
#undef CHANNELS 
#define CHANNELS 2
#undef CHANNEL_SORT
#define CHANNEL_SORT CHANNEL2_SORT 
#undef CHANNEL_SORT2
#define CHANNEL_SORT2 CHANNEL2_SORT2 
#undef CHANNEL_LOAD_CLIP
#define CHANNEL_LOAD_CLIP CHANNEL2_LOAD_CLIP 
#undef COMPOSITE
#define COMPOSITE composite2x

#include "gpu.composite_fast.inl"

#undef CHANNEL1_LOAD_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL1_LOAD_SORT_ALPHA /* 000000000000aaaa00000E0000000000 */ \
  ymm0x = _mm256_load_si256x (channels[0]);                             \
  ymm1x = ymm6x;                                                        \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm1x);

#undef CHANNEL1_LOAD_CLIP_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL1_LOAD_CLIP_SORT_ALPHA /* 000000000000aaaa00000E0000000000 */   \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_load_si256x (channels[0]);                                    \
  ymm3x = _mm256_slli_epi16 (ymm5x, 12);   /* ymm3x <= aaaa */                 \
  ymm0x = _mm256_or_si256 (ymm0x, ymm3x);                                      \
  ymm1x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_TRANSPARENT_MASK);              \
  ymm2x = ymm1x;                                                               \
  ymm5x = _mm256_slli_epi32 (ymm5x, 16);   /* @ymm5x <= win 1st mask to pos */ \
  ymm0x = _mm256_min_epu32 (ymm0x, ymm2x);                                     \
  ymm0x = _mm256_or_si256 (ymm0x, ymm5x);                                      \
  ymm1x = _mm256_min_epu32 (ymm1x, ymm2x); /* @ymm1x <= backdrop (if is.) */

#undef CHANNEL1_LOAD_CLIP_SORT  
#define CHANNEL1_LOAD_CLIP_SORT CHANNEL1_LOAD_CLIP_SORT_ALPHA
#undef CHANNEL1_LOAD_SORT  
#define CHANNEL1_LOAD_SORT CHANNEL1_LOAD_SORT_ALPHA

// one channel mixing
#undef CHANNELS 
#define CHANNELS 1
#undef CHANNEL_SORT
#define CHANNEL_SORT CHANNEL1_SORT 
#undef CHANNEL_SORT2
#define CHANNEL_SORT2 CHANNEL1_SORT2 
#undef CHANNEL_LOAD_CLIP
#define CHANNEL_LOAD_CLIP CHANNEL1_LOAD_CLIP 
#undef COMPOSITE
#define COMPOSITE composite1x

#include "gpu.composite_fast.inl"

#undef CHANNEL0_LOAD_SORT_ALPHA  
#define CHANNEL0_LOAD_SORT_ALPHA  \
  ymm0x = ymm6x;                                                                                                             

#undef CHANNEL0_LOAD_CLIP_SORT_ALPHA  /* fedcba9876543210fedcba9876543210 */
#define CHANNEL0_LOAD_CLIP_SORT_ALPHA /* 00000E00000000000000000000000000 */   \
  ymm5x = _mm256_load_si256x (win);                                            \
  ymm4x = _mm256_shuffle_epi32 (ymm7x, SHUFFLE_WIN_MASK);                      \
  ymm5x = _mm256_and_si256 (ymm5x, ymm4x);                                     \
  ymm5x = _mm256_permutevar8x32_epi32 (ymm6x, ymm5x);                          \
  ymm0x = _mm256_or_si256 (ymm6x, ymm5x);                                    

#undef CHANNEL0_LOAD_CLIP_SORT  
#define CHANNEL0_LOAD_CLIP_SORT CHANNEL1_LOAD_CLIP_SORT_ALPHA
#undef CHANNEL0_LOAD_SORT  
#define CHANNEL0_LOAD_SORT CHANNEL1_LOAD_SORT_ALPHA

// only backdrop exist.
#undef CHANNELS 
#define CHANNELS 0
#undef CHANNEL_SORT
#define CHANNEL_SORT CHANNEL0_SORT 
#undef CHANNEL_SORT2
#define CHANNEL_SORT2 CHANNEL0_SORT2 
#undef CHANNEL_LOAD_CLIP
#define CHANNEL_LOAD_CLIP CHANNEL0_LOAD_CLIP 
#undef COMPOSITE
#define COMPOSITE composite0x

#include "gpu.composite_fast.inl"

#endif 

  VSP_FORCEINLINE
  void set_buffer ( uint32_t *buffer, uint32_t value )
  {
#ifndef VSP_VECTOR_PROCESS
    for (uint32_t iterate = 0;  iterate != 32; iterate++)
    {
      buffer[0] = 
      buffer[1] = 
      buffer[2] = 
      buffer[3] = 
      buffer[4] = 
      buffer[5] = 
      buffer[6] = 
      buffer[7] = value;

      buffer += 8;
    }
#else 
    __m256i fill = _mm256_set1_epi32 (value);

#undef M_FILL
#define M_FILL(pos)                                                              \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 0], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 1], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 2], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 3], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 4], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 5], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 6], fill);  \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 7], fill);

    M_FILL (0)
    M_FILL (8)
    M_FILL (16)
    M_FILL (24)
#undef  M_FILL
#endif 
  }

  VSP_FORCEINLINE
  void clear_win_mask ( uint32_t *buffer, uint32_t mask)
  { // buffer -8~248...
#ifndef VSP_VECTOR_PROCESS
    for (uint32_t iterate = 0;  iterate != 32; iterate++)
    {
      buffer[0] &= ~mask;
      buffer[1] &= ~mask;
      buffer[2] &= ~mask;
      buffer[3] &= ~mask;
      buffer[4] &= ~mask;
      buffer[5] &= ~mask;
      buffer[6] &= ~mask;
      buffer[7] &= ~mask;

      buffer += 8;
    }
#else 
    __m256i ymm7x = _mm256_set1_epi32 (~mask);

    __m256i ymm0x;
    __m256i ymm1x;
    __m256i ymm2x;
    __m256i ymm3x;

#undef M_CLR
#define M_CLR(pos)                                                                       \
    ymm0x = _mm256_load_si256 ( & reinterpret_cast<const __m256i *> (buffer)[pos + 0] ); \
    ymm1x = _mm256_load_si256 ( & reinterpret_cast<const __m256i *> (buffer)[pos + 1] ); \
    ymm2x = _mm256_load_si256 ( & reinterpret_cast<const __m256i *> (buffer)[pos + 2] ); \
    ymm3x = _mm256_load_si256 ( & reinterpret_cast<const __m256i *> (buffer)[pos + 3] ); \
                                                                                         \
    ymm0x = _mm256_and_si256 (ymm0x, ymm7x);                                             \
    ymm1x = _mm256_and_si256 (ymm1x, ymm7x);                                             \
    ymm2x = _mm256_and_si256 (ymm2x, ymm7x);                                             \
    ymm3x = _mm256_and_si256 (ymm3x, ymm7x);                                             \
                                                                                         \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 0], ymm0x);        \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 1], ymm1x);        \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 2], ymm2x);        \
    _mm256_store_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 3], ymm3x);  

    M_CLR (0)
    M_CLR (4)
    M_CLR (8)
    M_CLR (12)
    M_CLR (16)
    M_CLR (20)
    M_CLR (24)
    M_CLR (28)

#undef  M_CLR
#endif 
  }

  VSP_FORCEINLINE
  void clr_set_win_mask ( uint32_t *buffer, uint32_t mask, uint32_t left, uint32_t right)
  { // buffer 0~240...
#ifndef VSP_VECTOR_PROCESS
    if ( left <= right )
    {
      for (uint32_t iterate = 0;  iterate != 240; iterate++)
      {
        buffer[0] &= ~mask;
        buffer[0] |= (iterate >= left && iterate < right) ? mask : 0;
      }
    }
    else
    {
      for (uint32_t iterate = 0;  iterate != 240; iterate++)
      {
        buffer[0] &= ~mask;
        buffer[0] |= (iterate >= left || iterate < right) ? mask : 0;
      }
    }
#else 
    if ( left <= right )
    {
      uint32_t lim = (0xffff - (right - 1 - left)) - 1;
      uint32_t posl= lim + 1 - left;
      uint32_t iterate = 15;

      __m256i ymm7x = _mm256_set1_epi32 (mask);
      __m256i ymm6x = _mm256_add_epi32 ( _mm256_set_epi32 (7, 6, 5, 4, 3, 2, 1, 0), _mm256_set1_epi32 (posl));  // pos 
      __m256i ymm5x = _mm256_set1_epi32 (8);  // adv loop.
      __m256i ymm4x = _mm256_set1_epi32 (lim);  // base rangge..
      __m256i ymm3x; 
      __m256i ymm2x;
      __m256i ymm1x; // x >= left && x < right
      __m256i ymm0x; // x >= right && x < left

      do
      {
        ymm0x = _mm256_load_si256x ( buffer );
        ymm1x = _mm256_load_si256x ( buffer + 8 );
        ymm0x = _mm256_andnot_si256 ( ymm7x, ymm0x );
        ymm1x = _mm256_andnot_si256 ( ymm7x, ymm1x );
        ymm2x = _mm256_cmpgt_epi32 (ymm6x, ymm4x);
        ymm4x = _mm256_adds_epu16 (ymm5x, ymm4x);
        ymm3x = _mm256_cmpgt_epi32 (ymm6x, ymm4x);
        ymm4x = _mm256_adds_epu16 (ymm5x, ymm4x);
        ymm2x = _mm256_and_si256 (ymm7x, ymm2x);
        ymm3x = _mm256_and_si256 (ymm7x, ymm3x);
        ymm2x = _mm256_or_si256 (ymm2x, ymm0x);
        ymm3x = _mm256_or_si256 (ymm3x, ymm1x);

        _mm256_store_si256x ( buffer, ymm2x);
        _mm256_store_si256x ( buffer + 8, ymm3x);

        buffer += 8;
        buffer += 8;
      } while (--iterate != 0);
    }
    else
    {
      uint32_t lim = (0xffff - (left - 1 - right)) - 1;
      uint32_t posl= lim + 1 - right;
      uint32_t iterate = 15;

      __m256i ymm7x = _mm256_set1_epi32 (mask);
      __m256i ymm6x = _mm256_add_epi32 ( _mm256_set_epi32 (7, 6, 5, 4, 3, 2, 1, 0), _mm256_set1_epi32 (posl));  // pos 
      __m256i ymm5x = _mm256_set1_epi32 (8);  // adv loop.
      __m256i ymm4x = _mm256_set1_epi32 (lim);  // base rangge..
      __m256i ymm3x; 
      __m256i ymm2x;
      __m256i ymm1x; // x >= left && x < right
      __m256i ymm0x; // x >= right && x < left

      do
      {
        ymm0x = _mm256_load_si256x ( buffer );
        ymm1x = _mm256_load_si256x ( buffer + 8 );
        ymm0x = _mm256_andnot_si256 ( ymm7x, ymm0x );
        ymm1x = _mm256_andnot_si256 ( ymm7x, ymm1x );
        ymm2x = _mm256_cmpgt_epi32 (ymm6x, ymm4x);
        ymm4x = _mm256_adds_epu16 (ymm5x, ymm4x);
        ymm3x = _mm256_cmpgt_epi32 (ymm6x, ymm4x);
        ymm4x = _mm256_adds_epu16 (ymm5x, ymm4x);
        ymm2x = _mm256_blendv_epi8 (ymm7x, ymm0x, ymm2x);
        ymm3x = _mm256_blendv_epi8 (ymm7x, ymm1x, ymm3x);
        ymm2x = _mm256_or_si256 (ymm2x, ymm0x);
        ymm3x = _mm256_or_si256 (ymm3x, ymm1x);

        _mm256_store_si256x ( buffer, ymm2x);
        _mm256_store_si256x ( buffer + 8, ymm3x);

        buffer += 8;
        buffer += 8;
      } while (--iterate != 0);
    }
#endif 
  }


  VSP_FORCEINLINE
  void set_buffer2 ( uint32_t *buffer, uint32_t value )
  {
#ifndef VSP_VECTOR_PROCESS
    for (uint32_t iterate = 0;  iterate != 32; iterate++)
    {
      buffer[0] = 
      buffer[1] = 
      buffer[2] = 
      buffer[3] = 
      buffer[4] = 
      buffer[5] = 
      buffer[6] = 
      buffer[7] = value;

      buffer += 8;
    }
#else 
    __m256i fill = _mm256_set1_epi32 (value);

#undef M_FILL
#define M_FILL(pos)                                                              \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 0], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 1], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 2], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 3], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 4], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 5], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 6], fill);  \
    _mm256_storeu_si256 ( & reinterpret_cast<__m256i *> (buffer)[pos + 7], fill);

    M_FILL (0)
    M_FILL (8)
    M_FILL (16)
    M_FILL (24)
#undef  M_FILL
#endif 
  }

  template < size_t t_nRenderMode, size_t t_nSizeMode, size_t t_nWrapMask>
  uint32_t *bg_render_ ( channel &chan )
  {
    if ( t_nRenderMode == 0 )
    { // standard 4bit tile character + 4bit palette
      uint32_t pos_y = (vcount - (vcount % mosaic_bg_y & chan.mosaic_mask) + scroll_y) & 511;
      uintptr_t mini_y2 = (pos_y & 7) << 2;
 
      uint32_t *buffer = chan.buffer_off_x;
      uint32_t *ned = chan.buffer_end;

      uint32_t iterate = chan.iterates;
      
      uint8_t *character_ = chan.character;
      uint32_t pri = chan.pri;

      uint32_t t = (pos_y & 0xf8) << 2;

      plat::cell **s_= & chan.nt_maps[pos_y >> 7 & 2];
      plat::cell *k = & s_[0][t]; // render start 
      plat::cell *s = & s_[1][t]; // render switch 

      do
      {
        uint32_t *pal = k->palette;
        uint32_t c = LOAD32x (& character_[k->tile_id[0] ^ mini_y2]);

        if ( k->filp_x == 0 )
        {
          buffer[0] = pal[c & 15] | pri; c >>= 4;
          buffer[1] = pal[c & 15] | pri; c >>= 4;
          buffer[2] = pal[c & 15] | pri; c >>= 4;
          buffer[3] = pal[c & 15] | pri; c >>= 4;
          buffer[4] = pal[c & 15] | pri; c >>= 4;
          buffer[5] = pal[c & 15] | pri; c >>= 4;
          buffer[6] = pal[c & 15] | pri; c >>= 4;
          buffer[7] = pal[c     ] | pri;
        }
        else
        {
          buffer[7] = pal[c & 15] | pri; c >>= 4;
          buffer[6] = pal[c & 15] | pri; c >>= 4;
          buffer[5] = pal[c & 15] | pri; c >>= 4;
          buffer[4] = pal[c & 15] | pri; c >>= 4;
          buffer[3] = pal[c & 15] | pri; c >>= 4;
          buffer[2] = pal[c & 15] | pri; c >>= 4;
          buffer[1] = pal[c & 15] | pri; c >>= 4;
          buffer[0] = pal[c     ] | pri;
        }
        k++;
        buffer += 8;

        if ( buffer == ned )
        { // switch nametable page.
          k = s;
        }
      } while (--iterate != 0);
    }
    else if ( t_nRenderMode == 1 )
    { // standard 8bit palette render
      uint32_t pos_y = (vcount - (vcount % mosaic_bg_y & chan.mosaic_mask) + scroll_y) & 511;
      uintptr_t mini_y2 = (pos_y & 7) << 3;
 
      uint32_t *buffer = chan.buffer_off_x;
      uint32_t *ned = chan.buffer_end;

      uint32_t iterate = chan.iterates;
      
      uint8_t *character_ = chan.character;
      uint32_t pri = chan.pri;
  
      uint32_t t = (pos_y & 0xf8) << 2;

      plat::cell **s_= & chan.nt_maps[pos_y >> 7 & 2];
      plat::cell *k = & s_[0][t]; // render start 
      plat::cell *s = & s_[1][t]; // render switch 

      do
      {
        uint32_t *cl = & character_[k->tile_id[1] ^ mini_y2];

        uint32_t c = LOAD32x (& cl[0]);
        uint32_t c2= LOAD32x (& cl[1]);

        if ( c->filp_x == 0 )
        {
          buffer[0] = pal[c & 0xff] | pri; c >>= 8;
          buffer[1] = pal[c & 0xff] | pri; c >>= 8;
          buffer[2] = pal[c & 0xff] | pri; c >>= 8;
          buffer[3] = pal[c       ] | pri;
          buffer[4] = pal[c2& 0xff] | pri; c2>>= 8;
          buffer[5] = pal[c2& 0xff] | pri; c2>>= 8;
          buffer[6] = pal[c2& 0xff] | pri; c2>>= 8;
          buffer[7] = pal[c2      ] | pri;
        }
        else
        {
          buffer[7] = pal[c & 0xff] | pri; c >>= 8;
          buffer[6] = pal[c & 0xff] | pri; c >>= 8;
          buffer[5] = pal[c & 0xff] | pri; c >>= 8;
          buffer[4] = pal[c       ] | pri;
          buffer[3] = pal[c2& 0xff] | pri; c2>>= 8;
          buffer[2] = pal[c2& 0xff] | pri; c2>>= 8;
          buffer[1] = pal[c2& 0xff] | pri; c2>>= 8;
          buffer[0] = pal[c2      ] | pri;
        }
        k++;
        buffer += 8;

        if ( buffer == ned )
        { // switch nametable page.
          k = s;
        }
      } while (--iterate != 0);
    }
    else 
    { 
      int32_t pos_x = chan.scale_start_rt[0];
      int32_t pos_y = chan.scale_start_rt[1];
      int32_t delta_x = chan.scale_params_rt[0]; // dx 
      int32_t delta_x2 = chan.scale_params_rt[1]; // dmx 
      int32_t delta_y = chan.scale_params_rt[2]; // dy 
      int32_t delta_y2 = chan.scale_params_rt[3]; // dmy 
      int32_t iterate = 240;
      int32_t pri = chan.pri;

      uint32_t control = chan.control;
      uint8_t *memory = & this->memory[0];
      uint32_t *pal = & bg_pal[0];
      uint32_t *buffer = & chan.buffer[8];

      if ( chan.mosaic_mask != 0 )
      {
        int32_t vvv = vcount % mosaic_bg_y;

        pos_x -= vvv * delta_x2;
        pos_y -= vvv * delta_y2;
      }

      if (t_nRenderMode == 2)
      { // rotate background render (8bit palette)
        uint8_t *chr_entry = memory + (control >> 2 & 3) * MEM_16K;
        uint8_t *nt_entry = memory + (control & 0x1f00 >> 8) * MEM_2K;

  #undef SIZE_MODE_MASK
  #undef OVERFLOW_MASK
  #define SIZE_MODE_MASK ((1 << (4 + t_nSizeMode)) - 1)
  #define OVERFLOW_MASK  (~((  ((1 << ((4 + t_nSizeMode)) * 2) - 1) << 8 ) | 0xff))

        if ( t_nWrapMask != 0 )
        {   
          pos_x = pos_x & 0x7ff
                | (pos_x & SIZE_MODE_MASK << 11) << 5;
          pos_y = pos_y & 0xff
                | (pos_y & 0x700) << 3
                | (pos_x & SIZE_MODE_MASK << 11) << 12;
          delta_x = delta_x & 0x7ff
                | (delta_x & SIZE_MODE_MASK << 11) << 5;
          delta_x|= 0x0000f800;
          delta_y = delta_y & 0xff
                | (delta_y & 0x700) << 3
                | (delta_y & SIZE_MODE_MASK << 11) << 12;
          delta_y|= 0x007fc700;

          // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
          // eeeeeeeeeeeeeXXXXXXX0xxxffffffff  yyy:offset y in tile. 
          // eeeeeeeeeeeeeYYYYYYY0yyyffffffff  yyy:offset y in tile.



          // 000000000XXXXXXX0xxx000000000000  yyy:offset y in tile. 
          // 000000000YYYYYYY0yyy000000000000  yyy:offset y in ti`le.
           

          do                
          { // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
            // 000000000XXXXXXX00000xxxffffffff  yyy:offset y in tile.  
            // XXXXXXX000000000xxxffffffff  yyy:offset y in tile.  
            //          XXXXXXX000000000xxxffffffff
            // 00YYYYYYY000000000yyy000ffffffff  XXXXXXX:0~15 for 128x128 resolution 
            //                                          :0~31 for 256x256 resolution 
            //                                          :0~63 for 512x512 resolution 
            //                                          :0~127 for 1024x1024 resolution 
            //                                   YYYYYYY:0~15 for 128x128 resolution 
            //                                          :0~31 for 256x256 resolution 
            //                                          :0~63 for 512x512 resolution 
            //                                          :0~127 for 1024x1024 resolution 
            pos_x &= 0xf8000700 | SIZE_MODE_MASK << 16; 
            pos_y &= 0x00003800 | SIZE_MODE_MASK << (20 + t_nSizeMode);                                        
                                         
            uint32_t mixer = pos_x | pos_y; 
            uint32_t nt_address = mixer >> 16; // wrap round, when exceeding 64K (is this how it is handled? Or directly access the memory of the sprite?....)
            uint32_t tile_id = nt_entry[nt_address]; 
            uint32_t chr_address = (static_cast<uint16_t> (mixer) >> 8) + (tile_id << 6);
            uint32_t tile_pixel = chr_entry[chr_address]; // not exceed 16K
            // v0:nametable entry 
            // v1:character entry 
            // v2:palette entry 
            // v3:pri mask 
            // v4:buffer pointer.
            // v5:iterate 


            pos_x += delta_x;
            pos_y += delta_y; 

            buffer[0] = pal[tile_pixel] | pri; 
            buffer++;
          } while (--iterate != 0);
        }
        else
        {
          do                
          { 
              // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
              // 000000000XXXXXXXxxx0000000000000  yyy:offset y in tile.  
              // 000000000YYYYYYYyyy0000000000000
                               
              // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
              // XXXXXXX000000000xxx0000000000000  yyy:offset y in tile.  
              // 0000000YYYYYYY000000000yyy000000

              // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
              // 00000000XXXXXXX000000xxx000000000  yyy:offset y in tile.  
              // 0XXXXXXX000000xxx000000000

              // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
              // 0000000xxx000xxx0000000000000000:offset y in tile.  
              // 0XXXXXXX000000xxx000000000



              // fedcba9876543210fedcba9876543210  xxx:offset x in tile 
              // 0000000000000000YYYYYyyyXXXXXxxx000000000000000  yyy:offset y in tile.  



              // 00000000000000YYYYYYYyyy000000000000000
              //                          XXXXXXX0000000xxx000000000000000  yyy:offset y in tile.  
              //                   YYYYYYY0000000yyy000000000000000
  //;    XXXXXXX0000000xxx000000000000000
 // ; YYYYYYY0000000yyy000000000000000
              // 0XXXXXXX00000000xxx0000000000000
              // 0YYYYYYY00000000yyy0000000000000

              // 0XXXXXXX00000000xxx0000000000000
              // 0YYYYYYY00000000yyy0000000000000







              


              // fedcba9876543210fedcba9876543210
              // 0000000000000000XXXXXxxx00000000
              //       YYYYYXXXXXyyyxxx
              //           yyyxxxYYYYYXXXXX
              //           yyyxxx  YYYYYXXXXX
              //       YYYYYXXXXXyyyxxx
              // 0000000000000000YYYYYyyy00000000
              // ~127 for 1024x1024 resolution 
              //                                            YYYYYYY:0~15 for 128x128 resolution 
              //                                          :0~31 for 256x256 resolution 
              //                                          :0~63 for 512x512 resolution 
              //                                          :0~127 for 1024x1024 resolution 
              uint32_t off_in_tile = pos_x >> 8 & 7 | pos_y >> 5 & 0x38;
              uint32_t off_in_nametable = ( (pos_y & ~0x7ff) << (t_nSizeMode + 4) | pos_x ) >> 11;

              uint32_t tile_id = nt_entry[off_in_nametable]; 
              uint32_t chr_address = off_in_tile + (tile_id << 6);
              uint32_t tile_pixel = chr_entry[chr_address]; 
                
              buffer[0] = pal [tile_pixel] | pri; 
            }
            else
            {
              buffer[0] = pal[0] | pri;   
            }
            pos_x += delta_x;
            pos_y += delta_y; 

            buffer++;
          } while (--iterate != 0);
        }
      }
      else if (t_nRenderMode == 3)
      { // framebuffer 240*160 
        memory = & memory[MEM_64K];

        do                
        { 
          if ( static_cast<uint32_t> (pos_x) < 0xf000
           && static_cast<uint32_t> (pos_y) < 0xa000 )
          {
            uint32_t pos_x2 = pos_x >> 8;
            uint32_t pos_y2 = pos_y >> 8;
            uint32_t pos_n = pos_y2 * 240 + pos_x2;
            uint32_t pos_n_ = pos_n << 1;
            uint32_t pix_lo = memory[pos_n_]; 
            uint32_t pix_hi = memory[pos_n_ + 1]; 
            uint32_t pix16 = pix_lo | pix_hi << 8;
            uint32_t pix16x= pix16 | pix16 << 16;
            uint32_t pix16x2= pix16x >> 10 | pix16x << 16; // swap red, blue.
                     pix16x2|= (pix16x & 0x1f << 5) << 3;
                     pix16x2&= 0x1f1f1f;
                     pix16x2<<= 3;

            buffer[0] = pix16x2 | pri; 
          }
          else
          {
            buffer[0] = pal[0] | pri;  
          }
          pos_x += delta_x;
          pos_y += delta_y; 

          buffer++;
        } while (--iterate != 0);
      }
      else if ( t_nVideoMode == 4 )
      { // framebuffer swapbuffer palette mode         
        memory = & memory[MEM_64K + ( (gpu<T>::control & 0x10) ? 0xa000 : 0)];

        do                
        { 
          if ( static_cast<uint32_t> (pos_x) < 0xf000
           && static_cast<uint32_t> (pos_y) < 0xa000 )
          {         
            uint32_t pos_x2 = pos_x >> 8;
            uint32_t pos_y2 = pos_y >> 8;
            uint32_t pos_n = pos_y2 * 240 + pos_x2;
            uint32_t col = memory[pos_n]; 

            buffer[0] = pal[col] | pri; 
          }
          else
          {
            buffer[0] = pal[0] | pri;   
          }
          pos_x += delta_x;
          pos_y += delta_y; 

          buffer++;
        } while (--iterate != 0);
      }
      else if ( t_nVideoMode == 5 )
      { // framebuffer swapbuffer rgb mode.      
        iterate = 160;

        memory = & memory[MEM_64K + ( (gpu<T>::control & 0x10) ? 0xa000 : 0)];

        do                
        { 
          if ( static_cast<uint32_t> (pos_x) < 0xa000
           && static_cast<uint32_t> (pos_y) < 0x8000 )
          {         
            uint32_t pos_x2 = pos_x >> 8;
            uint32_t pos_y2 = pos_y >> 8;
            uint32_t pos_n = pos_y2 * 160 + pos_x2;
            uint32_t pos_n_ = pos_n << 1;
            uint32_t pix_lo = memory[pos_n_]; 
            uint32_t pix_hi = memory[pos_n_ + 1]; 
            uint32_t pix16 = pix_lo | pix_hi << 8;
            uint32_t pix16x= pix16 | pix16 << 16;
            uint32_t pix16x2= pix16x >> 10 | pix16x << 16; // swap red, blue.
                     pix16x2|= (pix16x & 0x1f << 5) << 3;
                     pix16x2&= 0x1f1f1f;
                     pix16x2<<= 3;

            buffer[0] = pix16x2 | pri; 
          }
          else
          {
            buffer[0] = palette_[0] | pri;   
          }
          pos_x += delta_x;
          pos_y += delta_y; 

          buffer++;
        } while (--iterate != 0);
      }      
      chan.scale_start_rt[0] += delta_x2;
      chan.scale_start_rt[1] += delta_y2;
    }

    if ( chan.mosaic_mask != 0 )
    {
      mosaic_hori (& chan.buffer[8]);
    }
    return & chan.buffer[8];
  }
};



