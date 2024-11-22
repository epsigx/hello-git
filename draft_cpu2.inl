// template <class T>
struct mpu
{ // most of the implementation of gpu is referenced from the source code of vba-m
  // simply simulated two situations: 
  //   #0:instruction cache is always invalid (for timing accuracy) 
  //   #1:instruction cache is always valid (for performance).
  // 
  //  bank switch instruction opcode 
  // 0x47ffefdf 
  //   arm7 opcode -> 0x7fd 
  //   thumb opcode -> 0x3bf 
  union optrace
  {
    struct 
    {                          // d0:code breakpoint 
      uint32_t opcode;         // d1:code breakpoint (eval expression)
      uint32_t opcode2;        // d2:memory read breakpoint
      uint8_t *trace;          // d3:memory read breakpoint (eval expression)
      uint8_t *code;           // d4:memory write breakpoint
    };                         // d5:memory write breakpoint (eval expression)
    uintptr_t unused[4];       // d7-d6:
  };
  uint32_t cf; // carry flag
  uint32_t zf; // zero flag
  uint32_t nf; // negtive flag
  uint32_t of; // overflow flag

  uint32_t breakpoint_master_mask;  // for breakpoint next one instruction
                                    // for trap one instruction
  uint32_t trap_mask;               //

  uint32_t thumb_mask;
  uint32_t irq_mask; // standard irq enable mask 

  uint32_t mode;

  uint32_t regs[16];  // register r0-r12, sp, lr link, pc
  uint32_t regs2x[24]; // register bank cache r13-r14, psr
  uint32_t regs3x[10]; // register bank cache r8-r12
  uint32_t *regs4px[16]; // mode bank ptr for regs2x

  optrace prefetch; // pipeline opcode 

  uint32_t ime;
  uint32_t ie_if; 

#define ARM7_MODE_USER 0
#define ARM7_MODE_FIQ 1
#define ARM7_MODE_IRQ 2
#define ARM7_MODE_MGR 3
#define ARM7_MODE_ABT 7
#define ARM7_MODE_UNDEF 11
#define ARM7_MODE_SYS 15
  memors mmu[16];

  uint32_t waits[2]; // 0:noseq 1:seq.

  uint8_t 
  read8x (uint32_t address)
  {
    return 0;
  }

  uint16_t 
  read16x (uint32_t address)
  {
    return 0;
  }

  uint16_t 
  read32x (uint32_t address)
  {
    return 0;
  }

  uint8_t 
  read8 (uint32_t address)
  {
    return 0;
  }

  uint16_t 
  read16 (uint32_t address)
  {
    return 0;
  }

  uint16_t 
  read32 (uint32_t address)
  {
    return 0;
  }

  VSP_FORCEINLINE 
  uint32_t mul_clks (uint32_t rs) 
  { // multiplier's clks, m
    // see ARM7TDMI Technical Reference Manual's 6.20 Instruction speed summary
    // m is: 
    // 1 if bits [31:8] of the multiplier operand (rs) are all zero or one, else 
    // 2 if bits [31:16] of the multiplier operand (rs) are all zero or one, else 
    // 3 if bits [31:24] of the multiplier operand (rs) are all zero or all one, else 
    // 4.
#ifdef VSP_VECTOR_PROCESS
    uint32_t rs_ = (rs ^ static_cast<int32_t> (rs) >> 31) | 1;
    return 4 - (_lzcnt_u32 (rs_) >> 3); 
#else 
    rs ^= static_cast<int32_t> (rs) >> 31;

    if ((rs & 0xffffff00) == 0)                         
    {
      return 1;  
    }
    else if ((rs & 0xffff0000) == 0)                    
    {
      return 2;
    }
    else if ((rs & 0xff000000) == 0)                    
    { 
      return 3;
    }
    else                                                
    { 
      return 4;   
    }
#endif 
  }

  VSP_FORCEINLINE
  uint32_t get_cpsr (void)
  { // Current Program Status Register (CPSR)
    //   Bit   Expl.
    //   31    N - Sign Flag       (0=Not Signed, 1=Signed)               ;\
    //   30    Z - Zero Flag       (0=Not Zero, 1=Zero)                   ; Condition
    //   29    C - Carry Flag      (0=Borrow/No Carry, 1=Carry/No Borrow) ; Code Flags
    //   28    V - Overflow Flag   (0=No Overflow, 1=Overflow)            ;/
    //   27    Q - Sticky Overflow (1=Sticky Overflow, ARMv5TE and up only)
    //   26-8  Reserved            (For future use) - Do not change manually!
    //   7     I - IRQ disable     (0=Enable, 1=Disable)                     ;\
    //   6     F - FIQ disable     (0=Enable, 1=Disable)                     ; Control
    //   5     T - State Bit       (0=ARM, 1=THUMB) - Do not change manually!; Bits
    //   4-0   M4-M0 - Mode Bits   (See below)                               ;/
    regs2x[2] &= ~0xff0000bf;
    regs2x[2] |= nf & 0x80000000;
    regs2x[2] |= zf == 0 ? 0x40000000 : 0;
    regs2x[2] |= (cf & 1) != 0 ? 0x20000000 : 0;
    regs2x[2] |= (of & 0x80000000) ? 0x10000000 : 0;
    regs2x[2] |= irq_mask == 0 ? 0x80 : 0;
    regs2x[2] |= thumb_mask != 0 ? 0x20 : 0;
    regs2x[2] |= mode | 0x10;

    return regs2x[2];
  }

  VSP_FORCEINLINE
  uint32_t get_spsr (void)
  {
    uint32_t mode_ = mode & 15;
    return mode_ == 0 ? get_cpsr () : regs4px[mode_][2];
  }

  VSP_STATIC_FORCEINLINE
  uint32_t ror32 (uint32_t value, uint32_t shift)
  {
    VSP_ASSERT (shift < 32);
#ifdef VSP_VECTOR_PROCESS
    return _rotr (value, shift);
#else 
  //return value >> shift | value << (32 - shift);
    return value >> shift | value << ((32 - shift) & 31);
#endif 
  }

  VSP_STATIC_FORCEINLINE
  uint32_t ror16 (uint32_t value, uint32_t shift)
  {
    VSP_ASSERT (shift < 32);
#ifdef VSP_VECTOR_PROCESS
    return _rotr (value, shift);
#else 
  //return value >> shift | value << (32 - shift);
    return value >> shift | value << ((32 - shift) & 31);
#endif 
  }

  VSP_STATIC_FORCEINLINE
  uint64_t mul32to64 (uint32_t lhs, uint32_t rhs)
  {
#ifdef VSP_VECTOR_PROCESS
    return __emulu (lhs, rhs);
#else 
    return static_cast<uint64_t> (lhs) * rhs;
#endif 
  }

  VSP_STATIC_FORCEINLINE
  int64_t smul32to64 (int32_t lhs, int32_t rhs)
  {
#ifdef VSP_VECTOR_PROCESS
    return __emul (lhs, rhs);
#else 
    return static_cast<int64_t> (lhs) * rhs;
#endif 
  }

  int32_t tick (void)
  {
    uint32_t arm7_opcode;
    uint32_t arm7_cond;
    uint32_t arm7_fetch_seq;
    uint32_t arm7_instruction_;
    uint32_t arm7_instruction;
    uint32_t arm7_waits;
    uint32_t thumb_opcode;
    uint32_t thumb_cond;
    uint32_t thumb_fetch_seq;
    uint32_t thumb_instruction_;
    uint32_t thumb_instruction;
    uint32_t t;

    struct mem_hook_
    {
      memors *bank;

      uint8_t *mem; 
      uintptr_t fast_acc_mask;
      
      void *unused;
    } mem_hooks[16];

    int32_t ticks = 0;

    // mem_bank->mem[mem_bank_address]



// #define M_READ_HOOKx(ptr)
//   do                                                          \
//   {                                                           \
//     uint32_t value = ptr[0];                                  \
//     uint32_t t;                                               \
//                                                               \
//     if ( (t = (value & 0x0c) & breakpoint_master_mask) != 0 ) \
//     {                                                         \
//       if ( (t & 0x04) != 0 )                                  \
//       {                                                       \
//         goto L9;                                              \
//       }                                                       \
//       else if ( (t & 0x08) != 0 )                             \
//       {                                                       \
//       }                                                       \
//     }                                                         \
//   } while (0)

#define M_READ_HOOK_(address_, slot_, mask_, hit_cb)            \
  do                                                            \
  {                                                             \
    uint32_t address = (address_);                              \
                                                                \
    mem_hook_ &hook = mem_hooks[slot_];                         \
    memors &bank = & mmu[address >> 24 & 15];                   \
                                                                \
    uint32_t mirror_mask = bank.mirror_mask;                    \
                                                                \
    hook.mem = & bank.mem[address & mirror_mask & (mask_)];     \
    hook.fast_acc_mask = bank.fast_acc_mask;                    \
    hook.bank = & bank;                                         \
                                                                \
    uint32_t trace = hook.mem[mirror_mask + 4 + 1];             \
    uint32_t t;                                                 \
                                                                \
    if ( (t = (trace & 0x0c) & breakpoint_master_mask) != 0 )   \
    {                                                           \
      if ( (t & 0x04) != 0 || (                                 \
        (t & 0x08) != 0 && eval_read (address, mask_) == 0 ) )  \
      {                                                         \
        hit_cb ();                                              \
        goto L9;                                                \
      }                                                         \
    }                                                           \
  } while (0)

#define M_WRITE_HOOK_(address_, value_, slot_, mask_, hit_cb)    \
  do                                                          \
  {                                                           \
    uint32_t address = (address_);                            \
    uint32_t value = value_;                                  \
                                                              \
    mem_hook_ &hook = mem_hooks[slot_];                       \
    memors &bank = & mmu[address >> 24 & 15];                 \
                                                              \
    uint32_t mirror_mask = bank.mirror_mask;                  \
                                                              \
    hook.mem = & bank.mem[address & mirror_mask & (mask_)];   \
    hook.fast_acc_mask = bank.fast_acc_mask;                  \
    hook.bank = & bank;                                       \
                                                              \
    uint32_t trace = hook.mem[mirror_mask + 4 + 1];           \
    uint32_t t;                                               \
                                                              \
    if ( (t = (trace & 0x30) & breakpoint_master_mask) != 0 ) \
    {                                                         \
      if ( (t & 0x10) != 0 )                                  \
      {                                                       \
        hit_cb ();                                            \
        goto L9;                                              \
      }                                                       \
      else if ( (t & 0x20) != 0 && eval_write (address, value, mask_) == 0 )                             \
      {                                                       \
        hit_cb ();                                            \
        goto L9;                                              \
      }                                                       \
    }                                                         \
  } while (0)

#define M_ACCESS_HOOK_(address_, value_, slot_, mask_, hit_cb)        \
  do                                                                  \
  {                                                                   \
    uint32_t address = (address_);                                    \
    uint32_t value = value_;                                          \
                                                                      \
    mem_hook_ &hook = mem_hooks[slot_];                               \
    memors &bank = & mmu[address >> 24 & 15];                         \
                                                                      \
    uint32_t mirror_mask = bank.mirror_mask;                          \
                                                                      \
    hook.mem = & bank.mem[address & mirror_mask & (mask_)];           \
    hook.fast_acc_mask = bank.fast_acc_mask;                          \
    hook.bank = & bank;                                               \
                                                                      \
    uint32_t trace = hook.mem[mirror_mask + 4 + 1];                   \
    uint32_t t;                                                       \
                                                                      \
    if ( (t = (trace & 0x3c) & breakpoint_master_mask) != 0 )         \
    {                                                                 \
      if ( (t & 0x14) != 0 )                                          \
      {                                                               \
        hit_cb ();                                                    \
        goto L9;                                                      \
      }                                                               \
      else if ( (t & 0x08) != 0 && eval_read (address, mask_) == 0 )  \
      {                                                               \
        hit_cb ();                                                    \
        goto L9;                                                      \
      }                                                               \
      else if ( (t & 0x20) != 0 && eval_write (address, value, mask_) == 0 ) \
      {                                                                      \
        hit_cb ();                                                           \
        goto L9;                                                             \
      }                                                                      \
    }                                                                        \
  } while (0)

#define M_BREAKPOINT_CLEANUP_NONE() \
  do                                \
  {                                 \
  } while (0)

#define M_READ_HOOK8(address_) M_READ_HOOK_ (address_, 0, 0)
#define M_WRITE_HOOK8(address_, value_) M_WRITE_HOOK_ (address_, value_, 0, 0)
#define M_ACCESS_HOOK8(address_, value_) M_ACCESS_HOOK_ (address_, value_, 0, 0)
#define M_READ_HOOK16(address_) M_READ_HOOK_ (address_, 0, 1)
#define M_WRITE_HOOK16(address_, value_) M_WRITE_HOOK_ (address_, value_, 0, 1)
#define M_ACCESS_HOOK16(address_, value_) M_ACCESS_HOOK_ (address_, value_, 0, 1)
#define M_READ_HOOK32(address_) M_READ_HOOK_ (address_, 0, 3)
#define M_WRITE_HOOK32(address_, value_) M_WRITE_HOOK_ (address_, value_, 0, 3)
#define M_ACCESS_HOOK32(address_, value_) M_ACCESS_HOOK_ (address_, value_, 0, 3)
#define M_LDM_HOOK(slot_, address_) M_READ_HOOK_ (address_, slot_, 3)
#define M_STM_HOOK(slot_, address_, value_) M_WRITE_HOOK_ (address_, value_, slot_, 3)

#define M_THUMB_FLUSH_PIPELINE()                      \
  do                                                  \
  {                                                   \
    uint32_t pc = regs[15];                           \
    uint32_t pc2= pc & ~3;                            \
                                                      \
    memors &bank = mmu[pc >> 24 & 15];                \
                                                      \
    uint8_t *codeptr = bank.mem_rs;                   \
             codeptr+= pc & bank.mirror_mask;         \
                                                      \
    waits[0] = bank.code_waits[0];                    \
    waits[1] = bank.code_waits[1];                    \
                                                      \
    prefetch.opcode = LOAD16x (codeptr);              \
    prefetch.opcode2= LOAD16x (codeptr + 2);          \
    prefetch.code = codeptr + 4;                      \
    prefetch.trace = & codeptr[bank.mirror_mask + 5]; \
                                                      \
    ticks += waits[0]; /* noseq fetch */              \
    ticks += waits[1]; /* seq fetch */                \
                                                      \
    regs[15] = pc + 4;                                \
  } while (0)

#define M_ARM7_FLUSH_PIPELINE()                       \
  do                                                  \
  {                                                   \
    uint32_t pc = regs[15];                           \
    uint32_t pc2= pc & ~3;                            \
                                                      \
    memors &bank = mmu[pc >> 24 & 15];                \
                                                      \
    uint8_t *codeptr = bank.mem_rs;                   \
             codeptr+= pc & bank.mirror_mask;         \
                                                      \
    waits[0] = bank.code_waits[0];                    \
    waits[1] = bank.code_waits[1];                    \
                                                      \
    prefetch.opcode = LOAD32x (codeptr);              \
    prefetch.opcode2= LOAD32x (codeptr + 4);          \
    prefetch.code = codeptr + 8;                      \
    prefetch.trace = & codeptr[bank.mirror_mask + 5]; \
                                                      \
    ticks += waits[0]; /* noseq fetch */              \
    ticks += waits[1]; /* seq fetch */                \
                                                      \
    regs[15] = pc + 8;                                \
  } while (0)

#define M_ARM7_NEXT(slot)                       \
  do                                            \
  {                                             \
    prefetch.opcode = prefetch.opcode2;         \
    prefetch.opcode2 = LOAD32x (prefetch.code); \
    prefetch.trace+= 4;                         \
    prefetch.code += 4;                         \
                                                \
    ticks += waits[slot];                       \
                                                \
    regs[15] += 4;                              \
  } while (0)                                 
                                              
#define M_THUMB_NEXT(slot)                     \
  do                                           \
  {                                            \
    prefetch.opcode = prefetch.opcode2;        \
    prefetch.opcode2= LOAD16x (prefetch.code); \
    prefetch.trace+= 2;                        \
    prefetch.code += 2;                        \
                                               \
    ticks += waits[slot];                      \
                                               \
    regs[15] += 2;                             \
  } while (0)

#define M_ARM7_NEXT_SEQ() M_ARM7_NEXT (1)
#define M_ARM7_NEXT_NOSEQ() M_ARM7_NEXT (0)
#define M_THUMB_NEXT_SEQ() M_THUMB_NEXT (1)
#define M_THUMB_NEXT_NOSEQ() M_THUMB_NEXT (0)

    if ( thumb_mask != 0 )
    { // thumb instruction


    }
    else
    { // standard instruction





#define COND_MISS()                          \
  do                                         \
  {                                          \
    M_ARM7_NEXT_SEQ ();                      \
                                             \
    ticks++; /* condition = false:1S cycle */\
  } while (0)

#define COND_EXEC(opcode, cond) \
  case (opcode):                \
                                \
   if ( cond )                  \
   {                            \
     break;                     \
   }                            \
   COND_MISS()               

      if ( ((t = opcodes[1].trace[0] & 3) & breakpoint_master_mask) != 0 )
      { // code breakpoint
        if ( (t & 1) != 0 )
        {
          goto L9;
        }
        else if ((t & 2) != 0 && eval_exec (regs[15] - 8, 3) == 0 )
        {
          goto L9;
        }
      }
      arm7_opcode = opcodes[0].opcode;
      arm7_cond = arm7_opcode >> 28; 
      arm7_instruction_ = arm7_opcode & 0x0ff000f0;  
      arm7_instruction = (arm7_instruction_ | arm7_instruction_ << 24) >> 20;

      if ( VSP_UNLIKELY (arm7_cond != 0x0e) )
      {
        switch (arm7_cond)
        {
        COND_EXEC ( 0x00, ( zf == 0) );
        COND_EXEC ( 0x01, ( zf != 0) );
        COND_EXEC ( 0x02, ( (cf & 1) == 0) );
        COND_EXEC ( 0x03, ( (cf & 1) != 0) );
        COND_EXEC ( 0x04, ( nf & 0x80000000) );
        COND_EXEC ( 0x05, ( nf & 0x80000000) == 0 );
        COND_EXEC ( 0x06, ( of & 0x80000000) );
        COND_EXEC ( 0x07, ( of & 0x80000000) == 0 );
        COND_EXEC ( 0x08, ( static_cast<int32_t> (cf << 31) >> 31 & zf) );
        COND_EXEC ( 0x09, ( static_cast<int32_t> (cf << 31) >> 31 & zf) == 0 );
        COND_EXEC ( 0x0A, ( static_cast<int32_t> (nf ^ of) >= 0 ) );
        COND_EXEC ( 0x0B, ( static_cast<int32_t> (nf ^ of) < 0 ) );
        COND_EXEC ( 0x0C, ( static_cast<int32_t> (nf ^ of ^ 0x80000000) >> 31 & zf ) != 0 );
        COND_EXEC ( 0x0D, ( static_cast<int32_t> (nf ^ of ^ 0x80000000) >> 31 & zf ) == 0 );
        case 0x0e: // always. 
          break;
        case 0x0f:
          COND_MISS();
        default:
          assert (0);
          break;
        }
      } 

#define M_ARM7_MUL(sign)                           \
  do                                               \
  { /* cycles:1S+mI */                             \
    uint32_t  rm = regs[arm7_opcode & 15];         \
    uint32_t  rs = regs[arm7_opcode >> 8 & 15];    \
    uint32_t &rd = regs[arm7_opcode >> 16 & 15];   \
    uint32_t  mul_internal_cycles = mul_clks (rs); \
    uint32_t  res = rm * rs;                       \
                                                   \
    rd = res;                                      \
                                                   \
    ticks++; /* 1s */                              \
    ticks+= mul_internal_cycles; /* mI */          \
                                                   \
    if ( sign != 0 )                               \
    { /* set n, z flags */                         \
      nf = res;                                    \
      zf = res;                                    \
    }                                              \
    M_ARM7_NEXT_SEQ ();                            \
  } while (0)

#define M_ARM7_MLA(sign)                           \
  do                                               \
  { /* cycles:1S+mI+I */                           \
    uint32_t  rm = regs[arm7_opcode & 15];         \
    uint32_t  rs = regs[arm7_opcode >> 8 & 15];    \
    uint32_t  rn = regs[arm7_opcode >> 12 & 15];   \
    uint32_t &rd = regs[arm7_opcode >> 16 & 15];   \
    uint32_t mul_internal_cycles = mul_clks (rs);  \
    uint32_t res = rm * rs;                        \
                                                   \
    rd = res + rn;                                 \
                                                   \
    ticks++; /* 1s */                              \
    ticks+= mul_internal_cycles; /* mI */          \
    ticks++; /* 1I */                              \
                                                   \
    if ( sign != 0 )                               \
    { /* set n, z flags */                         \
      nf = res;                                    \
      zf = res;                                    \
    }                                              \
    M_ARM7_NEXT_SEQ ();                            \
  } while (0)

#define M_ARM7_UMULL(sign)                               \
  do                                                     \
  { /* cycles:1S+mI+I */                                 \
    uint32_t  rm  = regs[arm7_opcode & 15];              \
    uint32_t  rs  = regs[arm7_opcode >> 8 & 15];         \
    uint32_t &rdl = regs[arm7_opcode >> 12 & 15];        \
    uint32_t &rdh = regs[arm7_opcode >> 16 & 15];        \
    uint32_t mul_internal_cycles = mul_clks (rs);        \
    uint64_t res = mul32to64 (rm, rs);                   \
    uint32_t res_lo = static_cast<uint32_t> (res);       \
    uint32_t res_hi = static_cast<uint32_t> (res >> 32); \
                                                         \
    rdl = res_lo;                                        \
    rdh = res_hi;                                        \
                                                         \
    ticks++; /* 1s */                                    \
    ticks+= mul_internal_cycles; /* mI */                \
    ticks++; /* 1I */                                    \
                                                         \
    if ( sign != 0 )                                     \
    { /* set n, z flags */                               \
      zf = res_lo | res_hi;                              \
      nf = res_hi;                                       \
    }                                                    \
    M_ARM7_NEXT_SEQ ();                                  \
  } while (0);
  
#define M_ARM7_SMULL(sign)                               \
  do                                                     \
  { /* cycles:1S+mI+I */                                 \
     int32_t rm = regs[arm7_opcode & 15];                \
     int32_t rs = regs[arm7_opcode >> 8 & 15];           \
    uint32_t &rdl = regs[arm7_opcode >> 12 & 15];        \
    uint32_t &rdh = regs[arm7_opcode >> 16 & 15];        \
    uint32_t mul_internal_cycles = mul_clks (rs);        \
     int64_t res = smul32to64 (rm, rs);                  \
    uint32_t res_lo = static_cast<uint32_t> (res);       \
    uint32_t res_hi = static_cast<uint32_t> (res >> 32); \
                                                         \
    rdl = res_lo;                                        \
    rdh = res_hi;                                        \
                                                         \
    ticks++; /* 1s */                                    \
    ticks+= mul_internal_cycles; /* mI */                \
    ticks++; /* 1I */                                    \
                                                         \
    if ( sign != 0 )                                     \
    { /* set n, z flags */                               \
      zf = res_lo | res_hi;                              \
      nf = res_hi;                                       \
    }                                                    \
    M_ARM7_NEXT_SEQ ();                                  \
  } while (0)
  
#define M_ARM7_UMLAL(sign)                               \
  do                                                     \
  { /* cycles:1S+mI+2I */                                \
    uint32_t rm = regs[arm7_opcode & 15];                \
    uint32_t rs = regs[arm7_opcode >> 8 & 15];           \
    uint32_t &rdl = regs[arm7_opcode >> 12 & 15];        \
    uint32_t &rdh = regs[arm7_opcode >> 16 & 15];        \
    uint64_t dsh = rdh;                                  \
    uint64_t base= dsh << 32 | rdl;                      \
    uint32_t mul_internal_cycles = mul_clks (rs);        \
    uint64_t res = mul32to64 (rm, rs) + base;            \
    uint32_t res_lo = static_cast<uint32_t> (res);       \
    uint32_t res_hi = static_cast<uint32_t> (res >> 32); \
                                                         \
    rdl = res_lo;                                        \
    rdh = res_hi;                                        \
                                                         \
    ticks++; /* 1s */                                    \
    ticks+= mul_internal_cycles; /* mI */                \
    ticks++; /* 1I */                                    \
    ticks++; /* 1I */                                    \
                                                         \
    if ( sign != 0 )                                     \
    { /* set n, z flags */                               \
      zf = res_lo | res_hi;                              \
      nf = res_hi;                                       \
    }                                                    \
    M_ARM7_NEXT_SEQ ();                                  \
  } while (0) 

#define M_ARM7_SMLAL(sign)                               \
  do                                                     \
  { /* cycles:1S+mI+2I */                                \
     int32_t rm = regs[arm7_opcode & 15];                \
     int32_t rs = regs[arm7_opcode >> 8 & 15];           \
    uint32_t &rdl = regs[arm7_opcode >> 12 & 15];        \
    uint32_t &rdh = regs[arm7_opcode >> 16 & 15];        \
    uint64_t dsh = rdh;                                  \
    uint64_t base= dsh << 32 | rdl;                      \
    uint32_t mul_internal_cycles = mul_clks (rs);        \
     int64_t res = smul32to64 (rm, rs);                  \
             res+= base;                                 \
    uint32_t res_lo = static_cast<uint32_t> (res);       \
    uint32_t res_hi = static_cast<uint32_t> (res >> 32); \
                                                         \
    rdl = res_lo;                                        \
    rdh = res_hi;                                        \
                                                         \
    ticks++; /* 1s */                                    \
    ticks+= mul_internal_cycles; /* mI */                \
    ticks++; /* 1I */                                    \
    ticks++; /* 1I */                                    \
                                                         \
    if ( sign != 0 )                                     \
    { /* set n, z flags */                               \
      zf = res_lo | res_hi;                              \
      nf = res_hi;                                       \
    }                                                    \
    M_ARM7_NEXT_SEQ ();                                  \
  } while (0)

#define M_GET_CPSR() get_cpsr ()
#define M_GET_SPSR() get_spsr ()
           
#define M_READ8(address) 0
#define M_READ16(address) 0
#define M_READ32(address) 0
#define M_WRITE8(address) 0
#define M_WRITE16(address) 0
#define M_WRITE32(address, value) 0                                    
#define M_ARM7_SWP32()                                                   \
  do                                                                     \
  { /* cycles:1S+2N+1I */                                                \
    uint32_t  rn = regs[arm7_opcode >> 16 & 15];                         \
    uint32_t  rm = regs[arm7_opcode & 15];                               \
    uint32_t &rd = regs[arm7_opcode >> 12 & 15];                         \
    uint32_t rot = (rn & 3) << 3;                                        \
                                                                         \
    M_ACCESS_HOOK32 (rn, rm);                                            \
                                                                         \
    if ( mem_bank->fast_rd_mask != 0 )                                   \
    {                                                                    \
      rd = ror32 ( LOAD32x (& mem_bank->mem[mem_bank_address]), rot );   \
    }                                                                    \
    else                                                                 \
    {                                                                    \
      rd = ror32 (M_READ32 (rn));                                        \
    }                                                                    \
                                                                         \
    if ( mem_bank->fast_wr_mask != 0 )                                   \
    {                                                                    \
      STORE32x (& mem_bank->mem[mem_bank_address], rm);                  \
    }                                                                    \
    else                                                                 \
    {                                                                    \
      M_WRITE32 ( rn, rm );                                              \
    }                                                                    \
    ticks += mem_bank->mem_waits[4]; /* no-seq read 32bit wait state */  \
    ticks += mem_bank->mem_waits[4]; /* no-seq write 32bit wait state */ \
                                                                         \
    ticks++; /* 1S */                                                    \
    ticks++; /* 1N */                                                    \
    ticks++; /* 1N */                                                    \
    ticks++; /* 1I */                                                    \
                                                                         \
    M_ARM7_NEXT_SEQ ();                                                  \
  } while (0)

#define M_ARM7_SWP8()                                                    \
  do                                                                     \
  { /* cycles:1S+2N+1I */                                                \
    uint32_t  rn = regs[arm7_opcode >> 16 & 15];                         \
    uint32_t  rm = regs[arm7_opcode & 15];                               \
    uint32_t &rd = regs[arm7_opcode >> 12 & 15];                         \
                                                                         \
    M_ACCESS_HOOK8 (rn, rm);                                             \
                                                                         \
    if ( mem_bank->fast_rd_mask != 0 )                                   \
    {                                                                    \
      rd = mem_bank->mem[mem_bank_address];                              \
    }                                                                    \
    else                                                                 \
    {                                                                    \
      rd = M_READ8 (rn);                                                 \
    }                                                                    \
                                                                         \
    if ( mem_bank->fast_wr_mask != 0 )                                   \
    {                                                                    \
      mem_hooks[0].mem[0] = static_cast<uint8_t> (rm);                   \
    }                                                                    \
    else                                                                 \
    {                                                                    \
      M_WRITE8 ( rn, static_cast<uint8_t> (rm) );                        \
    }                                                                    \
    ticks += mem_bank->mem_waits[0]; /* no-seq read 8bit wait state */   \
    ticks += mem_bank->mem_waits[0]; /* no-seq write 8bit wait state */  \
                                                                         \
    ticks++; /* 1S */                                                    \
    ticks++; /* 1N */                                                    \
    ticks++; /* 1N */                                                    \
    ticks++; /* 1I */                                                    \
                                                                         \
    M_ARM7_NEXT_SEQ ();                                                  \
  } while (0)

#define M_ARM7_MRS_CPSR()                        \
  do                                             \
  { /* cycles:1S */                              \
    uint32_t &rd = regs[arm7_opcode >> 12 & 15]; \
                                                 \
    rd = M_GET_CPSR ();                          \
                                                 \
    ticks++; /* 1S */                            \
                                                 \
    M_ARM7_NEXT_SEQ ();                          \
  } while (0)

#define M_ARM7_MRS_SPSR()                        \
  do                                             \
  { /* cycles:1S */                              \
    uint32_t &rd = regs[arm7_opcode >> 12 & 15]; \
    uint32_t from = mode & 15;                   \
                                                 \
    rd = M_GET_SPSR ();                          \
                                                 \
    ticks++; /* 1S */                            \
                                                 \
    M_ARM7_NEXT_SEQ ();                          \
  } while (0)

#define M_ARM7_MSR_CPSR_(value)                            \
  do                                                       \
  {                                                        \
    uint32_t value_ = value;                               \
    uint32_t to = value_ & 15;                             \
    uint32_t from = mode & 15;                             \
    uint32_t mask = 0;                                     \
                                                           \
    if ((arm7_opcode & 0x10000) != 0)                      \
    {                                                      \
      mask |= 0x000000ff;                                  \
    }                                                      \
                                                           \
    if ((arm7_opcode & 0x20000) != 0)                      \
    {                                                      \
      mask |= 0x0000ff00;                                  \
    }                                                      \
                                                           \
    if ((arm7_opcode & 0x40000) != 0)                      \
    {                                                      \
      mask |= 0x00ff0000;                                  \
    }                                                      \
                                                           \
    if ((arm7_opcode & 0x80000) != 0)                      \
    {                                                      \
      mask |= 0xff000000;                                  \
    }                                                      \
                                                           \
    if ((from & 15) != 0 && (arm7_opcode & 0x10000) != 0)  \
    {                                                      \
      uint32_t *from_ = regs4px[from];                     \
      uint32_t *to_ = regs4px[to];                         \
                                                           \
      from_[0] = regs[13];                                 \
      from_[1] = regs[14];                                 \
                                                           \
      regs[13] = to_[0];                                   \
      regs[14] = to_[1];                                   \
                                                           \
      mode = to | 0x10;                                    \
                                                           \
      thumb_mask = 0;                                      \
      irq_mask = (value_ & 0x80) ? 0 : 0xffffffff;         \
    }                                                      \
    else                                                   \
    {                                                      \
      mask &= 0xff000000;                                  \
    }                                                      \
                                                           \
    if ( (arm7_opcode & 0x80000) == 0 )                    \
    { /* flags bit */                                      \
      nf = value_;                                         \
      zf = value_ & 0x40000000 ^ 0x40000000;                           \
      cf = value_ >> 29;                               \
      of = value_ << 3;                                    \
    }                                                      \
    regs2x[2] &=~mask;                                     \
    regs2x[2] |= value_ & mask;                            \
  } while (0)

#define M_ARM7_MSR_SPSR_(value)               \
  do                                          \
  {                                           \
    uint32_t  value_ = value;                 \
    uint32_t &spsr = regs4px[value_ & 15][2]; \
                                              \
    if ( (arm7_opcode & 0x10000) != 0 )       \
    { /* control bit */                       \
      spsr &= ~0xff;                          \
      spsr |= value_ & 0xff;                  \
      spsr |= 0x10;                           \
    }                                         \
                                              \
    if ( (arm7_opcode & 0x80000) == 0 )       \
    { /* flags bit */                         \
      spsr &= ~0xff000000;                    \
      spsr |= value_ & 0xff000000;            \
    }                                         \
  } while (0)

#define M_ARM7_MSR_CPSR_ROT_IMM32()              \
  do                                             \
  { /* cycles:1S */                              \
    uint32_t imm8 = arm7_opcode & 255;           \
    uint32_t rot = arm7_opcode >> 7 & 0x1e;      \
    uint32_t imm32 = ror32 (imm8, rot);          \
                                                 \
    M_ARM7_MSR_CPSR_ (imm32);                    \
                                                 \
    ticks++; /* 1S */                            \
                                                 \
    M_ARM7_NEXT_SEQ ();                          \
  } while (0)

#define M_ARM7_MSR_CPSR()                        \
  do                                             \
  { /* cycles:1S */                              \
    uint32_t rm = regs[arm7_opcode & 15];        \
                                                 \
    M_ARM7_MSR_CPSR_ (rm);                       \
                                                 \
    ticks++; /* 1S */                            \
                                                 \
    M_ARM7_NEXT_SEQ ();                          \
  } while (0)

#define M_ARM7_MSR_SPSR_ROT_IMM32()              \
  do                                             \
  { /* cycles:1S */                              \
    uint32_t imm8 = arm7_opcode & 255;           \
    uint32_t rot = arm7_opcode >> 7 & 0x1e;      \
    uint32_t imm32 = ror32 (imm8, rot);          \
                                                 \
    M_ARM7_MSR_SPSR_ (imm32);                    \
                                                 \
    ticks++; /* 1S */                            \
                                                 \
    M_ARM7_NEXT_SEQ ();                          \
  } while (0)

#define M_ARM7_MSR_SPSR()                        \
  do                                             \
  { /* cycles:1S */                              \
    uint32_t rm = regs[arm7_opcode & 15];        \
                                                 \
    M_ARM7_MSR_SPSR_ (rm);                       \
                                                 \
    ticks++; /* 1S */                            \
                                                 \
    M_ARM7_NEXT_SEQ ();                          \
  } while (0)

#define M_ARM7_SWI()                     \
  do                                     \
  { /* cycles:2S+1N */                   \
    uint32_t *from = regs4px[mode & 15]; \
    uint32_t *to = regs4px[3];           \
                                         \
    to[2] = M_GET_CPSR ();               \
                                         \
    from[0] = regs[13];                  \
    from[1] = regs[14];                  \
                                         \
    regs[13] = to[0];                    \
    regs[14] = regs[15] - 4;             \
                                         \
    mode = 0x13;                         \
    irq_mask = 0;                        \
    thumb_mask = 0;                      \
                                         \
    regs[15] = 8;                        \
                                         \
    ticks++; /* 1S */                    \
    ticks++; /* 1S */                    \
    ticks++; /* 1N */                    \
                                         \
    bc_size = 0x4000;                    \
    bc_offs = & mmu[0].mem[0];           \
                                         \
    waits[0] =                           \
    waits[1] = 0;                        \
                                         \
    M_ARM7_FLUSH_PIPELINE ();            \
  } while (0)
  


// Branch and Branch with Link (B, BL, BLX_imm)
// Branch (B) is supposed to jump to a subroutine. Branch with Link is meant to be
// used to call to a subroutine, return address is then saved in R14.
//   Bit    Expl.
//   31-28  Condition (must be 1111b for BLX)
//   27-25  Must be "101" for this instruction
//   24     Opcode (0-1) (or Halfword Offset for BLX)
//           0: B{cond} label    ;branch            PC=PC+8+nn*4
//           1: BL{cond} label   ;branch/link       PC=PC+8+nn*4, LR=PC+4
//           H: BLX label ;ARM9  ;branch/link/thumb PC=PC+8+nn*4+H*2, LR=PC+4, T=1
//   23-0   nn - Signed Offset, step 4      (-32M..+32M in steps of 4)
// Branch with Link can be used to 'call' to a sub-routine, which may then
// 'return' by MOV PC,R14 for example.
// Execution Time: 2S + 1N
// Return: No flags affected.
#define M_ARM7_B()                           \
  do                                         \
  { /* cycles:2S+1N */                       \
    int32_t offs = arm7_opcode << 8;         \
            offs >>= 6;                      \
    int32_t pc = regs[15] & -4;              \
            pc+= 4;                          \
                                             \
    regs[15] = pc + offs;                    \
                                             \
    memors &bank = mmu[regs[15] >> 24 & 15]; \
                                             \
    bc_offs = bank.mem_rs;                   \
    bc_offs-= regs[15] & ~bank.mirror_mask;  \
    bc_size = bank.mirror_mask + 1;          \
                                             \
    waits[0] = bank.code_waits[2];           \
    waits[1] = bank.code_waits[3];           \
                                             \
    ticks++; /* 1S */                        \
    ticks++; /* 1S */                        \
    ticks++; /* 1N */                        \
                                             \
    M_ARM7_FLUSH_PIPELINE ();                \
  } while (0)

#define M_ARM7_BL()                          \
  do                                         \
  { /* cycles:2S+1N */                       \
    int32_t offs = arm7_opcode << 8;         \
            offs >>= 6;                      \
    int32_t pc = regs[15] & -4;              \
            pc+= 4;                          \
                                             \
    regs[14] = regs[15];                     \
    regs[15] = pc + offs;                    \
                                             \
    memors &bank = mmu[regs[15] >> 24 & 15]; \
                                             \
    bc_offs = bank.mem_rs;                   \
    bc_offs-= regs[15] & ~bank.mirror_mask;  \
    bc_size = bank.mirror_mask + 1;          \
                                             \
    waits[0] = bank.code_waits[2];           \
    waits[1] = bank.code_waits[3];           \
                                             \
    ticks++; /* 1S */                        \
    ticks++; /* 1S */                        \
    ticks++; /* 1N */                        \
                                             \
    M_ARM7_FLUSH_PIPELINE ();                \
  } while (0)


// Branch and Exchange (BX, BLX_reg)
//   Bit    Expl.
//   31-28  Condition
//   27-8   Must be "0001.0010.1111.1111.1111" for this instruction
//   7-4    Opcode
//           0001b: BX{cond}  Rn    ;PC=Rn, T=Rn.0   (ARMv4T and ARMv5 and up)
//           0010b: BXJ{cond} Rn    ;Change to Jazelle bytecode (ARMv5TEJ and up)
//           0011b: BLX{cond} Rn    ;PC=Rn, T=Rn.0, LR=PC+4     (ARMv5 and up)
//   3-0    Rn - Operand Register  (R0-R14)
// Switching to THUMB Mode: Set Bit 0 of the value in Rn to 1, program continues
// then at Rn-1 in THUMB mode.
// Using BLX R14 is possible (sets PC=Old_LR, and New_LR=retadr).
// Using BX R15 acts as BX $+8 (tested and working on ARM7/ARM9, although it isn't
// officially defined as predictable behaviour).
// Execution Time: 2S + 1N
// Return: No flags affected.
#define M_ARM7_BX()                           \
  do                                          \
  { /* cycles:2S+1N */                        \
    uint32_t rn = regs[arm7_opcode & 15];     \
                                              \
    memors &bank = mmu[rn >> 24 & 15];        \
                                              \
    bc_size = bank.mirror_mask + 1;           \
    bc_offs = bank.mem_rs;                    \
                                              \
    if ( (rn & 1) != 0 )                      \
    { /* thumb mode */                        \
      regs[15] = rn & ~1;                     \
                                              \
      bc_offs-= regs[15] & ~bank.mirror_mask; \
                                              \
      thumb_mask = 0xffffffff;                \
                                              \
      waits[0] = bank.code_waits[0];          \
      waits[1] = bank.code_waits[1];          \
                                              \
      M_THUMB_FLUSH_PIPELINE ();              \
    }                                         \
    else                                      \
    { /* arm7 mode */                         \
      regs[15] = rn & ~3;                     \
                                              \
      bc_offs-= regs[15] & ~bank.mirror_mask; \
                                              \
      thumb_mask = 0;                         \
                                              \
      waits[0] = bank.code_waits[2];          \
      waits[1] = bank.code_waits[3];          \
                                              \
      M_ARM7_FLUSH_PIPELINE ();               \
    }                                         \
    ticks++; /* 1S */                         \
    ticks++; /* 1S */                         \
    ticks++; /* 1N */                         \
  } while (0)

  
// #define M_ARM7_MOV(sign, shifter_oprand)
      // |........x.x.x.x x.x.x.x.......2 ..............1 x.x.x.x.......0|
      // |7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0|
      // |_Cond__|0_0_0|___Op__|S|__Rn___|__Rd___|__Shift__|Typ|0|__Rm___| DataProc
      // |_Cond__|0_0_0|___Op__|S|__Rn___|__Rd___|__Rs___|0|Typ|1|__Rm___| DataProc
      // |_Cond__|0_0_1|___Op__|S|__Rn___|__Rd___|_Shift_|___Immediate___| DataProc
      // |_Cond__|0_0_1_1_0|P|1|0|_Field_|__Rd___|_Shift_|___Immediate___| PSR Imm
      // |_Cond__|0_0_0_1_0|P|L|0|_Field_|__Rd___|0_0_0_0|0_0_0_0|__Rm___| PSR Reg
      // |_Cond__|0_0_0_1_0_0_1_0_1_1_1_1_1_1_1_1_1_1_1_1|0_0|L|1|__Rn___| BX,BLX
      // |_Cond__|0_0_0_0_0_0|A|S|__Rd___|__Rn___|__Rs___|1_0_0_1|__Rm___| Multiply
      // |_Cond__|0_0_0_0_1|U|A|S|_RdHi__|_RdLo__|__Rs___|1_0_0_1|__Rm___| MulLong
      // |_Cond__|0_0_0_1_0|B|0_0|__Rn___|__Rd___|0_0_0_0|1_0_0_1|__Rm___| TransSwp12
      // |_Cond__|0_0_0|P|U|0|W|L|__Rn___|__Rd___|0_0_0_0|1|S|H|1|__Rm___| TransReg10
      // |_Cond__|0_0_0|P|U|1|W|L|__Rn___|__Rd___|OffsetH|1|S|H|1|OffsetL| TransImm10
      // |_Cond__|0_1_0|P|U|B|W|L|__Rn___|__Rd___|_________Offset________| TransImm9
      // |_Cond__|0_1_1|P|U|B|W|L|__Rn___|__Rd___|__Shift__|Typ|0|__Rm___| TransReg9
      // |_Cond__|0_1_1|________________xxx____________________|1|__xxx__| Undefined
      // |_Cond__|1_0_0|P|U|S|W|L|__Rn___|__________Register_List________| BlockTrans
      // |_Cond__|1_0_1|L|___________________Offset______________________| B,BL,BLX
      // |_Cond__|1_1_1_1|_____________Ignored_by_Processor______________| SWI
      switch (arm7_instruction)
      {
      case 0x009: M_ARM7_MUL (0); break; // arm7::mul 
      case 0x019: M_ARM7_MUL (1); break; // arm7::muls
      case 0x029: M_ARM7_MLA (0); break; // arm7::mla 
      case 0x039: M_ARM7_MLA (1); break; // arm7::mlas
      case 0x089: M_ARM7_UMULL (0); break; // arm7::umull 
      case 0x099: M_ARM7_UMULL (1); break; // arm7::umulls
      case 0x0a9: M_ARM7_UMLAL (0); break; // arm7::umlal 
      case 0x0b9: M_ARM7_UMLAL (1); break; // arm7::umlals
      case 0x0c9: M_ARM7_SMULL (0); break; // arm7::smull 
      case 0x0d9: M_ARM7_SMULL (1); break; // arm7::smulls
      case 0x0e9: M_ARM7_SMLAL (0); break; // arm7::smlal 
      case 0x0f9: M_ARM7_SMLAL (1); break; // arm7::smlals
      case 0x109: M_ARM7_SWP32 (); break; // arm7::swp 32bit 
      case 0x149: M_ARM7_SWP8 (); break;  // arm7::swp 8bit 
      case 0x100: M_ARM7_MRS_CPSR (); break; // arm7::mrs cpsr
      case 0x120: M_ARM7_MSR_CPSR (); break; // arm7::msr cpsr reg
      case 0x140: M_ARM7_MRS_SPSR (); break; // arm7::mrs spsr
      case 0x160: M_ARM7_MSR_SPSR (); break; // arm7::msr spsr reg
      case 0x320: case 0x321: case 0x322: case 0x323: // arm7::msr cpsr rot imm32
      case 0x324: case 0x325: case 0x326: case 0x327: // arm7::msr cpsr rot imm32
      case 0x328: case 0x329: case 0x32a: case 0x32b: // arm7::msr cpsr rot imm32
      case 0x32c: case 0x32d: case 0x32e: case 0x32f: // arm7::msr cpsr rot imm32
        M_ARM7_MSR_CPSR_ROT_IMM32 ();
        break;

      case 0x360: case 0x361: case 0x362: case 0x363: // arm7::msr spsr rot imm32
      case 0x364: case 0x365: case 0x366: case 0x367: // arm7::msr spsr rot imm32
      case 0x368: case 0x369: case 0x36a: case 0x36b: // arm7::msr spsr rot imm32
      case 0x36c: case 0x36d: case 0x36e: case 0x36f: // arm7::msr spsr rot imm32
        M_ARM7_MSR_SPSR_ROT_IMM32 ();
        break;

      case 0xf00: case 0xf01: case 0xf02: case 0xf03: // arm7::swi
      case 0xf04: case 0xf05: case 0xf06: case 0xf07: // arm7::swi
      case 0xf08: case 0xf09: case 0xf0a: case 0xf0b: // arm7::swi
      case 0xf0c: case 0xf0d: case 0xf0e:             // arm7::swi
      case 0xf0f:                                     // arm7::swi
      case 0xf10: case 0xf11: case 0xf12: case 0xf13: // arm7::swi
      case 0xf14: case 0xf15: case 0xf16: case 0xf17: // arm7::swi
      case 0xf18: case 0xf19: case 0xf1a: case 0xf1b: // arm7::swi
      case 0xf1c: case 0xf1d: case 0xf1e:             // arm7::swi
      case 0xf1f:                                     // arm7::swi
      case 0xf20: case 0xf21: case 0xf22: case 0xf23: // arm7::swi
      case 0xf24: case 0xf25: case 0xf26: case 0xf27: // arm7::swi
      case 0xf28: case 0xf29: case 0xf2a: case 0xf2b: // arm7::swi
      case 0xf2c: case 0xf2d: case 0xf2e:             // arm7::swi
      case 0xf2f:                                     // arm7::swi
      case 0xf30: case 0xf31: case 0xf32: case 0xf33: // arm7::swi
      case 0xf34: case 0xf35: case 0xf36: case 0xf37: // arm7::swi
      case 0xf38: case 0xf39: case 0xf3a: case 0xf3b: // arm7::swi
      case 0xf3c: case 0xf3d: case 0xf3e:             // arm7::swi
      case 0xf3f:                                     // arm7::swi
      case 0xf40: case 0xf41: case 0xf42: case 0xf43: // arm7::swi
      case 0xf44: case 0xf45: case 0xf46: case 0xf47: // arm7::swi
      case 0xf48: case 0xf49: case 0xf4a: case 0xf4b: // arm7::swi
      case 0xf4c: case 0xf4d: case 0xf4e:             // arm7::swi
      case 0xf4f:                                     // arm7::swi
      case 0xf50: case 0xf51: case 0xf52: case 0xf53: // arm7::swi
      case 0xf54: case 0xf55: case 0xf56: case 0xf57: // arm7::swi
      case 0xf58: case 0xf59: case 0xf5a: case 0xf5b: // arm7::swi
      case 0xf5c: case 0xf5d: case 0xf5e:             // arm7::swi
      case 0xf5f:                                     // arm7::swi
      case 0xf60: case 0xf61: case 0xf62: case 0xf63: // arm7::swi
      case 0xf64: case 0xf65: case 0xf66: case 0xf67: // arm7::swi
      case 0xf68: case 0xf69: case 0xf6a: case 0xf6b: // arm7::swi
      case 0xf6c: case 0xf6d: case 0xf6e:             // arm7::swi
      case 0xf6f:                                     // arm7::swi
      case 0xf70: case 0xf71: case 0xf72: case 0xf73: // arm7::swi
      case 0xf74: case 0xf75: case 0xf76: case 0xf77: // arm7::swi
      case 0xf78: case 0xf79: case 0xf7a: case 0xf7b: // arm7::swi
      case 0xf7c: case 0xf7d: case 0xf7e:             // arm7::swi
      case 0xf7f:                                     // arm7::swi
      case 0xf80: case 0xf81: case 0xf82: case 0xf83: // arm7::swi
      case 0xf84: case 0xf85: case 0xf86: case 0xf87: // arm7::swi
      case 0xf88: case 0xf89: case 0xf8a: case 0xf8b: // arm7::swi
      case 0xf8c: case 0xf8d: case 0xf8e:             // arm7::swi
      case 0xf8f:                                     // arm7::swi
      case 0xf90: case 0xf91: case 0xf92: case 0xf93: // arm7::swi
      case 0xf94: case 0xf95: case 0xf96: case 0xf97: // arm7::swi
      case 0xf98: case 0xf99: case 0xf9a: case 0xf9b: // arm7::swi
      case 0xf9c: case 0xf9d: case 0xf9e:             // arm7::swi
      case 0xf9f:                                     // arm7::swi
      case 0xfa0: case 0xfa1: case 0xfa2: case 0xfa3: // arm7::swi
      case 0xfa4: case 0xfa5: case 0xfa6: case 0xfa7: // arm7::swi
      case 0xfa8: case 0xfa9: case 0xfaa: case 0xfab: // arm7::swi
      case 0xfac: case 0xfad: case 0xfae:             // arm7::swi
      case 0xfaf:                                     // arm7::swi
      case 0xfb0: case 0xfb1: case 0xfb2: case 0xfb3: // arm7::swi
      case 0xfb4: case 0xfb5: case 0xfb6: case 0xfb7: // arm7::swi
      case 0xfb8: case 0xfb9: case 0xfba: case 0xfbb: // arm7::swi
      case 0xfbc: case 0xfbd: case 0xfbe:             // arm7::swi
      case 0xfbf:                                     // arm7::swi
      case 0xfc0: case 0xfc1: case 0xfc2: case 0xfc3: // arm7::swi
      case 0xfc4: case 0xfc5: case 0xfc6: case 0xfc7: // arm7::swi
      case 0xfc8: case 0xfc9: case 0xfca: case 0xfcb: // arm7::swi
      case 0xfcc: case 0xfcd: case 0xfce:             // arm7::swi
      case 0xfcf:                                     // arm7::swi
      case 0xfd0: case 0xfd1: case 0xfd2: case 0xfd3: // arm7::swi
      case 0xfd4: case 0xfd5: case 0xfd6: case 0xfd7: // arm7::swi
      case 0xfd8: case 0xfd9: case 0xfda: case 0xfdb: // arm7::swi
      case 0xfdc: case 0xfdd: case 0xfde:             // arm7::swi
      case 0xfdf:                                     // arm7::swi
      case 0xfe0: case 0xfe1: case 0xfe2: case 0xfe3: // arm7::swi
      case 0xfe4: case 0xfe5: case 0xfe6: case 0xfe7: // arm7::swi
      case 0xfe8: case 0xfe9: case 0xfea: case 0xfeb: // arm7::swi
      case 0xfec: case 0xfed: case 0xfee:             // arm7::swi
      case 0xfef:                                     // arm7::swi
      case 0xff0: case 0xff1: case 0xff2: case 0xff3: // arm7::swi
      case 0xff4: case 0xff5: case 0xff6: case 0xff7: // arm7::swi
      case 0xff8: case 0xff9: case 0xffa: case 0xffb: // arm7::swi
      case 0xffc: case 0xffd: case 0xffe:             // arm7::swi
      case 0xfff:
        M_ARM7_SWI ();
        break;

      case 0xa00: case 0xa01: case 0xa02: case 0xa03: // arm7::b
      case 0xa04: case 0xa05: case 0xa06: case 0xa07: // arm7::b
      case 0xa08: case 0xa09: case 0xa0a: case 0xa0b: // arm7::b
      case 0xa0c: case 0xa0d: case 0xa0e:             // arm7::b
      case 0xa0f:                                     // arm7::b
      case 0xa10: case 0xa11: case 0xa12: case 0xa13: // arm7::b
      case 0xa14: case 0xa15: case 0xa16: case 0xa17: // arm7::b
      case 0xa18: case 0xa19: case 0xa1a: case 0xa1b: // arm7::b
      case 0xa1c: case 0xa1d: case 0xa1e:             // arm7::b
      case 0xa1f:                                     // arm7::b
      case 0xa20: case 0xa21: case 0xa22: case 0xa23: // arm7::b
      case 0xa24: case 0xa25: case 0xa26: case 0xa27: // arm7::b
      case 0xa28: case 0xa29: case 0xa2a: case 0xa2b: // arm7::b
      case 0xa2c: case 0xa2d: case 0xa2e:             // arm7::b
      case 0xa2f:                                     // arm7::b
      case 0xa30: case 0xa31: case 0xa32: case 0xa33: // arm7::b
      case 0xa34: case 0xa35: case 0xa36: case 0xa37: // arm7::b
      case 0xa38: case 0xa39: case 0xa3a: case 0xa3b: // arm7::b
      case 0xa3c: case 0xa3d: case 0xa3e:             // arm7::b
      case 0xa3f:                                     // arm7::b
      case 0xa40: case 0xa41: case 0xa42: case 0xa43: // arm7::b
      case 0xa44: case 0xa45: case 0xa46: case 0xa47: // arm7::b
      case 0xa48: case 0xa49: case 0xa4a: case 0xa4b: // arm7::b
      case 0xa4c: case 0xa4d: case 0xa4e:             // arm7::b
      case 0xa4f:                                     // arm7::b
      case 0xa50: case 0xa51: case 0xa52: case 0xa53: // arm7::b
      case 0xa54: case 0xa55: case 0xa56: case 0xa57: // arm7::b
      case 0xa58: case 0xa59: case 0xa5a: case 0xa5b: // arm7::b
      case 0xa5c: case 0xa5d: case 0xa5e:             // arm7::b
      case 0xa5f:                                     // arm7::b
      case 0xa60: case 0xa61: case 0xa62: case 0xa63: // arm7::b
      case 0xa64: case 0xa65: case 0xa66: case 0xa67: // arm7::b
      case 0xa68: case 0xa69: case 0xa6a: case 0xa6b: // arm7::b
      case 0xa6c: case 0xa6d: case 0xa6e:             // arm7::b
      case 0xa6f:                                     // arm7::b
      case 0xa70: case 0xa71: case 0xa72: case 0xa73: // arm7::b
      case 0xa74: case 0xa75: case 0xa76: case 0xa77: // arm7::b
      case 0xa78: case 0xa79: case 0xa7a: case 0xa7b: // arm7::b
      case 0xa7c: case 0xa7d: case 0xa7e:             // arm7::b
      case 0xa7f:                                     // arm7::b
      case 0xa80: case 0xa81: case 0xa82: case 0xa83: // arm7::b
      case 0xa84: case 0xa85: case 0xa86: case 0xa87: // arm7::b
      case 0xa88: case 0xa89: case 0xa8a: case 0xa8b: // arm7::b
      case 0xa8c: case 0xa8d: case 0xa8e:             // arm7::b
      case 0xa8f:                                     // arm7::b
      case 0xa90: case 0xa91: case 0xa92: case 0xa93: // arm7::b
      case 0xa94: case 0xa95: case 0xa96: case 0xa97: // arm7::b
      case 0xa98: case 0xa99: case 0xa9a: case 0xa9b: // arm7::b
      case 0xa9c: case 0xa9d: case 0xa9e:             // arm7::b
      case 0xa9f:                                     // arm7::b
      case 0xaa0: case 0xaa1: case 0xaa2: case 0xaa3: // arm7::b
      case 0xaa4: case 0xaa5: case 0xaa6: case 0xaa7: // arm7::b
      case 0xaa8: case 0xaa9: case 0xaaa: case 0xaab: // arm7::b
      case 0xaac: case 0xaad: case 0xaae:             // arm7::b
      case 0xaaf:                                     // arm7::b
      case 0xab0: case 0xab1: case 0xab2: case 0xab3: // arm7::b
      case 0xab4: case 0xab5: case 0xab6: case 0xab7: // arm7::b
      case 0xab8: case 0xab9: case 0xaba: case 0xabb: // arm7::b
      case 0xabc: case 0xabd: case 0xabe:             // arm7::b
      case 0xabf:                                     // arm7::b
      case 0xac0: case 0xac1: case 0xac2: case 0xac3: // arm7::b
      case 0xac4: case 0xac5: case 0xac6: case 0xac7: // arm7::b
      case 0xac8: case 0xac9: case 0xaca: case 0xacb: // arm7::b
      case 0xacc: case 0xacd: case 0xace:             // arm7::b
      case 0xacf:                                     // arm7::b
      case 0xad0: case 0xad1: case 0xad2: case 0xad3: // arm7::b
      case 0xad4: case 0xad5: case 0xad6: case 0xad7: // arm7::b
      case 0xad8: case 0xad9: case 0xada: case 0xadb: // arm7::b
      case 0xadc: case 0xadd: case 0xade:             // arm7::b
      case 0xadf:                                     // arm7::b
      case 0xae0: case 0xae1: case 0xae2: case 0xae3: // arm7::b
      case 0xae4: case 0xae5: case 0xae6: case 0xae7: // arm7::b
      case 0xae8: case 0xae9: case 0xaea: case 0xaeb: // arm7::b
      case 0xaec: case 0xaed: case 0xaee:             // arm7::b
      case 0xaef:                                     // arm7::b
      case 0xaf0: case 0xaf1: case 0xaf2: case 0xaf3: // arm7::b
      case 0xaf4: case 0xaf5: case 0xaf6: case 0xaf7: // arm7::b
      case 0xaf8: case 0xaf9: case 0xafa: case 0xafb: // arm7::b
      case 0xafc: case 0xafd: case 0xafe:             // arm7::b
      case 0xaff:                                     // arm7::b
        M_ARM7_B ();
        break;

      case 0xb00: case 0xb01: case 0xb02: case 0xb03: // arm7::bl
      case 0xb04: case 0xb05: case 0xb06: case 0xb07: // arm7::bl
      case 0xb08: case 0xb09: case 0xb0a: case 0xb0b: // arm7::bl
      case 0xb0c: case 0xb0d: case 0xb0e:             // arm7::bl
      case 0xb0f:                                     // arm7::bl
      case 0xb10: case 0xb11: case 0xb12: case 0xb13: // arm7::bl
      case 0xb14: case 0xb15: case 0xb16: case 0xb17: // arm7::bl
      case 0xb18: case 0xb19: case 0xb1a: case 0xb1b: // arm7::bl
      case 0xb1c: case 0xb1d: case 0xb1e:             // arm7::bl
      case 0xb1f:                                     // arm7::bl
      case 0xb20: case 0xb21: case 0xb22: case 0xb23: // arm7::bl
      case 0xb24: case 0xb25: case 0xb26: case 0xb27: // arm7::bl
      case 0xb28: case 0xb29: case 0xb2a: case 0xb2b: // arm7::bl
      case 0xb2c: case 0xb2d: case 0xb2e:             // arm7::bl
      case 0xb2f:                                     // arm7::bl
      case 0xb30: case 0xb31: case 0xb32: case 0xb33: // arm7::bl
      case 0xb34: case 0xb35: case 0xb36: case 0xb37: // arm7::bl
      case 0xb38: case 0xb39: case 0xb3a: case 0xb3b: // arm7::bl
      case 0xb3c: case 0xb3d: case 0xb3e:             // arm7::bl
      case 0xb3f:                                     // arm7::bl
      case 0xb40: case 0xb41: case 0xb42: case 0xb43: // arm7::bl
      case 0xb44: case 0xb45: case 0xb46: case 0xb47: // arm7::bl
      case 0xb48: case 0xb49: case 0xb4a: case 0xb4b: // arm7::bl
      case 0xb4c: case 0xb4d: case 0xb4e:             // arm7::bl
      case 0xb4f:                                     // arm7::bl
      case 0xb50: case 0xb51: case 0xb52: case 0xb53: // arm7::bl
      case 0xb54: case 0xb55: case 0xb56: case 0xb57: // arm7::bl
      case 0xb58: case 0xb59: case 0xb5a: case 0xb5b: // arm7::bl
      case 0xb5c: case 0xb5d: case 0xb5e:             // arm7::bl
      case 0xb5f:                                     // arm7::bl
      case 0xb60: case 0xb61: case 0xb62: case 0xb63: // arm7::bl
      case 0xb64: case 0xb65: case 0xb66: case 0xb67: // arm7::bl
      case 0xb68: case 0xb69: case 0xb6a: case 0xb6b: // arm7::bl
      case 0xb6c: case 0xb6d: case 0xb6e:             // arm7::bl
      case 0xb6f:                                     // arm7::bl
      case 0xb70: case 0xb71: case 0xb72: case 0xb73: // arm7::bl
      case 0xb74: case 0xb75: case 0xb76: case 0xb77: // arm7::bl
      case 0xb78: case 0xb79: case 0xb7a: case 0xb7b: // arm7::bl
      case 0xb7c: case 0xb7d: case 0xb7e:             // arm7::bl
      case 0xb7f:                                     // arm7::bl
      case 0xb80: case 0xb81: case 0xb82: case 0xb83: // arm7::bl
      case 0xb84: case 0xb85: case 0xb86: case 0xb87: // arm7::bl
      case 0xb88: case 0xb89: case 0xb8a: case 0xb8b: // arm7::bl
      case 0xb8c: case 0xb8d: case 0xb8e:             // arm7::bl
      case 0xb8f:                                     // arm7::bl
      case 0xb90: case 0xb91: case 0xb92: case 0xb93: // arm7::bl
      case 0xb94: case 0xb95: case 0xb96: case 0xb97: // arm7::bl
      case 0xb98: case 0xb99: case 0xb9a: case 0xb9b: // arm7::bl
      case 0xb9c: case 0xb9d: case 0xb9e:             // arm7::bl
      case 0xb9f:                                     // arm7::bl
      case 0xba0: case 0xba1: case 0xba2: case 0xba3: // arm7::bl
      case 0xba4: case 0xba5: case 0xba6: case 0xba7: // arm7::bl
      case 0xba8: case 0xba9: case 0xbaa: case 0xbab: // arm7::bl
      case 0xbac: case 0xbad: case 0xbae:             // arm7::bl
      case 0xbaf:                                     // arm7::bl
      case 0xbb0: case 0xbb1: case 0xbb2: case 0xbb3: // arm7::bl
      case 0xbb4: case 0xbb5: case 0xbb6: case 0xbb7: // arm7::bl
      case 0xbb8: case 0xbb9: case 0xbba: case 0xbbb: // arm7::bl
      case 0xbbc: case 0xbbd: case 0xbbe:             // arm7::bl
      case 0xbbf:                                     // arm7::bl
      case 0xbc0: case 0xbc1: case 0xbc2: case 0xbc3: // arm7::bl
      case 0xbc4: case 0xbc5: case 0xbc6: case 0xbc7: // arm7::bl
      case 0xbc8: case 0xbc9: case 0xbca: case 0xbcb: // arm7::bl
      case 0xbcc: case 0xbcd: case 0xbce:             // arm7::bl
      case 0xbcf:                                     // arm7::bl
      case 0xbd0: case 0xbd1: case 0xbd2: case 0xbd3: // arm7::bl
      case 0xbd4: case 0xbd5: case 0xbd6: case 0xbd7: // arm7::bl
      case 0xbd8: case 0xbd9: case 0xbda: case 0xbdb: // arm7::bl
      case 0xbdc: case 0xbdd: case 0xbde:             // arm7::bl
      case 0xbdf:                                     // arm7::bl
      case 0xbe0: case 0xbe1: case 0xbe2: case 0xbe3: // arm7::bl
      case 0xbe4: case 0xbe5: case 0xbe6: case 0xbe7: // arm7::bl
      case 0xbe8: case 0xbe9: case 0xbea: case 0xbeb: // arm7::bl
      case 0xbec: case 0xbed: case 0xbee:             // arm7::bl
      case 0xbef:                                     // arm7::bl
      case 0xbf0: case 0xbf1: case 0xbf2: case 0xbf3: // arm7::bl
      case 0xbf4: case 0xbf5: case 0xbf6: case 0xbf7: // arm7::bl
      case 0xbf8: case 0xbf9: case 0xbfa: case 0xbfb: // arm7::bl
      case 0xbfc: case 0xbfd: case 0xbfe:             // arm7::bl
      case 0xbff:                                     // arm7::bl
        M_ARM7_BL ();
        break;

      case 0x121:                                     // arm7::bx 
        M_ARM7_BX ();
        break;
        
// ._______________________________________________________________.
// |........x.x.x.x x.x.x.x.......2 ..............1 x.x.x.x.......0|
// |7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0|
// |_Cond__|0_0_0|___Op__|S|__Rn___|__Rd___|__Shift__|Typ|0|__Rm___|
// |_Cond__|0_0_0|___Op__|S|__Rn___|__Rd___|__Rs___|0|Typ|1|__Rm___|
// |_Cond__|0_0_1|___Op__|S|__Rn___|__Rd___|_Shift_|___Immediate___|
#define M_ARM7_DATA_PROCESS(packet_)                                       \
  do                                                                       \
  {                                                                        \
    static const size_t packet = packet_;                                  \
    static const size_t command = packet >> 5 & 15;                        \
    static const size_t sign = packet >> 4 & 1;                            \
    static const size_t ror_imm8_part = packet & 15;                       \
    static const size_t shift_imm5_part = packet & 8;                      \
                                                                           \
    uint32_t carry = cf;                                                   \
    uint32_t rhs;                                                          \
    uint32_t lhs = regs[arm7_opcode >> 16 & 15]; /* rn */                  \
    uint32_t rdi = opcode >> 12 & 15; /* rd  */                            \
                                                                           \
    do                                                                     \
    { /* arm_arm.pdf A5.1 Addressing Mode 1 - Data-processing operands */  \
      if ( (packet & 0x200) != 0 )                                         \
      { /* ror 8bit imm */                                                 \
        uint32_t imm8 = opcode & 255;                                      \
        uint32_t rot = opcode >> 7 & 0x1e;                                 \
                                                                           \
        rhs = ror32 (imm8, rot);                                           \
                                                                           \
        if ( ror_imm8_part != 0 || rot != 0 )                              \
        {                                                                  \
          carry = value >> 31;                                             \
        }                                                                  \
      }                                                                    \
      else if ( (packet & 7) == 0 )                                        \
      { /* lsl imm */                                                      \
        uint32_t shift = opcode >> 7 & 0x1f;                               \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        if ( shift_imm5_part != 0 || shift != 0 )                          \
        {                                                                  \
          carry = rm >> (32 - shift);                                      \
          rhs <<= shift;                                                   \
        }                                                                  \
      }                                                                    \
      else if ( (packet & 7) == 2 )                                        \
      { /* lsr imm */                                                      \
        uint32_t shift = (opcode >> 7) - 1 & 0x1f;                         \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        carry = rhs >> shift;                                              \
        rhs = carry >> 1;                                                  \
      }                                                                    \
      else if ( (packet & 7) == 4 )                                        \
      { /* asr imm */                                                      \
        uint32_t shift = (opcode >> 7) - 1 & 0x1f;                         \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        carry = static_cast<int32_t> (rhs) >> shift;                       \
        rhs = static_cast<int32_t> (carry) >> 1;                           \
      }                                                                    \
      else if ( (packet & 7) == 6 )                                        \
      { /* ror imm */                                                      \
        uint32_t shift = opcode >> 7 & 0x1f;                               \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        if ( shift_imm5_part == 0 && VSP_UNLIKELY (shift == 0) )           \
        { /* rcr #1 */                                                     \
          uint32_t carry_ = rhs;                                           \
                                                                           \
          rhs >>= 1;                                                       \
          rhs |= carry << 31;                                              \
                                                                           \
          carry = carry_;                                                  \
          break;                                                           \
        }                                                                  \
        rhs = ror32 (rhs, shift);                                          \
                                                                           \
        carry = rhs >> 31;                                                 \
      }                                                                    \
      else if ( (packet & 7) == 1 )                                        \
      { /* lsl rs */                                                       \
        uint32_t rs = regs[opcode >> 8 & 15] & 0xff;                       \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        ticks++; /* 1I */                                                  \
                                                                           \
        if ( VSP_LIKELY (rs != 0) )                                        \
        {                                                                  \
          if ( rs >= 32 )                                                  \
          {                                                                \
            carry = rhs & (rs - 33) >> 31;                                 \
            rhs = 0;                                                       \
            break;                                                         \
          }                                                                \
          carry = rhs >> (32 - rs);                                        \
          rhs <<= rs;                                                      \
        }                                                                  \
      }                                                                    \
      else if ( (packet & 7) == 3 )                                        \
      { /* lsr rs */                                                       \
        uint32_t rs = regs[opcode >> 8 & 15] & 0xff;                       \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        ticks++; /* 1I */                                                  \
                                                                           \
        if ( VSP_LIKELY (rs != 0) )                                        \
        {                                                                  \
          if ( VSP_UNLIKELY (rs > 32) )                                    \
          {                                                                \
            rs = 32;                                                       \
          }                                                                \
          carry = rhs >> (rs - 1);                                         \
          rhs = carry >> 1;                                                \
        }                                                                  \
      }                                                                    \
      else if ( (packet & 7) == 5 )                                        \
      { /* asr rs */                                                       \
        uint32_t rs = regs[opcode >> 8 & 15] & 0xff;                       \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        ticks++; /* 1I */                                                  \
                                                                           \
        if ( VSP_LIKELY (rs != 0) )                                        \
        {                                                                  \
          if ( VSP_UNLIKELY (rs > 32) )                                    \
          {                                                                \
            rs = 32;                                                       \
          }                                                                \
          carry = static_cast<int32_t> (rhs) >> (rs - 1);                  \
          rhs = static_cast<int32_t> (carry) >> 1;                         \
        }                                                                  \
      }                                                                    \
      else if ( (packet & 7) == 7 )                                        \
      { /* ror rs */                                                       \
        uint32_t rs = regs[opcode >> 8 & 15] & 0xff;                       \
                                                                           \
        rhs = regs[opcode & 15];                                           \
                                                                           \
        ticks++; /* 1I */                                                  \
                                                                           \
        if ( VSP_LIKELY (rs != 0) )                                        \
        {                                                                  \
          rhs = ror32 (rhs, rs & 31);                                      \
                                                                           \
          carry = rhs >> 31;                                               \
        }                                                                  \
      }                                                                    \
      else                                                                 \
      {                                                                    \
        VSP_ASSERT (0);                                                    \
      }                                                                    \
    } while (0);                                                           \
                                                                           \
    if ( command == 2                                                      \
     || command == 10                                                      \
      || command == 6                                                      \
     || command == 3                                                       \
      || command == 7)                                                     \
    { /* sub/cmp/sbc/rsb/rsc */                                            \
      uint64_t dst64;                                                      \
                                                                           \
      if (command != 3 && command != 7)                                    \
      { /* sub/cmp/sbc */                                                  \
        dst64 = static_cast<uint64_t> (lhs) - rhs;                         \
      }                                                                    \
      else                                                                 \
      { /* rsb/rsc */                                                      \
        dst64 = static_cast<uint64_t> (rhs) - lhs;                         \
      }                                                                    \
                                                                           \
      if ( command == 6 || command == 7 )                                  \
      { /* sbc/rsc */                                                      \
        dst64 -= cf & 1 ^ 1;                                               \
      }                                                                    \
                                                                           \
      if (sign != 0 || command == 10)                                      \
      { /* set flags or cmp. */                                            \
        nf = dst;                                                          \
        zf = dst;                                                          \
        cf = dst64 >> 32;                                                  \
        cf^= 1;                                                            \
        of = (lhs ^ rhs) & (lhs ^ dst);                                    \
      }                                                                    \
    }                                                                      \
    else if (command == 4 || command == 11 || command == 5 )               \
    { /* add/cmn/adc */                                                    \
      uint64_t dst64 = static_cast<uint64_t> (lhs) + rhs;                  \
                                                                           \
      if ( command == 5 )                                                  \
      { /* adc. */                                                         \
        dst64 += cf & 1;                                                   \
      }                                                                    \
      dst = static_cast<uint32_t> (dst64);                                 \
                                                                           \
      if (sign != 0 || command == 11)                                      \
      { /* set flags or cmn. */                                            \
        nf = dst;                                                          \
        zf = dst;                                                          \
        cf = dst64 >> 32;                                                  \
        of = ((lhs ^ rhs) ^ 0x80000000) & (lhs ^ dst);                     \
      }                                                                    \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      if (command == 0 || command == 8)                                    \
      { /* and/tst */                                                      \
        dst = lhs & rhs;                                                   \
      }                                                                    \
      else if (command == 1 || command == 9)                               \
      { /* eor/teq */                                                      \
        dst = lhs ^ rhs;                                                   \
      }                                                                    \
      else if (command == 12)                                              \
      { /* orr */                                                          \
        dst = lhs | rhs;                                                   \
      }                                                                    \
      else if (command == 13)                                              \
      { /* move */                                                         \
        dst = rhs;                                                         \
      }                                                                    \
      else if (command == 14)                                              \
      { /* bic */                                                          \
        dst = lhs & ~rhs;                                                  \
      }                                                                    \
      else if (command == 15)                                              \
      { /* mvn */                                                          \
        dst = ~rhs;                                                        \
      }                                                                    \
      else                                                                 \
      {                                                                    \
        VSP_ASSERT (0);                                                    \
      }                                                                    \
                                                                           \
      if (sign != 0 || command == 8 || command == 9)                       \
      { /* sign or tst, teq. */                                            \
        nf = dst;                                                          \
        zf = dst;                                                          \
        cf = carry;                                                        \
      }                                                                    \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      VSP_ASSERT (0);                                                      \
    }                                                                      \
                                                                           \
    if ( command == 8                                                      \
     || command == 9                                                       \
      || command == 10                                                     \
     || command == 11 )                                                    \
    { /* tst/teq/cmp/cmn */                                                \
      ticks++; /* 1S */                                                    \
                                                                           \
      M_ARM7_NEXT_SEQ ();                                                  \
    }                                                                      \
    else                                                                   \
    {                                                                      \
      regs[rdi] = dst;                                                     \
                                                                           \
      if (rdi == 15)                                                       \
      {                                                                    \
        if (sign != 0)                                                     \
        { /* spsr to cpsr */                                               \
          if ( (mode & 15) != 0 )                                          \
          {                                                                \
            uint32_t spsr = get_spsr ();                                   \
            uint32_t *from = regs4px[mode & 15];                           \
            uint32_t *to = regs4px[spsr & 15];                             \
                                                                           \
            from[0] = regs[13];                                            \
            from[1] = regs[14];                                            \
                                                                           \
            regs[13] = to[0];                                              \
            regs[14] = to[1];                                              \
                                                                           \
            regs2x[2] = spsr;                                              \
                                                                           \
            mode = spsr & 15 | 0x10;                                       \
                                                                           \
            thumb_mask = (spsr & 0x20) ? 0 : 0xffffffff;                   \
                                                                           \
            irq_mask = (spsr & 0x80) ? 0 : 0xffffffff;                     \
                                                                           \
            nf = spsr;                                                     \
            zf = spsr & 0x40000000 ^ 0x40000000;                           \
            cf = spsr >> 29;                                               \
            of = spsr << 3;                                                \
                                                                           \
            if (thumb_mask != 0)                                           \
            {                                                              \
              regs[15] &= ~1;                                              \
                                                                           \
              M_THUMB_FLUSH_PIPELINE ();                                   \
            }                                                              \
            else                                                           \
            {                                                              \
              regs[15] &= ~3;                                              \
                                                                           \
              M_ARM7_FLUSH_PIPELINE ();                                    \
            }                                                              \
          }                                                                \
          else                                                             \
          { /* ub... */                                                    \
            VSP_ASSERT (0);                                                \
          }                                                                \
        }                                                                  \
        else                                                               \
        {                                                                  \
          regs[15] &= ~3;                                                  \
                                                                           \
          M_ARM7_FLUSH_PIPELINE ();                                        \
        }                                                                  \
        ticks++; /* 1S */                                                  \
        ticks++; /* 1S */                                                  \
        ticks++; /* 1N */                                                  \
      }                                                                    \
      else                                                                 \
      { /* not r15. */                                                     \
        ticks++; /* 1S */                                                  \
                                                                           \
        M_ARM7_NEXT_SEQ ();                                                \
      }                                                                    \
    }                                                                      \
  } while (0)

#define M_ARM7_ALU_S(alu_command)                                                            \
  case (alu_command << 5 | 0x010): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x010)); break;  \
  case (alu_command << 5 | 0x012): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x012)); break;  \
  case (alu_command << 5 | 0x014): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x014)); break;  \
  case (alu_command << 5 | 0x016): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x016)); break;  \
  case (alu_command << 5 | 0x018): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x018)); break;  \
  case (alu_command << 5 | 0x01a): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x01a)); break;  \
  case (alu_command << 5 | 0x01c): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x01c)); break;  \
  case (alu_command << 5 | 0x01e): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x01e)); break;  \
  case (alu_command << 5 | 0x011): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x011)); break;  \
  case (alu_command << 5 | 0x013): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x013)); break;  \
  case (alu_command << 5 | 0x015): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x015)); break;  \
  case (alu_command << 5 | 0x017): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x017)); break;  \
  case (alu_command << 5 | 0x210): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x210)); break;  \
  case (alu_command << 5 | 0x211): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x211)); break;  \
  case (alu_command << 5 | 0x212): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x212)); break;  \
  case (alu_command << 5 | 0x213): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x213)); break;  \
  case (alu_command << 5 | 0x214): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x214)); break;  \
  case (alu_command << 5 | 0x215): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x215)); break;  \
  case (alu_command << 5 | 0x216): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x216)); break;  \
  case (alu_command << 5 | 0x217): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x217)); break;  \
  case (alu_command << 5 | 0x218): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x218)); break;  \
  case (alu_command << 5 | 0x219): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x219)); break;  \
  case (alu_command << 5 | 0x21a): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x21a)); break;  \
  case (alu_command << 5 | 0x21b): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x21b)); break;  \
  case (alu_command << 5 | 0x21c): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x21c)); break;  \
  case (alu_command << 5 | 0x21d): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x21d)); break;  \
  case (alu_command << 5 | 0x21e): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x21e)); break;  \
  case (alu_command << 5 | 0x21f): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x21f)); break;

#define M_ARM7_ALU_N(alu_command)                                                            \
  case (alu_command << 5 | 0x000): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x000)); break;  \
  case (alu_command << 5 | 0x002): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x002)); break;  \
  case (alu_command << 5 | 0x004): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x004)); break;  \
  case (alu_command << 5 | 0x006): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x006)); break;  \
  case (alu_command << 5 | 0x008): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x008)); break;  \
  case (alu_command << 5 | 0x00a): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x00a)); break;  \
  case (alu_command << 5 | 0x00c): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x00c)); break;  \
  case (alu_command << 5 | 0x00e): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x00e)); break;  \
  case (alu_command << 5 | 0x001): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x001)); break;  \
  case (alu_command << 5 | 0x003): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x003)); break;  \
  case (alu_command << 5 | 0x005): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x005)); break;  \
  case (alu_command << 5 | 0x007): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x007)); break;  \
  case (alu_command << 5 | 0x200): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x200)); break;  \
  case (alu_command << 5 | 0x201): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x201)); break;  \
  case (alu_command << 5 | 0x202): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x202)); break;  \
  case (alu_command << 5 | 0x203): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x203)); break;  \
  case (alu_command << 5 | 0x204): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x204)); break;  \
  case (alu_command << 5 | 0x205): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x205)); break;  \
  case (alu_command << 5 | 0x206): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x206)); break;  \
  case (alu_command << 5 | 0x207): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x207)); break;  \
  case (alu_command << 5 | 0x208): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x208)); break;  \
  case (alu_command << 5 | 0x209): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x209)); break;  \
  case (alu_command << 5 | 0x20a): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x20a)); break;  \
  case (alu_command << 5 | 0x20b): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x20b)); break;  \
  case (alu_command << 5 | 0x20c): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x20c)); break;  \
  case (alu_command << 5 | 0x20d): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x20d)); break;  \
  case (alu_command << 5 | 0x20e): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x20e)); break;  \
  case (alu_command << 5 | 0x20f): M_ARM7_DATA_PROCESS ((alu_command << 5 | 0x20f)); break;  

      M_ARM7_ALU_S (0)       M_ARM7_ALU_N (0)
      M_ARM7_ALU_S (1)    M_ARM7_ALU_N (1)
      M_ARM7_ALU_S (2)     M_ARM7_ALU_N (2)
      M_ARM7_ALU_S (3)     M_ARM7_ALU_N (3)
      M_ARM7_ALU_S (4)     M_ARM7_ALU_N (4)
      M_ARM7_ALU_S (5)     M_ARM7_ALU_N (5)
      M_ARM7_ALU_S (6)     M_ARM7_ALU_N (6)
      M_ARM7_ALU_S (7)      M_ARM7_ALU_N (7)
      M_ARM7_ALU_S (8)
      M_ARM7_ALU_S (9)
      M_ARM7_ALU_S (10)
      M_ARM7_ALU_S (11)
      M_ARM7_ALU_S (12)  M_ARM7_ALU_N (12)
      M_ARM7_ALU_S (13)  M_ARM7_ALU_N (13)
      M_ARM7_ALU_S (14)  M_ARM7_ALU_N (14)
      M_ARM7_ALU_S (15)  M_ARM7_ALU_N (15)

      }



L1:;
L9:; // breakpoint hit.
     


    }
  }
  
  
// 0:ldrb
// 1:ldrsb
// 2:ldrh 
// 3:ldrsh 
// 4:ldr
// 5:strb 
// 6:strh 
// 7:str 
// ._______________________________________________________________.
// |........x.x.x.x x.x.x.x.......2 ..............1 x.x.x.x.......0|
// |7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0|
// |_Cond__|0_0_0|P|U|0|W|L|__Rn___|__Rd___|0_0_0_0|1|S|H|1|__Rm___| ldrh/strh/ldrsb/ldrsh
// |_Cond__|0_0_0|P|U|1|W|L|__Rn___|__Rd___|OffsetH|1|S|H|1|OffsetL| ldrh/strh/ldrsb/ldrsh
// |_Cond__|0_1_0|P|U|B|W|L|__Rn___|__Rd___|_________Offset________| ldr/str/ldrb/strb
// |_Cond__|0_1_1|P|U|B|W|L|__Rn___|__Rd___|__Shift__|Typ|0|__Rm___| ldr/str/ldrb/strb

#define M_ARM7_MACCESS_HALF(packet_, opcode_)                                              \
  do                                                                                       \
  {                                                                                        \
    static const size_t packet = packet_;                                                  \
                                                                                           \
    static const size_t field_w = packet & 0x020;                /* 0:ldrb  */             \
    static const size_t field_p = packet & 0x100;                /* 1:ldrsb */             \
    static const size_t field_l = packet & 0x010;                /* 2:ldrh  */             \
    static const size_t field_s = packet & 0x004;                /* 3:ldrsh */             \
    static const size_t field_h = packet & 0x002;                /* 4:ldr   */             \
    static const size_t field_b = packet & 0x040;                /* 5:strb  */             \
    static const size_t field_u = packet & 0x080;                /* 6:strh  */             \
    static const size_t field_lsh = field_l | field_s | field_h; /* 7:str   */             \
                                                                                           \
    static const size_t strh_mask  = (field_lsh == 0x002) ? 6 : 0x80;                      \
    static const size_t ldrh_mask  = (field_lsh == 0x012) ? 2 : 0x80;                      \
    static const size_t ldrsb_mask = (field_lsh == 0x014) ? 1 : 0x80;                      \
    static const size_t ldrsh_mask = (field_lsh == 0x016) ? 3 : 0x80;                      \
    static const size_t strh_or_ldrh  = (strh_mask < 8) ? strh_mask : ldrh_mask;           \
    static const size_t ldrsb_or_ldrsh  = (ldrsb_mask < 8) ? ldrsb_mask : ldrsh_mask;      \
    static const size_t command  = (strh_or_ldrh < 8) ? strh_or_ldrh : ldrsb_or_ldrsh;     \
    static const size_t write_mask = (command >= 5) ? SIZE_MAX : 0;                        \
                                                                                           \
    static const size_t size8_mask = (command <= 1 || command == 5) ? 1 : 0;                  \
    static const size_t size16_mask= (command == 2 || command == 3 || command == 6) ? 2 : 0;  \
                                                                             \
    uint32_t opcode = opcode_;                                               \
    uint32_t rdi = opcode >> 12 & 15;                                        \
    uint32_t rni = opcode >> 16 & 15;                                        \
    uint32_t offs;                                                           \
    uint32_t t;                                                              \
    uint32_t rn_wb;                                                          \
    uint32_t *rnip; /* write back pointer */                                 \
    uint32_t rw_addr;                                                        \
    uint32_t rn_write;                                                       \
                                                                             \
    if ( field_b != 0 )                                                      \
    { /* imm8bit */                                                          \
      offs = opcode & 0x0f;                                                  \
      offs|= opcode >> 4 & 0xf0;                                             \
    }                                                                        \
    else                                                                     \
    { /* rm */                                                               \
      offs = regs[opcode & 15];                                              \
    }                                                                        \
                                                                             \
    if ( field_p == 0 )                                                      \
    { /* rn++, write back */                                                 \
      rnip = & regs[rn];                                                     \
      rw_addr = rnip[0];                                                     \
      rn_wb = (field_u != 0) ? (rw_addr + offs) : (rw_addr - offs);          \
    }                                                                        \
    else if ( field_w == 0 )                                                 \
    { /* ++rn, none */                                                       \
      rnip = & t; /* dummy...*/                                              \
      rw_addr = (field_u != 0) ? (regs[rn] + offs) : (regs[rn] - offs);      \
      rn_wb = 0;                                                             \
    }                                                                        \
    else                                                                     \
    { /* ++rn, write back */                                                 \
      rnip = & regs[rn];                                                     \
      rw_addr = (field_u != 0) ? (rnip[0] + offs) : (rnip[0] - offs);        \
      rn_wb = rw_addr;                                                       \
    }                                                                        \
                                                                             \
    if (write_mask != 0)                                                     \
    {                                                                        \
      rn_write = regs[rdi];                                                  \
                                                                             \
      /* NO$GBA ARM Opcodes:Memory:Single Data Transfer (LDR, STR, PLD) */   \
      rn_write += (rdi + 1) >> 2 & 4;                                        \
                                                                             \
      if (field_w != 0 && field_p != 0 && rdi == rni)                        \
      { /* ++rn, write back && rd = rn */                                    \
        rn_write = rw_addr;                                                  \
      }                                                                      \
      M_WRITE_HOOK16 (rw_addr, rn_write)                                     \
                                                                             \
      if ( mem_bank->fast_wr_mask != 0 )                                     \
      {                                                                      \
        STORE16x (& mem_bank->mem[mem_bank_address]                          \
                , static_cast<uint16_t> (rn_write) );                        \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        M_WRITE16 ( rw_addr, rn_write );                                     \
      }                                                                      \
      ticks += mem_bank->mem_waits[2]; /* no-seq write 16bit wait state */   \
                                                                             \
      rnip[0] = rn_wb;                                                       \
                                                                             \
      ticks++; /* 1N */                                                      \
      ticks++; /* 1N */                                                      \
                                                                             \
      M_ARM7_NEXT_NOSEQ ();                                                  \
    }                                                                        \
    else                                                                     \
    {                                                                        \
      if (rdi == rni)                                                        \
      {                                                                      \
        rnip = & t; /* write back discard...*/                               \
      }                                                                      \
                                                                             \
      if (size8_mask != 0)                                                   \
      {                                                                      \
        M_READ_HOOK8 (rw_addr)                                               \
                                                                             \
        if ( command == 0 )                                                  \
        { /* ldrb */                                                         \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            regs[rdi] = mem_bank->mem[mem_bank_address];                     \
          }                                                                  \
          else                                                               \
          {                                                                  \
            regs[rdi] = M_READ8 (rw_addr);                                   \
          }                                                                  \
        }                                                                    \
        else                                                                 \
        { /* ldrsb */                                                        \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                  \
              static_cast<int8_t> ( mem_bank->mem[mem_bank_address] );       \
          }                                                                  \
          else                                                               \
          {                                                                  \
            reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                  \
             static_cast<int8_t> (                                           \
              static_cast<uint8_t> ( M_READ8 (rw_addr) ) );                  \
          }                                                                  \
        }                                                                    \
        ticks += mem_bank->mem_waits[0]; /* no-seq read 8bit wait state */   \
      }                                                                      \
      else if (size16_mask != 0)                                             \
      {                                                                      \
        M_READ_HOOK16 (rw_addr)                                              \
                                                                             \
        if ( command == 2 )                                                  \
        { /* ldrh */                                                         \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            regs[rdi] = LOAD16x ( & mem_bank->mem[mem_bank_address] );       \
          }                                                                  \
          else                                                               \
          {                                                                  \
            regs[rdi] = M_READ16 (rw_addr);                                  \
          }                                                                  \
                                                                             \
          if ( (rw_addr & 1) != 0 )                                          \
          { /* unaligned */                                                  \
            regs[rdi] = ror32 (regs[rdi], 8);                                \
          }                                                                  \
        }                                                                    \
        else if ( command == 3 )                                             \
        { /* ldrsh */                                                        \
          if ( (rw_addr & 1) != 0 )                                          \
          { /* unaligned, same as ldrsb */                                   \
            if ( mem_bank->fast_rd_mask != 0 )                               \
            {                                                                \
              reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                \
                static_cast<int8_t> ( mem_bank->mem[mem_bank_address] );     \
            }                                                                \
            else                                                             \
            {                                                                \
              reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                \
               static_cast<int8_t> (                                         \
                static_cast<uint8_t> ( M_READ8 (rw_addr) ) );                \
            }                                                                \
          }                                                                  \
          else                                                               \
          {                                                                  \
            if ( mem_bank->fast_rd_mask != 0 )                               \
            {                                                                \
              regs[rdi] = LOAD16x ( & mem_bank->mem[mem_bank_address] );     \
            }                                                                \
            else                                                             \
            {                                                                \
              regs[rdi] = M_READ16 (rw_addr);                                \
            }                                                                \
          }                                                                  \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          VSP_ASSERT (0);                                                    \
        }                                                                    \
        ticks += mem_bank->mem_waits[2]; /* no-seq read 16bit wait state */  \
      }                                                                      \
      ticks++; /* 1S */                                                      \
      ticks++; /* 1N */                                                      \
      ticks++; /* 1I */                                                      \
                                                                             \
      rnip[0] = rn_wb;                                                       \
                                                                             \
      if (rdi == 15)                                                         \
      { /* flush pipeline */                                                 \
        ticks++; /* 1S */                                                    \
        ticks++; /* 1N */                                                    \
                                                                             \
        M_ARM7_FLUSH_PIPELINE ();                                            \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        M_ARM7_NEXT_SEQ ();                                                  \
      }                                                                      \
    }                                                                        \
  } while (0)

#define M_ARM7_ACCESS(packet_, opcode_)                                             \
  do                                                                                       \
  {                                                                                        \
    static const size_t packet = packet_;                                                  \
                                                                                           \
    static const size_t field_w = packet & 0x020;                /* 0:ldrb  */             \
    static const size_t field_p = packet & 0x100;                /* 1:ldrsb */             \
    static const size_t field_l = packet & 0x010;                /* 2:ldrh  */             \
    static const size_t field_s = packet & 0x004;                /* 3:ldrsh */             \
    static const size_t field_h = packet & 0x002;                /* 4:ldr   */             \
    static const size_t field_b = packet & 0x040;                /* 5:strb  */             \
    static const size_t field_u = packet & 0x080;                /* 6:strh  */             \
    static const size_t field_lsh = field_l | field_s | field_h; /* 7:str   */             \
                                                                                           \
    static const size_t shift_imm5_part = packet & 8;                                      \
    static const size_t command  =    (field_l != 0)                                       \
                                    ? ((field_b != 0) ? 0 : 4)                             \
                                     : ((field_b != 0) ? 5 : 7);                           \
    static const size_t write_mask = (command >= 5) ? SIZE_MAX : 0;                        \
                                                                                           \
    static const size_t size8_mask = (command <= 1 || command == 5) ? 1 : 0;                  \
    static const size_t size32_mask = (command == 4 || command == 7) ? 4 : 0;                 \
                                                                             \
    uint32_t opcode = opcode_;                                               \
    uint32_t rdi = opcode >> 12 & 15;                                        \
    uint32_t rni = opcode >> 16 & 15;                                        \
    uint32_t offs;                                                           \
    uint32_t t;                                                              \
    uint32_t rn_wb;                                                          \
    uint32_t *rnip; /* write back pointer */                                 \
    uint32_t rw_addr;                                                        \
    uint32_t rn_write;                                                       \
                                                                             \
    if ( (packet & 0xe00) == 0x600 )                                         \
    { /* shift + imm5bit */                                                  \
      do                                                                     \
      {                                                                      \
        if ( (packet & 7) == 0 )                                             \
        { /* lsl imm */                                                      \
          uint32_t shift = opcode >> 7 & 0x1f;                               \
                                                                             \
          offs = regs[opcode & 15] << shift;                                 \
        }                                                                    \
        else if ( (packet & 7) == 2 )                                        \
        { /* lsr imm */                                                      \
          uint32_t shift = (opcode >> 7) - 1 & 0x1f;                         \
                                                                             \
          offs = regs[opcode & 15];                                          \
          offs >>= shift;                                                    \
          offs >>= 1;                                                        \
        }                                                                    \
        else if ( (packet & 7) == 4 )                                        \
        { /* asr imm */                                                      \
          uint32_t shift = (opcode >> 7) - 1 & 0x1f;                         \
                                                                             \
          offs = regs[opcode & 15];                                          \
          offs = static_cast<int32_t> (offs) >> shift;                       \
          offs = static_cast<int32_t> (offs) >> 1;                           \
        }                                                                    \
        else if ( (packet & 7) == 6 )                                        \
        { /* ror imm */                                                      \
          uint32_t shift = opcode >> 7 & 0x1f;                               \
                                                                             \
          offs = regs[opcode & 15];                                          \
                                                                             \
          if ( shift_imm5_part == 0 && VSP_UNLIKELY (shift == 0) )           \
          { /* rcr #1 */                                                     \
            offs >>= 1;                                                      \
            offs |= cf << 31;                                                \
            break;                                                           \
          }                                                                  \
          offs = ror32 (offs, shift);                                        \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          VSP_ASSERT (0);                                                    \
        }                                                                    \
      } while (0);                                                           \
    }                                                                        \
    else if ( (packet & 0xe00) == 0x400 )                                    \
    { /* imm12bit */                                                         \
      offs = opcode & 0xfff;                                                 \
    }                                                                        \
    else                                                                     \
    {                                                                        \
      VSP_ASSERT (0);                                                        \
    }                                                                        \
                                                                             \
    if ( field_p == 0 )                                                      \
    { /* rn++, write back (t suffix) */                                      \
      rnip = & regs[rn];                                                     \
      rw_addr = rnip[0];                                                     \
      rn_wb = (field_u != 0) ? (rw_addr + offs) : (rw_addr - offs);          \
    }                                                                        \
    else if ( field_w == 0 )                                                 \
    { /* ++rn, none */                                                       \
      rnip = & t; /* dummy...*/                                              \
      rw_addr = (field_u != 0) ? (regs[rn] + offs) : (regs[rn] - offs);      \
      rn_wb = 0;                                                             \
    }                                                                        \
    else                                                                     \
    { /* ++rn, write back */                                                 \
      rnip = & regs[rn];                                                     \
      rw_addr = (field_u != 0) ? (rnip[0] + offs) : (rnip[0] - offs);        \
      rn_wb = rw_addr;                                                       \
    }                                                                        \
                                                                             \
    if (write_mask != 0)                                                     \
    {                                                                        \
      rn_write = regs[rdi];                                                  \
                                                                             \
      /* NO$GBA ARM Opcodes:Memory:Single Data Transfer (LDR, STR, PLD) */   \
      rn_write += (rdi + 1) >> 2 & 4;                                        \
                                                                             \
      if (field_w != 0 && field_p != 0 && rdi == rni)                        \
      { /* ++rn, write back && rd = rn */                                    \
        rn_write = rw_addr;                                                  \
      }                                                                      \
                                                                             \
      if (size8_mask != 0)                                                   \
      {                                                                      \
        M_WRITE_HOOK8 (rw_addr, rn_write)                                    \
                                                                             \
        if ( mem_bank->fast_wr_mask != 0 )                                   \
        {                                                                    \
          mem_bank->mem[mem_bank_address] =                                  \
            static_cast<uint8_t> (rn_write);                                 \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          M_WRITE8 ( rw_addr, static_cast<uint8_t> (rn_write) );             \
        }                                                                    \
        ticks += mem_bank->mem_waits[0]; /* no-seq write 8bit wait state */  \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        VSP_ASSERT (size32_mask != 0);                                       \
                                                                             \
        M_WRITE_HOOK32 (rw_addr, rn_write)                                   \
                                                                             \
        if ( mem_bank->fast_wr_mask != 0 )                                   \
        {                                                                    \
          STORE32x (& mem_bank->mem[mem_bank_address], rn_write );           \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          M_WRITE32 ( rw_addr, rn_write );                                   \
        }                                                                    \
        ticks += mem_bank->mem_waits[4]; /* no-seq write 32bit wait state */ \
      }                                                                      \
      rnip[0] = rn_wb;                                                       \
                                                                             \
      ticks++; /* 1N */                                                      \
      ticks++; /* 1N */                                                      \
                                                                             \
      M_ARM7_NEXT_NOSEQ ();                                                  \
    }                                                                        \
    else                                                                     \
    {                                                                        \
      if (rdi == rni)                                                        \
      {                                                                      \
        rnip = & t; /* write back discard...*/                               \
      }                                                                      \
                                                                             \
      if (size8_mask != 0)                                                   \
      {                                                                      \
        M_READ_HOOK8 (rw_addr)                                               \
                                                                             \
        if ( command == 0 )                                                  \
        { /* ldrb */                                                         \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            regs[rdi] = mem_bank->mem[mem_bank_address];                     \
          }                                                                  \
          else                                                               \
          {                                                                  \
            regs[rdi] = M_READ8 (rw_addr);                                   \
          }                                                                  \
        }                                                                    \
        else                                                                 \
        { /* ldrsb */                                                        \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                  \
              static_cast<int8_t> ( mem_bank->mem[mem_bank_address] );       \
          }                                                                  \
          else                                                               \
          {                                                                  \
            reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                  \
             static_cast<int8_t> (                                           \
              static_cast<uint8_t> ( M_READ8 (rw_addr) ) );                  \
          }                                                                  \
        }                                                                    \
        ticks += mem_bank->mem_waits[0]; /* no-seq read 8bit wait state */   \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        VSP_ASSERT (size32_mask != 0);                                       \
                                                                             \
        M_READ_HOOK32 (rw_addr)                                              \
                                                                             \
        if ( mem_bank->fast_rd_mask != 0 )                                   \
        {                                                                    \
          regs[rdi] = ror32 ( LOAD32x (& mem_bank->mem[mem_bank_address]), ((rw_addr & 3) << 3) );   \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          regs[rdi] = ror32 (M_READ32 (rn), ((rw_addr & 3) << 3));           \
        }                                                                    \
        ticks += mem_bank->mem_waits[4]; /* no-seq read 32bit wait state */  \
      }                                                                      \
      ticks++; /* 1S */                                                      \
      ticks++; /* 1N */                                                      \
      ticks++; /* 1I */                                                      \
                                                                             \
      rnip[0] = rn_wb;                                                       \
                                                                             \
      if (rdi == 15)                                                         \
      { /* flush pipeline */                                                 \
        ticks++; /* 1S */                                                    \
        ticks++; /* 1N */                                                    \
                                                                             \
        M_ARM7_FLUSH_PIPELINE ();                                            \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        M_ARM7_NEXT_SEQ ();                                                  \
      }                                                                      \
    }                                                                        \
  } while (0)

#define M_ARM7_ACCESS_SINGLE(packet_, opcode_)                                             \
  do                                                                                       \
  {                                                                                        \
    static const size_t packet = packet_;                                                  \
                                                                                           \
    static const size_t field_w = packet & 0x020;                /* 0:ldrb  */             \
    static const size_t field_p = packet & 0x100;                /* 1:ldrsb */             \
    static const size_t field_l = packet & 0x010;                /* 2:ldrh  */             \
    static const size_t field_s = packet & 0x004;                /* 3:ldrsh */             \
    static const size_t field_h = packet & 0x002;                /* 4:ldr   */             \
    static const size_t field_b = packet & 0x040;                /* 5:strb  */             \
    static const size_t field_u = packet & 0x080;                /* 6:strh  */             \
    static const size_t field_lsh = field_l | field_s | field_h; /* 7:str   */             \
    static const size_t field_lb = field_l | field_b;                                      \
                                                                                           \
    static const size_t shift_imm5_part = packet & 8;                                      \
    static const size_t strh_mask  = (field_lsh == 0x002) ? 6 : 0x80;                      \
    static const size_t ldrh_mask  = (field_lsh == 0x012) ? 2 : 0x80;                      \
    static const size_t ldrsb_mask = (field_lsh == 0x014) ? 1 : 0x80;                      \
    static const size_t ldrsh_mask = (field_lsh == 0x016) ? 3 : 0x80;                      \
    static const size_t strh_or_ldrh  = (strh_mask < 8) ? strh_mask : ldrh_mask;           \
    static const size_t ldrsb_or_ldrsh  = (ldrsb_mask < 8) ? ldrsb_mask : ldrsh_mask;      \
    static const size_t rside_res  = (strh_or_ldrh < 8) ? strh_or_ldrh : ldrsb_or_ldrsh;   \
    static const size_t lside_res  =    (field_l != 0)                                     \
                                    ? ((field_b != 0) ? 0 : 4)                             \
                                     : ((field_b != 0) ? 5 : 7);                           \
    static const size_t command = ((packet & 0xe00) == 0) ? rside_res : lside_res;         \
    static const size_t write_mask = (command >= 5) ? SIZE_MAX : 0;                        \
    static const size_t wb_mask = (field_w == 0 && field_p != 0) ? 0 : SIZE_MAX;                        \
                                                                                           \
    static const size_t size8_mask = (command <= 1 || command == 5) ? 1 : 0x80;                  \
    static const size_t size16_mask= (command == 2 || command == 3 || command == 6) ? 2 : 0x80;  \
    static const size_t size32_mask = (command == 4 || command == 7) ? 4 : 0x80;                 \
    static const size_t size8or16_mask = (size8_mask < 8) ? size8_mask : size16_mask;            \
    static const size_t size_mask = (size32_mask < 8) ? size32_mask : size8or16_mask;        \
                                                                             \
    uint32_t opcode = opcode_;                                               \
    uint32_t rdi = opcode >> 12 & 15;                                        \
    uint32_t rni = opcode >> 16 & 15;                                        \
    uint32_t offs;                                                           \
    uint32_t t;                                                              \
    uint32_t rn_wb;                                                          \
    uint32_t *rnip; /* write back pointer */                                 \
    uint32_t rw_addr;                                                        \
    uint32_t rn_write;                                                       \
                                                                             \
    if ( (packet & 0xe00) == 0x600 )                                         \
    { /* shift + imm5bit */                                                  \
      do                                                                     \
      {                                                                      \
        if ( (packet & 7) == 0 )                                             \
        { /* lsl imm */                                                      \
          uint32_t shift = opcode >> 7 & 0x1f;                               \
                                                                             \
          offs = regs[opcode & 15] << shift;                                 \
        }                                                                    \
        else if ( (packet & 7) == 2 )                                        \
        { /* lsr imm */                                                      \
          uint32_t shift = (opcode >> 7) - 1 & 0x1f;                         \
                                                                             \
          offs = regs[opcode & 15];                                          \
          offs >>= shift;                                                    \
          offs >>= 1;                                                        \
        }                                                                    \
        else if ( (packet & 7) == 4 )                                        \
        { /* asr imm */                                                      \
          uint32_t shift = (opcode >> 7) - 1 & 0x1f;                         \
                                                                             \
          offs = regs[opcode & 15];                                          \
          offs = static_cast<int32_t> (offs) >> shift;                       \
          offs = static_cast<int32_t> (offs) >> 1;                           \
        }                                                                    \
        else if ( (packet & 7) == 6 )                                        \
        { /* ror imm */                                                      \
          uint32_t shift = opcode >> 7 & 0x1f;                               \
                                                                             \
          offs = regs[opcode & 15];                                          \
                                                                             \
          if ( shift_imm5_part == 0 && VSP_UNLIKELY (shift == 0) )           \
          { /* rcr #1 */                                                     \
            offs >>= 1;                                                      \
            offs |= cf << 31;                                                \
            break;                                                           \
          }                                                                  \
          offs = ror32 (offs, shift);                                        \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          VSP_ASSERT (0);                                                    \
        }                                                                    \
      } while (0);                                                           \
    }                                                                        \
    else if ( (packet & 0xe00) == 0x400 )                                    \
    { /* imm12bit */                                                         \
      offs = opcode & 0xfff;                                                 \
    }                                                                        \
    else if ( (packet & 0x040) != 0 )                                        \
    { /* imm8bit */                                                          \
      offs = opcode & 0x0f;                                                  \
      offs|= opcode >> 4 & 0xf0;                                             \
    }                                                                        \
    else                                                                     \
    { /* rm */                                                               \
      offs = regs[opcode & 15];                                              \
    }                                                                        \
                                                                             \
    if ( field_w == 0 && field_p == 0 )                                      \
    { /* rn++, write back */                                                 \
      rnip = & regs[rn];                                                     \
      rw_addr = rnip[0];                                                     \
      rn_wb = (field_u != 0) ? (rw_addr + offs) : (rw_addr - offs);          \
    }                                                                        \
    else if ( field_w == 0 && field_p != 0 )                                 \
    { /* ++rn, none */                                                       \
      rnip = & t; /* dummy...*/                                              \
      rw_addr = (field_u != 0) ? (regs[rn] + offs) : (regs[rn] - offs);      \
      rn_wb = 0;                                                             \
    }                                                                        \
    else if ( field_w != 0 && field_p == 0 )                                 \
    { /* rn++, write back (t suffix) */                                      \
      rnip = & regs[rn];                                                     \
      rw_addr = rnip[0];                                                     \
      rn_wb = (field_u != 0) ? (rw_addr + offs) : (rw_addr - offs);          \
    }                                                                        \
    else                                                                     \
    { /* ++rn, write back */                                                 \
      rnip = & regs[rn]; /* dummy...*/                                       \
      rw_addr = (field_u != 0) ? (rnip[0] + offs) : (rnip[0] - offs);        \
      rn_wb = rw_addr;                                                       \
    }                                                                        \
                                                                             \
    if (write_mask != 0)                                                     \
    {                                                                        \
      rn_write = regs[rdi];                                                  \
                                                                             \
      /* NO$GBA ARM Opcodes:Memory:Single Data Transfer (LDR, STR, PLD) */   \
      rn_write += (rdi + 1) >> 2 & 4;                                        \
                                                                             \
      if (field_w != 0 && field_p != 0 && rdi == rni)                        \
      { /* ++rn, write back && rd = rn */                                    \
        rn_write = rw_addr;                                                  \
      }                                                                      \
                                                                             \
      if (size8_mask != 0)                                                   \
      {                                                                      \
        M_WRITE_HOOK8 (rw_addr, rn_write)                                    \
                                                                             \
        if ( mem_bank->fast_wr_mask != 0 )                                   \
        {                                                                    \
          mem_bank->mem[mem_bank_address] =                                  \
            static_cast<uint8_t> (rn_write);                                 \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          M_WRITE8 ( rw_addr, static_cast<uint8_t> (rn_write) );             \
        }                                                                    \
        ticks += mem_bank->mem_waits[0]; /* no-seq write 8bit wait state */  \
      }                                                                      \
      else if (size16_mask != 0)                                             \
      {                                                                      \
        M_WRITE_HOOK16 (rw_addr, rn_write)                                   \
                                                                             \
        if ( mem_bank->fast_wr_mask != 0 )                                   \
        {                                                                    \
          STORE16x (& mem_bank->mem[mem_bank_address]                        \
                 , static_cast<uint16_t> (rn_write) );                       \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          M_WRITE16 ( rw_addr, rn_write );                                   \
        }                                                                    \
        ticks += mem_bank->mem_waits[2]; /* no-seq write 16bit wait state */ \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        VSP_ASSERT (size32_mask != 0);                                       \
                                                                             \
        M_WRITE_HOOK32 (rw_addr, rn_write)                                   \
                                                                             \
        if ( mem_bank->fast_wr_mask != 0 )                                   \
        {                                                                    \
          STORE32x (& mem_bank->mem[mem_bank_address], rn_write );           \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          M_WRITE32 ( rw_addr, rn_write );                                   \
        }                                                                    \
        ticks += mem_bank->mem_waits[4]; /* no-seq write 32bit wait state */ \
      }                                                                      \
      rnip[0] = rn_wb;                                                       \
                                                                             \
      ticks++; /* 1N */                                                      \
      ticks++; /* 1N */                                                      \
                                                                             \
      M_ARM7_NEXT_NOSEQ ();                                                  \
    }                                                                        \
    else                                                                     \
    {                                                                        \
      if (rdi == rni)                                                        \
      {                                                                      \
        rnip = & t; /* write back discard...*/                               \
      }                                                                      \
                                                                             \
      if (size8_mask != 0)                                                   \
      {                                                                      \
        M_READ_HOOK8 (rw_addr)                                               \
                                                                             \
        if ( command == 0 )                                                  \
        { /* ldrb */                                                         \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            regs[rdi] = mem_bank->mem[mem_bank_address];                     \
          }                                                                  \
          else                                                               \
          {                                                                  \
            regs[rdi] = M_READ8 (rw_addr);                                   \
          }                                                                  \
        }                                                                    \
        else                                                                 \
        { /* ldrsb */                                                        \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                  \
              static_cast<int8_t> ( mem_bank->mem[mem_bank_address] );       \
          }                                                                  \
          else                                                               \
          {                                                                  \
            reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                  \
             static_cast<int8_t> (                                           \
              static_cast<uint8_t> ( M_READ8 (rw_addr) ) );                  \
          }                                                                  \
        }                                                                    \
        ticks += mem_bank->mem_waits[0]; /* no-seq read 8bit wait state */   \
      }                                                                      \
      else if (size16_mask != 0)                                             \
      {                                                                      \
        M_READ_HOOK16 (rw_addr)                                              \
                                                                             \
        if ( command == 2 )                                                  \
        { /* ldrh */                                                         \
          if ( mem_bank->fast_rd_mask != 0 )                                 \
          {                                                                  \
            regs[rdi] = LOAD16x ( & mem_bank->mem[mem_bank_address] );       \
          }                                                                  \
          else                                                               \
          {                                                                  \
            regs[rdi] = M_READ16 (rw_addr);                                  \
          }                                                                  \
                                                                             \
          if ( (rw_addr & 1) != 0 )                                          \
          { /* unaligned */                                                  \
            regs[rdi] = ror32 (regs[rdi], 8);                                \
          }                                                                  \
        }                                                                    \
        else if ( command == 3 )                                             \
        { /* ldrsh */                                                        \
          if ( (rw_addr & 1) != 0 )                                          \
          { /* unaligned, same as ldrsb */                                   \
            if ( mem_bank->fast_rd_mask != 0 )                               \
            {                                                                \
              reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                \
                static_cast<int8_t> ( mem_bank->mem[mem_bank_address] );     \
            }                                                                \
            else                                                             \
            {                                                                \
              reinterpret_cast<int32_t *> (& regs[rdi] )[0] =                \
               static_cast<int8_t> (                                         \
                static_cast<uint8_t> ( M_READ8 (rw_addr) ) );                \
            }                                                                \
          }                                                                  \
          else                                                               \
          {                                                                  \
            if ( mem_bank->fast_rd_mask != 0 )                               \
            {                                                                \
              regs[rdi] = LOAD16x ( & mem_bank->mem[mem_bank_address] );     \
            }                                                                \
            else                                                             \
            {                                                                \
              regs[rdi] = M_READ16 (rw_addr);                                \
            }                                                                \
          }                                                                  \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          VSP_ASSERT (0);                                                    \
        }                                                                    \
        ticks += mem_bank->mem_waits[2]; /* no-seq read 16bit wait state */  \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        VSP_ASSERT (size32_mask != 0);                                       \
                                                                             \
        M_READ_HOOK32 (rw_addr)                                              \
                                                                             \
        if ( mem_bank->fast_rd_mask != 0 )                                   \
        {                                                                    \
          regs[rdi] = ror32 ( LOAD32x (& mem_bank->mem[mem_bank_address]), ((rw_addr & 3) << 3) );   \
        }                                                                    \
        else                                                                 \
        {                                                                    \
          regs[rdi] = ror32 (M_READ32 (rn), ((rw_addr & 3) << 3));           \
        }                                                                    \
        ticks += mem_bank->mem_waits[4]; /* no-seq read 32bit wait state */  \
      }                                                                      \
      ticks++; /* 1S */                                                      \
      ticks++; /* 1N */                                                      \
      ticks++; /* 1I */                                                      \
                                                                             \
      rnip[0] = rn_wb;                                                       \
                                                                             \
      if (rdi == 15)                                                         \
      { /* flush pipeline */                                                 \
        ticks++; /* 1S */                                                    \
        ticks++; /* 1N */                                                    \
                                                                             \
        M_ARM7_FLUSH_PIPELINE ();                                            \
      }                                                                      \
      else                                                                   \
      {                                                                      \
        M_ARM7_NEXT_SEQ ();                                                  \
      }                                                                      \
    }                                                                        \
  } while (0)

  // |x.x.x.x x.x.x.x.......2 ..............1 x.x.x.x.......0|
  // |3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0_7_6_5_4_3_2_1_0|
  // |1_0_0|P|U|0|W|1|__Rn___|__________Register_List________| BlockTrans
#define M_LDM_S_(reg_slot_, mem_slot, addr)             \
  do                                                    \
  {                                                     \
    if ( (mem_hooks[mem_slot].fast_acc_mask & 1) != 0 ) \
    {                                                   \
      regs[reg_slot_] = LOAD32x (mem_hooks[mem_slot].mem);     \
    }                                                   \
    else                                                \
    {                                                   \
      regs[reg_slot_] = M_READ32 (addr);                \
    }                                                   \
    ticks += mem_hooks[mem_slot].mem_waits[seq + 4];    \
                                                        \
    addr += 4;                                          \
  } while (0)
  
#define LDM_BREAKPOINT_HIT_CLEANUP()                   \
  do                                                   \
  {                                                    \
    if ( (packet & 0xe70) == 0x850 && ldm_mode == 4 )  \
    {                                                  \
      regs[13] = t[0];                                 \
      regs[14] = t[1];                                 \
    }                                                  \
  } while (0)

#undef LDM_LDs
#define LDM_LDs(slot)                 \
  do                                  \
  {                                   \
    regsL[slot] = regs[slot];         \
                                      \
    if ( (mask & (1 << slot)) != 0 )  \
    {                                 \
      regsL[slot] = LOAD32x (memory); \
                                      \
      M_LDM_HOOKx (trace);            \
                                      \
      trace += 4;                     \
      memory += 4;                    \
    }                                 \
  } while (0)

#undef LDM_LDs
#define LDM_LDs(slot)                 \
  do                                  \
  {                                   \
    if ( (mask & (1 << slot)) != 0 )  \
    {                                 \
      regs[slot] = LOAD32x (memory);  \
                                      \
      memory += 4;                    \
    }                                 \
  } while (0)

#undef LDM_LDs2
#define LDM_LDs2(slot)                 \
  do                                  \
  {                                   \
    if ( (mask & (1 << slot)) != 0 )  \
    {                                 \
      regs[slot] = M_READ32 (addr);   \
                                      \
      addr += 4;                      \
    }                                 \
  } while (0)

#define M_ARM7_LDM(packet_, opcode_)                                \
  do                                                                \
  {                                                                 \
    static const size_t packet = packet_;                           \
                                                                    \
    uint32_t opcode = opcode_;                                      \
    uint32_t ldm_mode = 0;                                          \
    uint32_t t[2];                                                  \
    uint32_t mode_ = mode & 15;                                     \
    uint32_t usr_mask =                                             \
      ( mode_ == 0 || mode_ == 15 ) ? 1 : 0;                        \
                                                                    \
    if (  (packet & 0xe70) == 0x850                                 \
      && (opcode & 0x8000 ) == 0 )                                  \
    { /* LDM (2) */                                                 \
      if ( usr_mask == 0 )                                          \
      { /* current not user/sys mode */                             \
        t[0] = regs[13];                                            \
        t[1] = regs[14];                                            \
                                                                    \
        regs[13] = regs2x[0];                                       \
        regs[14] = regs2x[1];                                       \
                                                                    \
        ldm_mode = 4;                                               \
      }                                                             \
    }                                                               \
                                                                    \
    do                                                              \
    {                                                               \
      uint32_t rni = opcode >> 16 & 15;                             \
      uint16_t mask = opcode & 0xffff;                              \
      uint32_t addr = regs[rni];                                    \
      uint32_t addrb;                                               \
      uint32_t counts = counts1 (mask);                             \
      uint32_t counts4x = counts << 2;                              \
      uint32_t seq = 0;                                             \
      uint32_t addrwb;                                              \
      uint32_t regsL[16]; /* store. temp. */                        \
                                                                    \
      ticks++; /* 1N */                                             \
      ticks++; /* 1I */                                             \
                                                                    \
      if ( mask == 0 )                                              \
      { /* ub... */                                                 \
        M_ARM7_NEXT_SEQ ();                                         \
        break;                                                      \
      }                                                             \
      else if ( (packet & 0x080) != 0 )                             \
      { /* address increment */                                     \
        if ( (packet & 0x100) != 0 )                                \
        { /* pre add. */                                            \
          addr += 4;                                                \
        }                                                           \
        addrb = addr;                                               \
        addrwb= addr + counts4x;                                    \
      }                                                             \
      else                                                          \
      { /* address decrement */                                     \
        addrwb= addr - counts4x;                                    \
        addrb = addrwb;                                             \
                                                                    \
        if ( (packet & 0x100) == 0 )                                \
        { /* no pre */                                              \
          addrb += 4;                                               \
        }                                                           \
      }                                                             \
      addr = addrb;                                                 \
                                                                    \
      memors &bank = mmu[addr >> 24 & 15];                          \
                                                                    \
      if ( (bank.fast_acc_mask & 1) != 0                            \
         && ( (addr & bank.mirror_mask)                             \
           + counts4x <= bank.mirror_mask) )                        \
      { /* fast read && same bank */                                \
        uint32_t mirror_mask = bank.mirror_mask;                    \
        uint8_t *memory= & bank.mem[addr & mirror_mask & ~3];       \
        uint8_t *trace = & memory[mirror_mask + 4 + 1];             \
                                                                    \
        LDM_LDs (0);                                                \
        LDM_LDs (1);                                                \
        LDM_LDs (2);                                                \
        LDM_LDs (3);                                                \
        LDM_LDs (4);                                                \
        LDM_LDs (5);                                                \
        LDM_LDs (6);                                                \
        LDM_LDs (7);                                                \
        LDM_LDs (8);                                                \
        LDM_LDs (9);                                                \
        LDM_LDs (10);                                               \
        LDM_LDs (11);                                               \
        LDM_LDs (12);                                               \
        LDM_LDs (13);                                               \
        LDM_LDs (14);                                               \
        LDM_LDs (15);                                               \
                                                                    \
        regs[0] = regsL[0];                                         \
        regs[1] = regsL[1];                                         \
        regs[2] = regsL[2];                                         \
        regs[3] = regsL[3];                                         \
        regs[4] = regsL[4];                                         \
        regs[5] = regsL[5];                                         \
        regs[6] = regsL[6];                                         \
        regs[7] = regsL[7];                                         \
        regs[8] = regsL[8];                                         \
        regs[9] = regsL[9];                                         \
        regs[10]= regsL[10];                                        \
        regs[11]= regsL[11];                                        \
        regs[12]= regsL[12];                                        \
        regs[13]= regsL[13];                                        \
        regs[14]= regsL[14];                                        \
        regs[15]= regsL[15];                                        \
                                                                    \
        ticks += counts; /* nS */                                   \
        ticks += bank.mem_waits[5] * (counts - 1);  /* (n-1)S waitstate */ \
        ticks += bank.mem_waits[4]; /* (1)N waitstate */                   \
      }                                                                    \
      else                                                                 \
      { /* memory hook */                                                  \
        if ( (mask & 0x0001) != 0 ) { M_LDM_HOOK (0, addr); addr += 4; }   \
        if ( (mask & 0x0002) != 0 ) { M_LDM_HOOK (1, addr); addr += 4; }   \
        if ( (mask & 0x0004) != 0 ) { M_LDM_HOOK (2, addr); addr += 4; }   \
        if ( (mask & 0x0008) != 0 ) { M_LDM_HOOK (3, addr); addr += 4; }   \
        if ( (mask & 0x0010) != 0 ) { M_LDM_HOOK (4, addr); addr += 4; }   \
        if ( (mask & 0x0020) != 0 ) { M_LDM_HOOK (5, addr); addr += 4; }   \
        if ( (mask & 0x0040) != 0 ) { M_LDM_HOOK (6, addr); addr += 4; }   \
        if ( (mask & 0x0080) != 0 ) { M_LDM_HOOK (7, addr); addr += 4; }   \
        if ( (mask & 0x0100) != 0 ) { M_LDM_HOOK (8, addr); addr += 4; }   \
        if ( (mask & 0x0200) != 0 ) { M_LDM_HOOK (9, addr); addr += 4; }   \
        if ( (mask & 0x0400) != 0 ) { M_LDM_HOOK (10,addr); addr += 4; }   \
        if ( (mask & 0x0800) != 0 ) { M_LDM_HOOK (11,addr); addr += 4; }   \
        if ( (mask & 0x1000) != 0 ) { M_LDM_HOOK (12,addr); addr += 4; }   \
        if ( (mask & 0x2000) != 0 ) { M_LDM_HOOK (13,addr); addr += 4; }   \
        if ( (mask & 0x4000) != 0 ) { M_LDM_HOOK (14,addr); addr += 4; }   \
        if ( (mask & 0x8000) != 0 ) { M_LDM_HOOK (15,addr); addr += 4; }   \
                                                                           \
        addr = addrb; /* resume */                                                     \
                                                                                       \
        if ( (mask & 0x0001) != 0 ) { M_LDM_S_ ( 0, 0, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0002) != 0 ) { M_LDM_S_ ( 1, 1, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0004) != 0 ) { M_LDM_S_ ( 2, 2, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0008) != 0 ) { M_LDM_S_ ( 3, 3, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0010) != 0 ) { M_LDM_S_ ( 4, 4, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0020) != 0 ) { M_LDM_S_ ( 5, 5, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0040) != 0 ) { M_LDM_S_ ( 6, 6, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0080) != 0 ) { M_LDM_S_ ( 7, 7, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0100) != 0 ) { M_LDM_S_ ( 8, 8, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0200) != 0 ) { M_LDM_S_ ( 9, 9, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0400) != 0 ) { M_LDM_S_ (10,10, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x0800) != 0 ) { M_LDM_S_ (11,11, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x1000) != 0 ) { M_LDM_S_ (12,12, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x2000) != 0 ) { M_LDM_S_ (13,13, addr); addr += 4; seq = 1; }    \
        if ( (mask & 0x4000) != 0 ) { M_LDM_S_ (14,14, addr); addr += 4; seq = 1; }    \
                                                                                       \
        ticks += counts; /* nS */                                                      \
      }                                                                                \
                                                                                       \
      if ( (packet & 0x020) != 0 && (mask & 1 << rni) == 0 )                           \
      {                                                                                \
        regs[rni] = addrwb;                                                            \
      }                                                                                \
                                                                                       \
      if ( (mask & 0x8000) != 0 )                                                      \
      {                                                                                \
        if ( (packet & 0xe50) == 0x850 )                                   \
        { /* LDM (3), spsr to cpsr */                                      \
          if ( (mode & 15) != 0 )                                          \
          {                                                                \
            uint32_t spsr = get_spsr ();                                   \
            uint32_t *from = regs4px[mode & 15];                           \
            uint32_t *to = regs4px[spsr & 15];                             \
                                                                           \
            from[0] = regs[13];                                            \
            from[1] = regs[14];                                            \
                                                                           \
            regs[13] = to[0];                                              \
            regs[14] = to[1];                                              \
                                                                           \
            regs2x[2] = spsr;                                              \
                                                                           \
            mode = spsr & 15 | 0x10;                                       \
                                                                           \
            thumb_mask = (spsr & 0x20) ? 0 : 0xffffffff;                   \
                                                                           \
            irq_mask = (spsr & 0x80) ? 0 : 0xffffffff;                     \
                                                                           \
            nf = spsr;                                                     \
            zf = spsr & 0x40000000 ^ 0x40000000;                           \
            cf = spsr >> 29;                                               \
            of = spsr << 3;                                                \
                                                                           \
            if (thumb_mask != 0)                                           \
            {                                                              \
              regs[15] &= ~1;                                              \
                                                                           \
              M_THUMB_FLUSH_PIPELINE ();                                   \
            }                                                              \
            else                                                           \
            {                                                              \
              regs[15] &= ~3;                                              \
                                                                           \
              M_ARM7_FLUSH_PIPELINE ();                                    \
            }                                                              \
          }                                                                \
          else                                                             \
          { /* ub... */                                                    \
            VSP_ASSERT (0);                                                \
          }                                                                \
        }                                                                              \
        else                                                                           \
        {                                                                              \
          regs[15] &= ~3;                                                              \
                                                                                       \
          ticks++; /* 1S */                                                            \
          ticks++; /* 1N */                                                            \
                                                                                       \
          M_ARM7_FLUSH_PIPELINE ();                                                    \
        }                                                                              \
      }                                                                                \
      else                                                                             \
      {                                                                                \
        M_ARM7_NEXT_SEQ ();                                                            \
      }                                                                                \
    } while (0);                                                                       \
                                                                                       \
    if ( ldm_mode == 4 )                                                               \
    {                                                                                  \
      regs2x[0] = regs[13];                                                            \
      regs2x[1] = regs[14];                                                            \
                                                                                       \
      regs[13] = t[0];                                                                 \
      regs[14] = t[1];                                                                 \
    }                                                                                  \
  } while (0)

  VSP_STATIC_FORCEINLINE
  uint8_t counts1 (uint16_t x)
  {
#ifdef VSP_VECTOR_PROCESS
    return static_cast<uint8_t> (_mm_popcnt_u32 (x));
#else                             // abcd efgh ijkl mnop                                
    uint16_t a = x & 0x5555;      //  b d  f h  j l  n p
    uint16_t b = x >> 1 & 0x5555; //  a c  e g  i k  m o
             x = a + b;           // aabb ccdd eeff gghh
             a = x & 0x3333;      //   bb   dd   ff   hh 
             b = x >> 2 & 0x3333; //   aa   cc   ee   gg 
             x = a + b;           // 0aaa 0bbb 0ccc 0ddd
             x = x + (x >> 4);    // 0000 0aaa 0bbb 0ccc                
             x&= 0x0f0f;          // 0000 aaaa 0000 bbbb
    return static_cast<uint8_t> ((x >> 8) + x);
#endif 
  }

  int eval_read ( uint32_t address, uint32_t mask )
  {
    return 1;
  }

  int eval_write ( uint32_t address, uint32_t value, uint32_t mask )
  {
    return 1;
  }

  int eval_exec ( uint32_t address, uint32_t mask )
  {
    return 1;
  }
};


