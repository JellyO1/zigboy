//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.
const std = @import("std");
const c = @cImport({
    @cDefine("SDL_DISABLE_OLD_NAMEs", {});
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_revision.h");
});

const graphics = @import("graphics.zig");
const PPU = @import("ppu.zig").PPU;
const MMU = @import("mmu.zig").MMU;

// Two types of cycles: T states and M cycles
// M cycles (“machine” cycles, 1:4 clock) are the base unit of CPU instructions
// T states or T cycles (“transistor”(?) states, 1:1 clock) are the base unit of system operation and many components are clocked directly on T state
pub const MCycle = 4;
// Sharp SM83 clock rate (4.194304 MHz)
pub const CPUClockRate = 4194304;

pub const ConditionCode = enum(u2) {
    Z,
    C,
    NZ,
    NC,
};

pub const Flags = packed struct(u8) {
    /// Bits 0–3 are grounded to 0
    G: u4 = 0,
    /// Carry flag (bit 4)
    C: bool = false,
    /// Half carry flag (bit 5)
    H: bool = false,
    /// Subtract flag (bit 6)
    N: bool = false,
    /// Zero flag (bit 7)
    Z: bool = false,

    pub fn init(flags: ?Flags) Flags {
        return .{
            .C = if (flags) |f| f.C else false,
            .H = if (flags) |f| f.H else false,
            .N = if (flags) |f| f.N else false,
            .Z = if (flags) |f| f.Z else false,
        };
    }
};

pub const Registers = struct {
    /// Accumulator
    A: u8,
    B: u8,
    C: u8,
    D: u8,
    E: u8,
    H: u8,
    L: u8,
    F: Flags,

    /// Program Counter
    PC: u16,
    /// Stack Pointer
    SP: u16,

    pub const Byte = enum {
        A,
        B,
        C,
        D,
        E,
        H,
        L,
    };

    pub const Word = enum {
        AF,
        BC,
        DE,
        HL,
    };

    pub fn init(flags: ?Flags) Registers {
        return .{
            .A = 0,
            .B = 0,
            .C = 0,
            .D = 0,
            .E = 0,
            .H = 0,
            .L = 0,
            .F = Flags.init(flags),
            .PC = 0,
            .SP = 0,
        };
    }

    pub fn getByte(self: *Registers, byte: Byte) u8 {
        switch (byte) {
            Byte.A => return self.A,
            Byte.B => return self.B,
            Byte.C => return self.C,
            Byte.D => return self.D,
            Byte.E => return self.E,
            Byte.H => return self.H,
            Byte.L => return self.L,
        }
    }

    pub fn setByte(self: *Registers, byte: Byte, value: u8) void {
        switch (byte) {
            Byte.A => self.A = value,
            Byte.B => self.B = value,
            Byte.C => self.C = value,
            Byte.D => self.D = value,
            Byte.E => self.E = value,
            Byte.H => self.H = value,
            Byte.L => self.L = value,
        }
    }

    pub fn getWord(self: *Registers, word: Word) u16 {
        return switch (word) {
            Word.AF => @intCast(@as(u16, self.A) << 8 | @as(u8, @bitCast(self.F))),
            Word.BC => @intCast(@as(u16, self.B) << 8 | self.C),
            Word.DE => @intCast(@as(u16, self.D) << 8 | self.E),
            Word.HL => @intCast(@as(u16, self.H) << 8 | self.L),
        };
    }

    pub fn setWord(self: *Registers, word: Word, value: u16) void {
        switch (word) {
            Word.AF => {
                self.A = @intCast((value >> 8) & 0xFF);
                self.F = Flags.init(.{
                    .C = (value & 0b00010000) != 0, // bit 4
                    .H = (value & 0b00100000) != 0, // bit 5
                    .N = (value & 0b01000000) != 0, // bit 6
                    .Z = (value & 0b10000000) != 0, // bit 7
                });
            },
            Word.BC => {
                self.B = @intCast((value >> 8) & 0xFF);
                self.C = @intCast(value & 0xFF);
            },
            Word.DE => {
                self.D = @intCast((value >> 8) & 0xFF);
                self.E = @intCast(value & 0xFF);
            },
            Word.HL => {
                self.H = @intCast((value >> 8) & 0xFF);
                self.L = @intCast(value & 0xFF);
            },
        }
    }
};

pub const GameBoyState = struct {
    /// Picture Processing Unit
    ppu: PPU,
    /// General Purpose registers
    registers: Registers,
    /// Memory management unit
    mmu: MMU,
    /// Total cycles elapsed
    cycles: u64,
    /// Interrupt Master Enable
    ime: bool,
    /// EI delay (1 instruction delay before turning on IME)
    ei_delay: bool,
    /// Halt
    halt: bool,

    pub fn init(flags: ?Flags) !GameBoyState {
        // Load bootrom to memory
        const boot_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, "src/dmg_boot.bin", 256);
        defer std.heap.page_allocator.free(boot_rom);
        const boot_rom_slice = boot_rom[0..0x100].*;

        const game_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, "src/cpu_instrs.gb", 1024 * 1024);
        defer std.heap.page_allocator.free(game_rom);

        var mmu = MMU.init(boot_rom_slice, game_rom);
        return .{
            .registers = Registers.init(flags),
            .mmu = mmu,
            .ppu = PPU.init(&mmu),
            .cycles = 0,
            .ime = false,
            .ei_delay = false,
            .halt = false,
        };
    }

    pub fn initTest(registers: Registers, ime: bool, ei_delay: bool) GameBoyState {
        var mmu = MMU.init(null, null);
        return .{
            .registers = registers,
            .mmu = mmu,
            .ppu = PPU.init(&mmu),
            .cycles = 0,
            .ime = ime,
            .ei_delay = ei_delay,
            .halt = false,
        };
    }

    pub fn step(self: *GameBoyState) void {
        const opcode = fetch(self);
        const cycles = execute(self, opcode);
        self.cycles += cycles;
        self.ppu.step(cycles);

        // sleep for the number of nanoseconds equivalent to the number of cycles
        // std.time.sleep((cycles / CPUClockRate) * std.time.ns_per_s);
    }
};

/// No operation
pub fn noop() u32 {
    return MCycle;
}

pub fn adc_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const result, const overflow = @addWithOverflow(state.registers.A, state.registers.getByte(r8) +% @as(u8, @as(u1, @bitCast(state.registers.F.C))));
    state.registers.A = result;

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle;
}

pub fn adc_a_vhl(state: *GameBoyState) u32 {
    const result, const overflow = @addWithOverflow(state.registers.A, state.mmu.read(state.registers.getWord(Registers.Word.HL)) +% @as(u8, @as(u1, @bitCast(state.registers.F.C))));
    state.registers.A = result;

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle * 2;
}

pub fn adc_a_n8(state: *GameBoyState, n8: *const u8) u32 {
    const result, const overflow = @addWithOverflow(state.registers.A, n8.* +% @as(u8, @as(u1, @bitCast(state.registers.F.C))));
    state.registers.A = result;

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle * 2;
}

pub fn add_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const result, const overflow = @addWithOverflow(state.registers.A, state.registers.getByte(r8));
    state.registers.A = result;

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle;
}

pub fn add_a_vhl(state: *GameBoyState) u32 {
    const result, const overflow = @addWithOverflow(state.registers.A, state.mmu.read(state.registers.getWord(Registers.Word.HL)));
    state.registers.A = result;

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle * 2;
}

pub fn add_a_n8(state: *GameBoyState, n8: *const u8) u32 {
    const result, const overflow = @addWithOverflow(state.registers.A, n8.*);
    state.registers.A = result;

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle * 2;
}

pub fn add_hl_r16(state: *GameBoyState, r16: Registers.Word) u32 {
    const result, const overflow = @addWithOverflow(state.registers.getWord(Registers.Word.HL), state.registers.getWord(r16));
    state.registers.setWord(Registers.Word.HL, result);

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x800) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle * 2;
}

pub fn add_hl_sp(state: *GameBoyState) u32 {
    const result, const overflow = @addWithOverflow(state.registers.getWord(Registers.Word.HL), state.registers.SP);
    state.registers.setWord(Registers.Word.HL, result);

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x800) != 0;
    state.registers.F.C = overflow == 1;

    return MCycle * 2;
}

pub fn add_sp_e8(state: *GameBoyState, e8: *const i8) u32 {
    const u32Result: u32 = @intCast(@as(i32, @intCast(state.registers.SP)) +% e8.*);
    const result: u16 = @truncate(u32Result);

    state.registers.SP = result;

    state.registers.F.Z = false;
    state.registers.F.N = false;
    state.registers.F.H = (result & 0x10) != 0;
    state.registers.F.C = (result & 0x100) != 0;

    return MCycle * 4;
}

pub fn and_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    state.registers.A = state.registers.A & val;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = false;
    state.registers.F.H = true;
    state.registers.F.C = false;
    return MCycle;
}

pub fn and_a_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.registers.A = state.registers.A & val;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = false;
    state.registers.F.H = true;
    state.registers.F.C = false;
    return MCycle * 2;
}

pub fn and_a_n8(state: *GameBoyState, n8: *const u8) u32 {
    const val = n8.*;
    state.registers.A = state.registers.A & val;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = false;
    state.registers.F.H = true;
    state.registers.F.C = false;
    return MCycle * 2;
}

pub fn bit_u3_r8(state: *GameBoyState, comptime vu3: u3, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    state.registers.F.Z = (val & (1 << vu3)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = true;
    return MCycle * 2;
}

pub fn bit_u3_vhl(state: *GameBoyState, comptime vu3: u3) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.registers.F.Z = (val & (1 << vu3)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = true;
    return MCycle * 3;
}

pub fn call_n16(state: *GameBoyState, n16: *const u16) u32 {
    // Save the next instruction address
    const next_instr_addr = state.registers.PC;

    // Split 16-bit value into high and low 8-bit values
    const high: u8 = @intCast(next_instr_addr >> 8);
    const low: u8 = @intCast(next_instr_addr & 0xFF);

    // Decrement SP by 2 (16 bits)
    state.registers.SP -= 2;

    // Write high and low bytes to stack
    state.mmu.write(state.registers.SP + 1, high);
    state.mmu.write(state.registers.SP, low);

    // jump to the new address
    state.registers.PC = n16.*;

    return MCycle * 6;
}

pub fn call_cc_n16(state: *GameBoyState, n16: *const u16, cc: ConditionCode) u32 {
    switch (cc) {
        .Z => {
            if (state.registers.F.Z) {
                return call_n16(state, n16);
            }
        },
        .C => {
            if (state.registers.F.C) {
                return call_n16(state, n16);
            }
        },
        .NZ => {
            if (!state.registers.F.Z) {
                return call_n16(state, n16);
            }
        },
        .NC => {
            if (!state.registers.F.C) {
                return call_n16(state, n16);
            }
        },
    }

    return MCycle * 3;
}

pub fn ccf(state: *GameBoyState) u32 {
    state.registers.F.C = !state.registers.F.C;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle;
}

pub fn cp_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const result, const overflow = @subWithOverflow(state.registers.A, val);
    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (state.registers.A & 0xF) < (val & 0xF);
    state.registers.F.C = overflow == 1;
    return MCycle;
}

pub fn cp_a_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const result, const overflow = @subWithOverflow(state.registers.A, val);
    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (state.registers.A & 0xF) < (val & 0xF);
    state.registers.F.C = overflow == 1;
    return MCycle * 2;
}

pub fn cp_a_n8(state: *GameBoyState, n8: *const u8) u32 {
    const val = n8.*;
    const result, const overflow = @subWithOverflow(state.registers.A, val);
    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = (state.registers.A & 0xF) < (val & 0xF);
    state.registers.F.C = overflow == 1;
    return MCycle * 2;
}

pub fn cpl(state: *GameBoyState) u32 {
    state.registers.A = ~state.registers.A;
    state.registers.F.N = true;
    state.registers.F.H = true;
    return MCycle;
}

pub fn daa(state: *GameBoyState) u32 {
    var adjustment: u8 = 0;
    var setCarry: bool = false;
    if (state.registers.F.N) {
        if (state.registers.F.C) {
            adjustment += 0x60;
        }
        if (state.registers.F.H) {
            adjustment += 0x6;
        }

        state.registers.A -%= adjustment;
    } else {
        if (state.registers.F.C or state.registers.A > 0x99) {
            adjustment += 0x60;
            setCarry = true;
        }
        if (state.registers.F.H or (state.registers.A & 0xF) > 0x9) {
            adjustment += 0x6;
        }

        state.registers.A +%= adjustment;
    }

    state.registers.F.C = setCarry;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.H = false;
    return MCycle;
}

pub fn dec_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    state.registers.setByte(r8, val -% 1);
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = true;
    state.registers.F.H = (val & 0xF) < 1;
    return MCycle;
}

pub fn dec_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.mmu.write(state.registers.getWord(Registers.Word.HL), val -% 1);
    state.registers.F.Z = (val -% 1) == 0;
    state.registers.F.N = true;
    state.registers.F.H = (val & 0xF) < 1;
    return MCycle * 3;
}

pub fn dec_r16(state: *GameBoyState, r16: Registers.Word) u32 {
    const val = state.registers.getWord(r16);
    state.registers.setWord(r16, val -% 1);
    return MCycle * 2;
}

pub fn dec_sp(state: *GameBoyState) u32 {
    state.registers.SP -%= 1;
    return MCycle * 2;
}

pub fn di(state: *GameBoyState) u32 {
    state.ime = false;
    return MCycle;
}

pub fn ei(state: *GameBoyState) u32 {
    state.ei_delay = true;
    return MCycle;
}

/// Enter CPU low-power consumption mode until an interrupt occurs.
///
/// The exact behavior of this instruction depends on the state of the IME flag, and whether interrupts are pending (i.e. whether ‘[IE] & [IF]’ is non-zero):
///
/// If the IME flag is set:
///
/// * The CPU enters low-power mode until after an interrupt is about to be serviced. The handler is executed normally, and the CPU resumes execution after the HALT when that returns.
///
/// If the IME flag is not set, and no interrupts are pending:
///
/// * As soon as an interrupt becomes pending, the CPU resumes execution. This is like the above, except that the handler is not called.
///
/// If the IME flag is not set, and some interrupt is pending:
///
/// * The CPU continues execution after the HALT, but the byte after it is read twice in a row (PC is not incremented, due to a hardware bug).
pub fn halt(state: *GameBoyState) u32 {
    state.halt = true;
    return 0;
}

pub fn inc_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    state.registers.setByte(r8, val +% 1);
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = (val & 0xF) > 0xF;
    return MCycle;
}

pub fn inc_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.mmu.write(state.registers.getWord(Registers.Word.HL), val +% 1);
    state.registers.F.Z = (val +% 1) == 0;
    state.registers.F.N = false;
    state.registers.F.H = (val & 0xF) > 0xF;
    return MCycle * 3;
}

pub fn inc_r16(state: *GameBoyState, r16: Registers.Word) u32 {
    const val = state.registers.getWord(r16);
    state.registers.setWord(r16, val +% 1);
    return MCycle * 2;
}

pub fn inc_sp(state: *GameBoyState) u32 {
    state.registers.SP +%= 1;
    return MCycle * 2;
}

pub fn jp_n16(state: *GameBoyState, n16: *const u16) u32 {
    state.registers.PC = n16.*;
    return MCycle * 4;
}

pub fn jp_cc_n16(state: *GameBoyState, n16: *const u16, cc: ConditionCode) u32 {
    switch (cc) {
        .Z => {
            if (state.registers.F.Z) {
                return jp_n16(state, n16);
            }
        },
        .C => {
            if (state.registers.F.C) {
                return jp_n16(state, n16);
            }
        },
        .NZ => {
            if (!state.registers.F.Z) {
                return jp_n16(state, n16);
            }
        },
        .NC => {
            if (!state.registers.F.C) {
                return jp_n16(state, n16);
            }
        },
    }
    return MCycle * 3;
}

pub fn jp_hl(state: *GameBoyState) u32 {
    state.registers.PC = state.registers.getWord(Registers.Word.HL);
    return MCycle;
}

pub fn jr_e8(state: *GameBoyState, e8: i8) u32 {
    state.registers.PC +%= @as(u16, @bitCast(@as(i16, e8)));
    return MCycle * 3;
}

pub fn jr_cc_e8(state: *GameBoyState, e8: i8, cc: ConditionCode) u32 {
    switch (cc) {
        .Z => {
            if (state.registers.F.Z) {
                return jr_e8(state, e8);
            }
        },
        .C => {
            if (state.registers.F.C) {
                return jr_e8(state, e8);
            }
        },
        .NZ => {
            if (!state.registers.F.Z) {
                return jr_e8(state, e8);
            }
        },
        .NC => {
            if (!state.registers.F.C) {
                return jr_e8(state, e8);
            }
        },
    }
    return MCycle * 2;
}

/// Copy the value of the register in the right to the register in the left
pub fn ld_r8_r8(state: *GameBoyState, r8_left: Registers.Byte, r8_right: Registers.Byte) u32 {
    //Storing a register into itself is a no-op; however, some Game Boy emulators interpret LD B,B as a breakpoint, or LD D,D as a debug message (such as BGB).
    if (r8_left == r8_right) {
        if (r8_left == Registers.Byte.B) {
            // breakpoint
        }

        return noop();
    }

    state.registers.setByte(r8_left, state.registers.getByte(r8_right));

    // LD r8,r8 takes 4 cycles (1 M-cycle)
    return MCycle;
}

/// Copy the value of the immediate byte to the register
pub fn ld_r8_n8(state: *GameBoyState, r8: Registers.Byte, n8: *const u8) u32 {
    state.registers.setByte(r8, n8.*);
    return MCycle * 2;
}

/// Copy the value of the immediate word to the register
pub fn ld_r16_n16(state: *GameBoyState, r16: Registers.Word, n16: *const u16) u32 {
    state.registers.setWord(r16, n16.*);
    return MCycle * 3;
}

/// Copy the value of the register to the memory location specified by the registers H and L
pub fn ld_vhl_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    state.mmu.write(state.registers.getWord(Registers.Word.HL), state.registers.getByte(r8));
    return MCycle * 2;
}

/// Copy the value of the immediate byte to the memory location specified by the registers H and L
pub fn ld_vhl_n8(state: *GameBoyState, n8: *const u8) u32 {
    state.mmu.write(state.registers.getWord(Registers.Word.HL), n8.*);
    return MCycle * 3;
}

/// Copy the value of the memory location specified by the registers H and L to the register
pub fn ld_r8_vhl(state: *GameBoyState, r8: Registers.Byte) u32 {
    state.registers.setByte(r8, state.mmu.read(state.registers.getWord(Registers.Word.HL)));
    return MCycle * 2;
}

/// Copy the value in the register A to the byte pointed to by r16
pub fn ld_vr16_a(state: *GameBoyState, r16: Registers.Word) u32 {
    state.mmu.write(state.registers.getWord(r16), state.registers.A);
    return MCycle * 2;
}

/// Copy the value in the register A to the byte pointed to by n16
pub fn ld_n16_a(state: *GameBoyState, n16: *const u16) u32 {
    state.mmu.write(n16.*, state.registers.A);

    return MCycle * 4;
}

/// Copy the value in the register A to the byte pointed to by n16
///
/// n16 must be in the range 0xFF00-0xFFFF
pub fn ldh_n16_a(state: *GameBoyState, n16: u16) u32 {
    if (n16 >= 0xFF00 and n16 <= 0xFFFF) {
        state.mmu.write(n16, state.registers.A);
    }

    return MCycle * 3;
}

/// Copy the value in the register A to the byte pointed to by 0xFF00 + n8
pub fn ldh_n8_a(state: *GameBoyState, n8: *const u8) u32 {
    state.mmu.write(0xFF00 + @as(u16, @intCast(n8.*)), state.registers.A);
    return MCycle * 3;
}

/// Copy the value in the register A to the byte pointed to by 0xFF00 + C
pub fn ldh_vc_a(state: *GameBoyState) u32 {
    state.mmu.write(0xFF00 + @as(u16, @intCast(state.registers.getByte(Registers.Byte.C))), state.registers.A);
    return MCycle * 2;
}

/// Copy the value in the byte pointed to by r16 to the register A
pub fn ld_a_vr16(state: *GameBoyState, r16: Registers.Word) u32 {
    state.registers.A = state.mmu.read(state.registers.getWord(r16));
    return MCycle * 2;
}

/// Copy the value in the byte pointed to by n16 to the register A
pub fn ld_a_vn16(state: *GameBoyState, n16: u16) u32 {
    state.registers.A = state.mmu.read(n16);
    return MCycle * 4;
}

/// Copy the value in the byte pointed to by n16 to the register A
///
/// n16 must be in the range 0xFF00-0xFFFF
pub fn ldh_a_vn16(state: *GameBoyState, n16: u16) u32 {
    if (n16 >= 0xFF00 and n16 <= 0xFFFF) {
        state.registers.A = state.mmu.read(n16);
    }
    return MCycle * 3;
}

/// Copy the value in the byte pointed to by 0xFF00 + n8 to the register A
pub fn ldh_a_n8(state: *GameBoyState, n8: *const u8) u32 {
    state.registers.A = state.mmu.read(0xFF00 + @as(u16, @intCast(n8.*)));
    return MCycle * 3;
}

/// Copy the value in the byte pointed to by 0xFF00 + C to the register A
pub fn ldh_a_vc(state: *GameBoyState) u32 {
    state.registers.A = state.mmu.read(0xFF00 + @as(u16, @intCast(state.registers.getByte(Registers.Byte.C))));
    return MCycle * 2;
}

/// Copy the value in the register A to the byte pointed to by HL and increment HL
pub fn ld_hli_a(state: *GameBoyState) u32 {
    state.mmu.write(state.registers.getWord(Registers.Word.HL), state.registers.A);
    state.registers.setWord(Registers.Word.HL, state.registers.getWord(Registers.Word.HL) + 1);
    return MCycle * 2;
}

/// Copy the value in the register A to the byte pointed to by HL and decrement HL
pub fn ld_hld_a(state: *GameBoyState) u32 {
    state.mmu.write(state.registers.getWord(Registers.Word.HL), state.registers.A);
    state.registers.setWord(Registers.Word.HL, state.registers.getWord(Registers.Word.HL) - 1);
    return MCycle * 2;
}

/// Copy the value in the byte pointed to by HL to the register A and decrement HL
pub fn ld_a_hld(state: *GameBoyState) u32 {
    state.registers.A = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.registers.setWord(Registers.Word.HL, state.registers.getWord(Registers.Word.HL) - 1);
    return MCycle * 2;
}

/// Copy the value in the byte pointed to by HL to the register A and increment HL
pub fn ld_a_hli(state: *GameBoyState) u32 {
    state.registers.A = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.registers.setWord(Registers.Word.HL, state.registers.getWord(Registers.Word.HL) + 1);
    return MCycle * 2;
}

/// Copy the value n16 into the register SP
pub fn ld_sp_n16(state: *GameBoyState, n16: *const u16) u32 {
    state.registers.SP = n16.*;
    return MCycle * 3;
}

/// Copy SP & 0xFF to address n16 and SP >> 8 to address n16 + 1
pub fn ld_vn16_sp(state: *GameBoyState, n16: *const u16) u32 {
    state.mmu.write(n16.*, @intCast(state.registers.SP & 0xFF));
    state.mmu.write(n16.* + 1, @intCast(state.registers.SP >> 8));
    return MCycle * 5;
}

/// Add the signed value e8 to SP and copy the result into HL
pub fn ld_hl_sp_plus_e8(state: *GameBoyState, e8: i8) u32 {
    // TODO: all of this might not be necessary.

    const sp = state.registers.SP;
    const offset = @as(u16, @bitCast(@as(i16, e8)));

    // Wrapping addition (+%) when it doesn't fit wraps around
    const ov = @addWithOverflow(sp, offset);
    const result = ov[0];

    // Reset zero flag
    state.registers.F.Z = false;
    // Reset subtract flag
    state.registers.F.N = false;

    // Half carry occurs if bit 3 carries into bit 4
    state.registers.F.H = ((sp & 0x0F) + (offset & 0x0F)) & 0x10 != 0;
    // Carry occurs if bit 7 carries into bit 8
    state.registers.F.C = ov[1] != 0;

    // store result in HL
    state.registers.setWord(Registers.Word.HL, result);

    return MCycle * 3;
}

/// Copy the value in the register HL into the register SP
pub fn ld_sp_hl(state: *GameBoyState) u32 {
    state.registers.SP = state.registers.getWord(Registers.Word.HL);
    return MCycle * 2;
}

/// Set the value of register A to the bitwise OR between A and the value in register r8
pub fn or_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    state.registers.A = state.registers.A | state.registers.getByte(r8);

    if (state.registers.A == 0) {
        state.registers.F.Z = true;
    }

    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle;
}

/// Set the value of register A to the bitwise OR between A and the value in memory at the address in register HL
pub fn or_a_vhl(state: *GameBoyState) u32 {
    state.registers.A = state.registers.A | state.mmu.read(state.registers.getWord(Registers.Word.HL));

    if (state.registers.A == 0) {
        state.registers.F.Z = true;
    }

    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 2;
}

/// Set the value of register A to the bitwise OR between A and the value in n8
pub fn or_a_n8(state: *GameBoyState, n8: *const u8) u32 {
    state.registers.A = state.registers.A | n8.*;

    if (state.registers.A == 0) {
        state.registers.F.Z = true;
    }

    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 2;
}

/// Pop register AF from the stack
pub fn pop_af(state: *GameBoyState) u32 {
    state.registers.F = @as(Flags, @bitCast(state.mmu.read(state.registers.SP)));
    state.registers.A = state.mmu.read(state.registers.SP + 1);
    state.registers.SP += 2;

    return MCycle * 3;
}

/// Pop register r16 from the stack
pub fn pop_r16(state: *GameBoyState, r16: Registers.Word) u32 {
    switch (r16) {
        Registers.Word.BC => {
            state.registers.B = state.mmu.read(state.registers.SP + 1);
            state.registers.C = state.mmu.read(state.registers.SP);
            state.registers.SP += 2;
        },
        Registers.Word.DE => {
            state.registers.D = state.mmu.read(state.registers.SP + 1);
            state.registers.E = state.mmu.read(state.registers.SP);
            state.registers.SP += 2;
        },
        Registers.Word.HL => {
            state.registers.H = state.mmu.read(state.registers.SP + 1);
            state.registers.L = state.mmu.read(state.registers.SP);
            state.registers.SP += 2;
        },
        else => {},
    }

    return MCycle * 3;
}

/// Push register AF to the stack
pub fn push_af(state: *GameBoyState) u32 {
    state.registers.SP -= 2;
    state.mmu.write(state.registers.SP + 1, state.registers.A);
    state.mmu.write(state.registers.SP, @as(u8, @bitCast(state.registers.F)));

    return MCycle * 4;
}

/// Push register r16 to the stack
pub fn push_r16(state: *GameBoyState, r16: Registers.Word) u32 {
    state.registers.SP -= 2;
    switch (r16) {
        Registers.Word.BC => {
            state.mmu.write(state.registers.SP + 1, state.registers.B);
            state.mmu.write(state.registers.SP, state.registers.C);
        },
        Registers.Word.DE => {
            state.mmu.write(state.registers.SP + 1, state.registers.D);
            state.mmu.write(state.registers.SP, state.registers.E);
        },
        Registers.Word.HL => {
            state.mmu.write(state.registers.SP + 1, state.registers.H);
            state.mmu.write(state.registers.SP, state.registers.L);
        },
        else => {},
    }

    return MCycle * 4;
}

/// Reset bit vu3 of register r8
pub fn res_u3_r8(state: *GameBoyState, vu3: u3, r8: Registers.Byte) u32 {
    // Create a mask with 1 at the position vu3 by left-shifting 1 by vu3 (e.g. 0b00000001 << 6 -> 0b01000000)
    const mask = @as(u8, 1) << vu3;

    // Applies a bitwise not to the mask (e.g. ~0b01000000 -> 0b10111111)
    const invertedMask = ~mask;

    // Applies a bitwise AND between the register value and the inverted mask (e.g. 0b11111111 & 0b10111111 -> 0b10111111)
    state.registers.setByte(r8, state.registers.getByte(r8) & invertedMask);
    return MCycle * 2;
}

/// Reset bit vu3 of memory at HL
pub fn res_u3_vhl(state: *GameBoyState, vu3: u3) u32 {
    // Create a mask with 1 at the position vu3 by left-shifting 1 by vu3 (e.g. 0b00000001 << 6 -> 0b01000000)
    const mask = @as(u8, 1) << vu3;

    // Applies a bitwise not to the mask (e.g. ~0b01000000 -> 0b10111111)
    const invertedMask = ~mask;

    // Applies a bitwise AND between the register value and the inverted mask (e.g. 0b11111111 & 0b10111111 -> 0b10111111)
    state.mmu.write(state.registers.getWord(Registers.Word.HL), state.mmu.read(state.registers.getWord(Registers.Word.HL)) & invertedMask);
    return MCycle * 4;
}

/// Returns from a function call
pub fn ret(state: *GameBoyState) u32 {
    state.registers.PC = @as(u16, state.mmu.read(state.registers.SP));
    state.registers.PC |= @as(u16, state.mmu.read(state.registers.SP + 1)) << 8;
    state.registers.SP += 2;
    return MCycle * 4;
}

/// Returns from a function call if condition is met
pub fn ret_cc(state: *GameBoyState, cc: ConditionCode) u32 {
    switch (cc) {
        ConditionCode.Z => {
            if (state.registers.F.Z) {
                return ret(state) + MCycle;
            }
        },
        ConditionCode.C => {
            if (state.registers.F.C) {
                return ret(state) + MCycle;
            }
        },
        ConditionCode.NZ => {
            if (!state.registers.F.Z) {
                return ret(state) + MCycle;
            }
        },
        ConditionCode.NC => {
            if (!state.registers.F.C) {
                return ret(state) + MCycle;
            }
        },
    }

    return MCycle * 2;
}

/// Returns from a function call and enables interrupts
pub fn reti(state: *GameBoyState) u32 {
    const cycles = ret(state);
    state.ime = true;
    return cycles;
}

/// Rotates the value of register r8 left, through the carry flag
pub fn rl_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C)));
    // Shift left and set least significant bit to carry flag
    state.registers.setByte(r8, (val << 1) | (carry));

    // Set carry flag to the most significant bit of the original register value
    state.registers.F.C = (val >> 7) == 1;

    // Set zero flag if register value is 0
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 2;
}

/// Rotates the value of memory location specified by HL left, through the carry flag
pub fn rl_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C)));
    // Shift left and set least significant bit to carry flag
    state.mmu.write(state.registers.getWord(Registers.Word.HL), (val << 1) | (carry));

    // Set carry flag to the most significant bit of the original register value
    state.registers.F.C = (val >> 7) == 1;

    // Set zero flag if register value is 0
    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 4;
}

/// Rotates the value of register A left, through the carry flag
pub fn rla(state: *GameBoyState) u32 {
    const val = state.registers.A;
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C)));

    // Shift left and set least significant bit to carry flag (aka rotate left)
    state.registers.A = (val << 1) | (carry);

    // Set carry flag to the most significant bit of the original register value
    state.registers.F.C = (val >> 7) == 1;

    state.registers.F.Z = false;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle;
}

/// Rotates the value of register r8 left and sets the carry flag
pub fn rlc_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);

    // Rotate the register left
    state.registers.setByte(r8, (val << 1) | (val >> 7));

    // Set carry flag to the most significant bit of the original register value
    state.registers.F.C = (val >> 7) == 1;

    // Set zero flag if register value is 0
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 2;
}

/// Rotates the value of memory location specified by HL left and sets the carry flag
pub fn rlc_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));

    // Rotate the memory location left
    state.mmu.write(state.registers.getWord(Registers.Word.HL), (val << 1) | (val >> 7));

    // Set carry flag to the most significant bit of the original register value
    state.registers.F.C = (val >> 7) == 1;

    // Set zero flag if register value is 0
    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 4;
}

/// Rotates the value of register A left and sets the carry flag
pub fn rlca(state: *GameBoyState) u32 {
    const val = state.registers.A;

    // Rotate the register left
    state.registers.A = (val << 1) | (val >> 7);

    // Set carry flag to the most significant bit of the original register value
    state.registers.F.C = (val >> 7) == 1;

    state.registers.F.Z = false;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle;
}

/// Rotates the value of register r8 right through the carry flag
pub fn rr_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C))) << 7;

    // Rotate the register right
    state.registers.setByte(r8, (val >> 1) | carry);

    // Set carry flag to the least significant bit of the original register value
    state.registers.F.C = (val & 1) == 1;

    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 2;
}

/// Rotates the value of memory location specified by HL right through the carry flag
pub fn rr_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C))) << 7;

    // Write the rotated value to memory
    state.mmu.write(state.registers.getWord(Registers.Word.HL), (val >> 1) | carry);

    // Set carry flag to the least significant bit of the original register value
    state.registers.F.C = (val & 1) == 1;

    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 4;
}

/// Rotates the value of register A right through the carry flag
pub fn rra(state: *GameBoyState) u32 {
    const val = state.registers.A;
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C))) << 7;

    // Rotate the register right
    state.registers.A = (val >> 1) | carry;

    // Set carry flag to the least significant bit of the original register value
    state.registers.F.C = (val & 1) == 1;

    state.registers.F.Z = false;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle;
}

/// Rotates the value of register r8 right and sets the carry flag
pub fn rrc_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);

    // Rotate the register right
    state.registers.setByte(r8, (val >> 1) | (val << 7));

    // Set carry flag to the result
    state.registers.F.C = state.registers.getByte(r8) & 1 == 1;

    // Set if result is zero
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 2;
}

/// Rotates the value of memory location specified by HL right and sets the carry flag
pub fn rrc_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));

    // Write the rotated value to memory
    state.mmu.write(state.registers.getWord(Registers.Word.HL), (val >> 1) | (val << 7));

    // Set carry flag to the least significant bit of the original register value
    state.registers.F.C = (val & 1) == 1;

    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle * 4;
}

/// Rotates the value of register A right and sets the carry flag
pub fn rrca(state: *GameBoyState) u32 {
    const val = state.registers.A;

    // Rotate the register right
    state.registers.A = (val >> 1) | (val << 7);

    // Set carry flag to the least significant bit of the original register value
    state.registers.F.C = (val & 1) == 1;

    state.registers.F.Z = false;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle;
}

/// Jump to the specified vector
pub fn rst_vec(state: *GameBoyState, vec: u16) u32 {
    switch (vec) {
        0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38 => {
            // Save the next instruction address
            const next_instr_addr = state.registers.PC;

            // Split 16-bit value into high and low 8-bit values
            const high: u8 = @intCast(next_instr_addr >> 8);
            const low: u8 = @intCast(next_instr_addr & 0xFF);

            // Decrement SP by 2 (16 bits)
            state.registers.SP -= 2;

            // Write high and low bytes to stack
            state.mmu.write(state.registers.SP + 1, high);
            state.mmu.write(state.registers.SP, low);

            // jump to the new address
            state.registers.PC = vec;
        },
        else => {},
    }

    return MCycle * 4;
}

/// Subtract the value in r8 from register A
pub fn sbc_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C)));

    const borrow, const overflow = @subWithOverflow(state.registers.A, val +% carry);

    state.registers.A = borrow;

    // Set if borrow (i.e. if (r8 + carry) > A)
    state.registers.F.C = overflow != 0;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = true;
    state.registers.F.H = (state.registers.A & 0x10) != 0;

    return MCycle;
}

/// Subtract the value in memory at the address in register HL from register A
pub fn sbc_a_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C)));

    const borrow, const overflow = @subWithOverflow(state.registers.A, val +% carry);

    state.registers.A = borrow;

    // Set if borrow (i.e. if (r8 + carry) > A)
    state.registers.F.C = overflow != 0;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = true;
    state.registers.F.H = (state.registers.A & 0x10) != 0;

    return MCycle * 2;
}

/// Subtract the value in n8 from register A
pub fn sbc_a_n8(state: *GameBoyState, val: *const u8) u32 {
    const carry = @as(u8, @as(u1, @bitCast(state.registers.F.C)));

    const borrow, const overflow = @subWithOverflow(state.registers.A, val.* +% carry);

    state.registers.A = borrow;

    // Set if borrow (i.e. if (r8 + carry) > A)
    state.registers.F.C = overflow != 0;
    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = true;
    state.registers.F.H = (state.registers.A & 0x10) != 0;

    return MCycle * 2;
}

/// Set the carry flag
pub fn scf(state: *GameBoyState) u32 {
    state.registers.F.C = true;
    state.registers.F.N = false;
    state.registers.F.H = false;

    return MCycle;
}

/// Set the bit vu3 in register r8 to 1.
pub fn set_u3_r8(state: *GameBoyState, comptime vu3: u3, r8: Registers.Byte) u32 {
    state.registers.setByte(r8, state.registers.getByte(r8) | (1 << vu3));
    return MCycle * 2;
}

pub fn set_u3_vhl(state: *GameBoyState, comptime vu3: u3) u32 {
    state.mmu.write(state.registers.getWord(Registers.Word.HL), state.mmu.read(state.registers.getWord(Registers.Word.HL)) | (1 << vu3));
    return MCycle * 4;
}

pub fn sla_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const carry = val & 0x80 != 0;
    state.registers.setByte(r8, val << 1);
    state.registers.F.C = carry;
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle * 2;
}

pub fn sla_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const carry = val & 0x80 != 0;
    state.mmu.write(state.registers.getWord(Registers.Word.HL), val << 1);
    state.registers.F.C = carry;
    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle * 4;
}

pub fn sra_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const carry = val & 0x1 != 0;
    // Shift right but keep bit 7 unchanged
    state.registers.setByte(r8, (val >> 1) | (val & 0x80));
    state.registers.F.C = carry;
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle * 2;
}

pub fn sra_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const carry = val & 0x1 != 0;
    // Shift right but keep bit 7 unchanged
    state.mmu.write(state.registers.getWord(Registers.Word.HL), (val >> 1) | (val & 0x80));
    state.registers.F.C = carry;
    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle * 4;
}

pub fn srl_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const carry = val & 0x1 != 0;
    state.registers.setByte(r8, val >> 1);
    state.registers.F.C = carry;
    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle * 2;
}

pub fn srl_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const carry = val & 0x1 != 0;
    state.mmu.write(state.registers.getWord(Registers.Word.HL), val >> 1);
    state.registers.F.C = carry;
    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    return MCycle * 4;
}

/// TODO: Implement stop (https://gbdev.io/pandocs/Reducing_Power_Consumption.html#using-the-stop-instruction)
pub fn stop(state: *GameBoyState) u32 {
    _ = state;

    return noop();
}

pub fn sub_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    const result, const overflow = @subWithOverflow(state.registers.A, val);
    state.registers.setByte(Registers.Byte.A, result);
    state.registers.F.C = overflow == 1;
    state.registers.F.Z = result == 0;
    state.registers.F.N = true;
    // Set if half-borrow
    state.registers.F.H = (state.registers.A & 0xF) < (val & 0xF);
    return MCycle;
}

pub fn sub_a_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    const result, const overflow = @subWithOverflow(state.registers.A, val);
    state.registers.setByte(Registers.Byte.A, result);
    state.registers.F.C = overflow == 1;
    state.registers.F.Z = result == 0;
    state.registers.F.N = true;
    // Set if half-borrow
    state.registers.F.H = (state.registers.A & 0xF) < (val & 0xF);
    return MCycle * 2;
}

pub fn sub_a_n8(state: *GameBoyState, val: *const u8) u32 {
    const result, const overflow = @subWithOverflow(state.registers.A, val.*);
    state.registers.setByte(Registers.Byte.A, result);
    state.registers.F.C = overflow == 1;
    state.registers.F.Z = result == 0;
    state.registers.F.N = true;
    // Set if half-borrow
    state.registers.F.H = (state.registers.A & 0xF) < (val.* & 0xF);
    return MCycle * 2;
}

pub fn swap_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    state.registers.setByte(r8, (val >> 4) | (val << 4));

    state.registers.F.Z = state.registers.getByte(r8) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 2;
}

pub fn swap_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.mmu.write(state.registers.getWord(Registers.Word.HL), (val >> 4) | (val << 4));

    state.registers.F.Z = state.mmu.read(state.registers.getWord(Registers.Word.HL)) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 4;
}

pub fn xor_a_r8(state: *GameBoyState, r8: Registers.Byte) u32 {
    const val = state.registers.getByte(r8);
    state.registers.setByte(Registers.Byte.A, state.registers.A ^ val);

    state.registers.F.Z = state.registers.getByte(Registers.Byte.A) == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 1;
}

pub fn xor_a_vhl(state: *GameBoyState) u32 {
    const val = state.mmu.read(state.registers.getWord(Registers.Word.HL));
    state.registers.A = state.registers.A ^ val;

    state.registers.F.Z = state.registers.A == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 2;
}

pub fn xor_a_n8(state: *GameBoyState, val: *const u8) u32 {
    const result = state.registers.A ^ val.*;
    state.registers.setByte(Registers.Byte.A, result);

    state.registers.F.Z = result == 0;
    state.registers.F.N = false;
    state.registers.F.H = false;
    state.registers.F.C = false;

    return MCycle * 2;
}

pub fn fetch(state: *GameBoyState) u8 {
    const byte = state.mmu.read(state.registers.PC);
    state.registers.PC +%= 1;
    return byte;
}

pub fn fetch16(state: *GameBoyState) u16 {
    const val = state.mmu.read16(state.registers.PC);
    state.registers.PC +%= 2;
    return val;
}

pub fn execute(state: *GameBoyState, opcode: u8) u32 {
    // If EI delay is set, set IME and clear EI delay
    if (state.ei_delay) {
        state.ei_delay = false;
        state.ime = true;
    }

    return switch (opcode) {
        0x00 => noop(),
        0x01 => ld_r16_n16(state, Registers.Word.BC, &fetch16(state)),
        0x02 => ld_vr16_a(state, Registers.Word.BC),
        0x03 => inc_r16(state, Registers.Word.BC),
        0x04 => inc_r8(state, Registers.Byte.B),
        0x05 => dec_r8(state, Registers.Byte.B),
        0x06 => ld_r8_n8(state, Registers.Byte.B, &fetch(state)),
        0x07 => rlca(state),
        0x08 => ld_vn16_sp(state, &fetch16(state)),
        0x09 => add_hl_r16(state, Registers.Word.BC),
        0x0A => ld_a_vr16(state, Registers.Word.BC),
        0x0B => dec_r16(state, Registers.Word.BC),
        0x0C => inc_r8(state, Registers.Byte.C),
        0x0D => dec_r8(state, Registers.Byte.C),
        0x0E => ld_r8_n8(state, Registers.Byte.C, &fetch(state)),
        0x0F => rrca(state),
        0x10 => stop(state),
        0x11 => ld_r16_n16(state, Registers.Word.DE, &fetch16(state)),
        0x12 => ld_vr16_a(state, Registers.Word.DE),
        0x13 => inc_r16(state, Registers.Word.DE),
        0x14 => inc_r8(state, Registers.Byte.D),
        0x15 => dec_r8(state, Registers.Byte.D),
        0x16 => ld_r8_n8(state, Registers.Byte.D, &fetch(state)),
        0x17 => rla(state),
        0x18 => jr_e8(state, @as(i8, @bitCast(fetch(state)))),
        0x19 => add_hl_r16(state, Registers.Word.DE),
        0x1A => ld_a_vr16(state, Registers.Word.DE),
        0x1B => dec_r16(state, Registers.Word.DE),
        0x1C => inc_r8(state, Registers.Byte.E),
        0x1D => dec_r8(state, Registers.Byte.E),
        0x1E => ld_r8_n8(state, Registers.Byte.E, &fetch(state)),
        0x1F => rra(state),
        0x20 => jr_cc_e8(state, @as(i8, @bitCast(fetch(state))), ConditionCode.NZ),
        0x21 => ld_r16_n16(state, Registers.Word.HL, &fetch16(state)),
        0x22 => ld_hli_a(state),
        0x23 => inc_r16(state, Registers.Word.HL),
        0x24 => inc_r8(state, Registers.Byte.H),
        0x25 => dec_r8(state, Registers.Byte.H),
        0x26 => ld_r8_n8(state, Registers.Byte.H, &fetch(state)),
        0x27 => daa(state),
        0x28 => jr_cc_e8(state, @as(i8, @bitCast(fetch(state))), ConditionCode.Z),
        0x29 => add_hl_r16(state, Registers.Word.HL),
        0x2A => ld_a_hli(state),
        0x2B => dec_r16(state, Registers.Word.HL),
        0x2C => inc_r8(state, Registers.Byte.L),
        0x2D => dec_r8(state, Registers.Byte.L),
        0x2E => ld_r8_n8(state, Registers.Byte.L, &fetch(state)),
        0x2F => cpl(state),
        0x30 => jr_cc_e8(state, @as(i8, @bitCast(fetch(state))), ConditionCode.NC),
        0x31 => ld_sp_n16(state, &fetch16(state)),
        0x32 => ld_hld_a(state),
        0x33 => inc_sp(state),
        0x34 => inc_vhl(state),
        0x35 => dec_vhl(state),
        0x36 => ld_vhl_n8(state, &fetch(state)),
        0x37 => scf(state),
        0x38 => jr_cc_e8(state, @as(i8, @bitCast(fetch(state))), ConditionCode.C),
        0x39 => add_hl_sp(state),
        0x3A => ld_a_hld(state),
        0x3B => dec_sp(state),
        0x3C => inc_r8(state, Registers.Byte.A),
        0x3D => dec_r8(state, Registers.Byte.A),
        0x3E => ld_r8_n8(state, Registers.Byte.A, &fetch(state)),
        0x3F => ccf(state),
        0x40 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.B),
        0x41 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.C),
        0x42 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.D),
        0x43 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.E),
        0x44 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.H),
        0x45 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.L),
        0x46 => ld_r8_vhl(state, Registers.Byte.B),
        0x47 => ld_r8_r8(state, Registers.Byte.B, Registers.Byte.A),
        0x48 => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.B),
        0x49 => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.C),
        0x4A => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.D),
        0x4B => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.E),
        0x4C => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.H),
        0x4D => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.L),
        0x4E => ld_r8_vhl(state, Registers.Byte.C),
        0x4F => ld_r8_r8(state, Registers.Byte.C, Registers.Byte.A),
        0x50 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.B),
        0x51 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.C),
        0x52 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.D),
        0x53 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.E),
        0x54 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.H),
        0x55 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.L),
        0x56 => ld_r8_vhl(state, Registers.Byte.D),
        0x57 => ld_r8_r8(state, Registers.Byte.D, Registers.Byte.A),
        0x58 => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.B),
        0x59 => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.C),
        0x5A => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.D),
        0x5B => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.E),
        0x5C => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.H),
        0x5D => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.L),
        0x5E => ld_r8_vhl(state, Registers.Byte.E),
        0x5F => ld_r8_r8(state, Registers.Byte.E, Registers.Byte.A),
        0x60 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.B),
        0x61 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.C),
        0x62 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.D),
        0x63 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.E),
        0x64 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.H),
        0x65 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.L),
        0x66 => ld_r8_vhl(state, Registers.Byte.H),
        0x67 => ld_r8_r8(state, Registers.Byte.H, Registers.Byte.A),
        0x68 => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.B),
        0x69 => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.C),
        0x6A => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.D),
        0x6B => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.E),
        0x6C => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.H),
        0x6D => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.L),
        0x6E => ld_r8_vhl(state, Registers.Byte.L),
        0x6F => ld_r8_r8(state, Registers.Byte.L, Registers.Byte.A),
        0x70 => ld_vhl_r8(state, Registers.Byte.B),
        0x71 => ld_vhl_r8(state, Registers.Byte.C),
        0x72 => ld_vhl_r8(state, Registers.Byte.D),
        0x73 => ld_vhl_r8(state, Registers.Byte.E),
        0x74 => ld_vhl_r8(state, Registers.Byte.H),
        0x75 => ld_vhl_r8(state, Registers.Byte.L),
        0x76 => halt(state),
        0x77 => ld_vhl_r8(state, Registers.Byte.A),
        0x78 => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.B),
        0x79 => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.C),
        0x7A => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.D),
        0x7B => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.E),
        0x7C => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.H),
        0x7D => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.L),
        0x7E => ld_r8_vhl(state, Registers.Byte.A),
        0x7F => ld_r8_r8(state, Registers.Byte.A, Registers.Byte.A),
        0x80 => add_a_r8(state, Registers.Byte.B),
        0x81 => add_a_r8(state, Registers.Byte.C),
        0x82 => add_a_r8(state, Registers.Byte.D),
        0x83 => add_a_r8(state, Registers.Byte.E),
        0x84 => add_a_r8(state, Registers.Byte.H),
        0x85 => add_a_r8(state, Registers.Byte.L),
        0x86 => add_a_vhl(state),
        0x87 => add_a_r8(state, Registers.Byte.A),
        0x88 => adc_a_r8(state, Registers.Byte.B),
        0x89 => adc_a_r8(state, Registers.Byte.C),
        0x8A => adc_a_r8(state, Registers.Byte.D),
        0x8B => adc_a_r8(state, Registers.Byte.E),
        0x8C => adc_a_r8(state, Registers.Byte.H),
        0x8D => adc_a_r8(state, Registers.Byte.L),
        0x8E => adc_a_vhl(state),
        0x8F => adc_a_r8(state, Registers.Byte.A),
        0x90 => sub_a_r8(state, Registers.Byte.B),
        0x91 => sub_a_r8(state, Registers.Byte.C),
        0x92 => sub_a_r8(state, Registers.Byte.D),
        0x93 => sub_a_r8(state, Registers.Byte.E),
        0x94 => sub_a_r8(state, Registers.Byte.H),
        0x95 => sub_a_r8(state, Registers.Byte.L),
        0x96 => sub_a_vhl(state),
        0x97 => sub_a_r8(state, Registers.Byte.A),
        0x98 => sbc_a_r8(state, Registers.Byte.B),
        0x99 => sbc_a_r8(state, Registers.Byte.C),
        0x9A => sbc_a_r8(state, Registers.Byte.D),
        0x9B => sbc_a_r8(state, Registers.Byte.E),
        0x9C => sbc_a_r8(state, Registers.Byte.H),
        0x9D => sbc_a_r8(state, Registers.Byte.L),
        0x9E => sbc_a_vhl(state),
        0x9F => sbc_a_r8(state, Registers.Byte.A),
        0xA0 => and_a_r8(state, Registers.Byte.B),
        0xA1 => and_a_r8(state, Registers.Byte.C),
        0xA2 => and_a_r8(state, Registers.Byte.D),
        0xA3 => and_a_r8(state, Registers.Byte.E),
        0xA4 => and_a_r8(state, Registers.Byte.H),
        0xA5 => and_a_r8(state, Registers.Byte.L),
        0xA6 => and_a_vhl(state),
        0xA7 => and_a_r8(state, Registers.Byte.A),
        0xA8 => xor_a_r8(state, Registers.Byte.B),
        0xA9 => xor_a_r8(state, Registers.Byte.C),
        0xAA => xor_a_r8(state, Registers.Byte.D),
        0xAB => xor_a_r8(state, Registers.Byte.E),
        0xAC => xor_a_r8(state, Registers.Byte.H),
        0xAD => xor_a_r8(state, Registers.Byte.L),
        0xAE => xor_a_vhl(state),
        0xAF => xor_a_r8(state, Registers.Byte.A),
        0xB0 => or_a_r8(state, Registers.Byte.B),
        0xB1 => or_a_r8(state, Registers.Byte.C),
        0xB2 => or_a_r8(state, Registers.Byte.D),
        0xB3 => or_a_r8(state, Registers.Byte.E),
        0xB4 => or_a_r8(state, Registers.Byte.H),
        0xB5 => or_a_r8(state, Registers.Byte.L),
        0xB6 => or_a_vhl(state),
        0xB7 => or_a_r8(state, Registers.Byte.A),
        0xB8 => cp_a_r8(state, Registers.Byte.B),
        0xB9 => cp_a_r8(state, Registers.Byte.C),
        0xBA => cp_a_r8(state, Registers.Byte.D),
        0xBB => cp_a_r8(state, Registers.Byte.E),
        0xBC => cp_a_r8(state, Registers.Byte.H),
        0xBD => cp_a_r8(state, Registers.Byte.L),
        0xBE => cp_a_vhl(state),
        0xBF => cp_a_r8(state, Registers.Byte.A),
        0xC0 => ret_cc(state, ConditionCode.NZ),
        0xC1 => pop_r16(state, Registers.Word.BC),
        0xC2 => jp_cc_n16(state, &fetch16(state), ConditionCode.NZ),
        0xC3 => jp_n16(state, &fetch16(state)),
        0xC4 => call_cc_n16(state, &fetch16(state), ConditionCode.NZ),
        0xC5 => push_r16(state, Registers.Word.BC),
        0xC6 => add_a_n8(state, &fetch(state)),
        0xC7 => rst_vec(state, 0x00),
        0xC8 => ret_cc(state, ConditionCode.Z),
        0xC9 => ret(state),
        0xCA => jp_cc_n16(state, &fetch16(state), ConditionCode.Z),
        0xCB => {
            const cbOpcode = fetch(state);
            return switch (cbOpcode) {
                0x00 => rlc_r8(state, Registers.Byte.B),
                0x01 => rlc_r8(state, Registers.Byte.C),
                0x02 => rlc_r8(state, Registers.Byte.D),
                0x03 => rlc_r8(state, Registers.Byte.E),
                0x04 => rlc_r8(state, Registers.Byte.H),
                0x05 => rlc_r8(state, Registers.Byte.L),
                0x06 => rlc_vhl(state),
                0x07 => rlc_r8(state, Registers.Byte.A),
                0x08 => rrc_r8(state, Registers.Byte.B),
                0x09 => rrc_r8(state, Registers.Byte.C),
                0x0A => rrc_r8(state, Registers.Byte.D),
                0x0B => rrc_r8(state, Registers.Byte.E),
                0x0C => rrc_r8(state, Registers.Byte.H),
                0x0D => rrc_r8(state, Registers.Byte.L),
                0x0E => rrc_vhl(state),
                0x0F => rrc_r8(state, Registers.Byte.A),
                0x10 => rl_r8(state, Registers.Byte.B),
                0x11 => rl_r8(state, Registers.Byte.C),
                0x12 => rl_r8(state, Registers.Byte.D),
                0x13 => rl_r8(state, Registers.Byte.E),
                0x14 => rl_r8(state, Registers.Byte.H),
                0x15 => rl_r8(state, Registers.Byte.L),
                0x16 => rl_vhl(state),
                0x17 => rl_r8(state, Registers.Byte.A),
                0x18 => rr_r8(state, Registers.Byte.B),
                0x19 => rr_r8(state, Registers.Byte.C),
                0x1A => rr_r8(state, Registers.Byte.D),
                0x1B => rr_r8(state, Registers.Byte.E),
                0x1C => rr_r8(state, Registers.Byte.H),
                0x1D => rr_r8(state, Registers.Byte.L),
                0x1E => rr_vhl(state),
                0x1F => rr_r8(state, Registers.Byte.A),
                0x20 => sla_r8(state, Registers.Byte.B),
                0x21 => sla_r8(state, Registers.Byte.C),
                0x22 => sla_r8(state, Registers.Byte.D),
                0x23 => sla_r8(state, Registers.Byte.E),
                0x24 => sla_r8(state, Registers.Byte.H),
                0x25 => sla_r8(state, Registers.Byte.L),
                0x26 => sla_vhl(state),
                0x27 => sla_r8(state, Registers.Byte.A),
                0x28 => sra_r8(state, Registers.Byte.B),
                0x29 => sra_r8(state, Registers.Byte.C),
                0x2A => sra_r8(state, Registers.Byte.D),
                0x2B => sra_r8(state, Registers.Byte.E),
                0x2C => sra_r8(state, Registers.Byte.H),
                0x2D => sra_r8(state, Registers.Byte.L),
                0x2E => sra_vhl(state),
                0x2F => sra_r8(state, Registers.Byte.A),
                0x30 => swap_r8(state, Registers.Byte.B),
                0x31 => swap_r8(state, Registers.Byte.C),
                0x32 => swap_r8(state, Registers.Byte.D),
                0x33 => swap_r8(state, Registers.Byte.E),
                0x34 => swap_r8(state, Registers.Byte.H),
                0x35 => swap_r8(state, Registers.Byte.L),
                0x36 => swap_vhl(state),
                0x37 => swap_r8(state, Registers.Byte.A),
                0x38 => srl_r8(state, Registers.Byte.B),
                0x39 => srl_r8(state, Registers.Byte.C),
                0x3A => srl_r8(state, Registers.Byte.D),
                0x3B => srl_r8(state, Registers.Byte.E),
                0x3C => srl_r8(state, Registers.Byte.H),
                0x3D => srl_r8(state, Registers.Byte.L),
                0x3E => srl_vhl(state),
                0x3F => srl_r8(state, Registers.Byte.A),
                0x40 => bit_u3_r8(state, 0, Registers.Byte.B),
                0x41 => bit_u3_r8(state, 0, Registers.Byte.C),
                0x42 => bit_u3_r8(state, 0, Registers.Byte.D),
                0x43 => bit_u3_r8(state, 0, Registers.Byte.E),
                0x44 => bit_u3_r8(state, 0, Registers.Byte.H),
                0x45 => bit_u3_r8(state, 0, Registers.Byte.L),
                0x46 => bit_u3_vhl(state, 0),
                0x47 => bit_u3_r8(state, 0, Registers.Byte.A),
                0x48 => bit_u3_r8(state, 1, Registers.Byte.B),
                0x49 => bit_u3_r8(state, 1, Registers.Byte.C),
                0x4A => bit_u3_r8(state, 1, Registers.Byte.D),
                0x4B => bit_u3_r8(state, 1, Registers.Byte.E),
                0x4C => bit_u3_r8(state, 1, Registers.Byte.H),
                0x4D => bit_u3_r8(state, 1, Registers.Byte.L),
                0x4E => bit_u3_vhl(state, 1),
                0x4F => bit_u3_r8(state, 1, Registers.Byte.A),
                0x50 => bit_u3_r8(state, 2, Registers.Byte.B),
                0x51 => bit_u3_r8(state, 2, Registers.Byte.C),
                0x52 => bit_u3_r8(state, 2, Registers.Byte.D),
                0x53 => bit_u3_r8(state, 2, Registers.Byte.E),
                0x54 => bit_u3_r8(state, 2, Registers.Byte.H),
                0x55 => bit_u3_r8(state, 2, Registers.Byte.L),
                0x56 => bit_u3_vhl(state, 2),
                0x57 => bit_u3_r8(state, 2, Registers.Byte.A),
                0x58 => bit_u3_r8(state, 3, Registers.Byte.B),
                0x59 => bit_u3_r8(state, 3, Registers.Byte.C),
                0x5A => bit_u3_r8(state, 3, Registers.Byte.D),
                0x5B => bit_u3_r8(state, 3, Registers.Byte.E),
                0x5C => bit_u3_r8(state, 3, Registers.Byte.H),
                0x5D => bit_u3_r8(state, 3, Registers.Byte.L),
                0x5E => bit_u3_vhl(state, 3),
                0x5F => bit_u3_r8(state, 3, Registers.Byte.A),
                0x60 => bit_u3_r8(state, 4, Registers.Byte.B),
                0x61 => bit_u3_r8(state, 4, Registers.Byte.C),
                0x62 => bit_u3_r8(state, 4, Registers.Byte.D),
                0x63 => bit_u3_r8(state, 4, Registers.Byte.E),
                0x64 => bit_u3_r8(state, 4, Registers.Byte.H),
                0x65 => bit_u3_r8(state, 4, Registers.Byte.L),
                0x66 => bit_u3_vhl(state, 4),
                0x67 => bit_u3_r8(state, 4, Registers.Byte.A),
                0x68 => bit_u3_r8(state, 5, Registers.Byte.B),
                0x69 => bit_u3_r8(state, 5, Registers.Byte.C),
                0x6A => bit_u3_r8(state, 5, Registers.Byte.D),
                0x6B => bit_u3_r8(state, 5, Registers.Byte.E),
                0x6C => bit_u3_r8(state, 5, Registers.Byte.H),
                0x6D => bit_u3_r8(state, 5, Registers.Byte.L),
                0x6E => bit_u3_vhl(state, 5),
                0x6F => bit_u3_r8(state, 5, Registers.Byte.A),
                0x70 => bit_u3_r8(state, 6, Registers.Byte.B),
                0x71 => bit_u3_r8(state, 6, Registers.Byte.C),
                0x72 => bit_u3_r8(state, 6, Registers.Byte.D),
                0x73 => bit_u3_r8(state, 6, Registers.Byte.E),
                0x74 => bit_u3_r8(state, 6, Registers.Byte.H),
                0x75 => bit_u3_r8(state, 6, Registers.Byte.L),
                0x76 => bit_u3_vhl(state, 6),
                0x77 => bit_u3_r8(state, 6, Registers.Byte.A),
                0x78 => bit_u3_r8(state, 7, Registers.Byte.B),
                0x79 => bit_u3_r8(state, 7, Registers.Byte.C),
                0x7A => bit_u3_r8(state, 7, Registers.Byte.D),
                0x7B => bit_u3_r8(state, 7, Registers.Byte.E),
                0x7C => bit_u3_r8(state, 7, Registers.Byte.H),
                0x7D => bit_u3_r8(state, 7, Registers.Byte.L),
                0x7E => bit_u3_vhl(state, 7),
                0x7F => bit_u3_r8(state, 7, Registers.Byte.A),
                0x80 => res_u3_r8(state, 0, Registers.Byte.B),
                0x81 => res_u3_r8(state, 0, Registers.Byte.C),
                0x82 => res_u3_r8(state, 0, Registers.Byte.D),
                0x83 => res_u3_r8(state, 0, Registers.Byte.E),
                0x84 => res_u3_r8(state, 0, Registers.Byte.H),
                0x85 => res_u3_r8(state, 0, Registers.Byte.L),
                0x86 => res_u3_vhl(state, 0),
                0x87 => res_u3_r8(state, 0, Registers.Byte.A),
                0x88 => res_u3_r8(state, 1, Registers.Byte.B),
                0x89 => res_u3_r8(state, 1, Registers.Byte.C),
                0x8A => res_u3_r8(state, 1, Registers.Byte.D),
                0x8B => res_u3_r8(state, 1, Registers.Byte.E),
                0x8C => res_u3_r8(state, 1, Registers.Byte.H),
                0x8D => res_u3_r8(state, 1, Registers.Byte.L),
                0x8E => res_u3_vhl(state, 1),
                0x8F => res_u3_r8(state, 1, Registers.Byte.A),
                0x90 => res_u3_r8(state, 2, Registers.Byte.B),
                0x91 => res_u3_r8(state, 2, Registers.Byte.C),
                0x92 => res_u3_r8(state, 2, Registers.Byte.D),
                0x93 => res_u3_r8(state, 2, Registers.Byte.E),
                0x94 => res_u3_r8(state, 2, Registers.Byte.H),
                0x95 => res_u3_r8(state, 2, Registers.Byte.L),
                0x96 => res_u3_vhl(state, 2),
                0x97 => res_u3_r8(state, 2, Registers.Byte.A),
                0x98 => res_u3_r8(state, 3, Registers.Byte.B),
                0x99 => res_u3_r8(state, 3, Registers.Byte.C),
                0x9A => res_u3_r8(state, 3, Registers.Byte.D),
                0x9B => res_u3_r8(state, 3, Registers.Byte.E),
                0x9C => res_u3_r8(state, 3, Registers.Byte.H),
                0x9D => res_u3_r8(state, 3, Registers.Byte.L),
                0x9E => res_u3_vhl(state, 3),
                0x9F => res_u3_r8(state, 3, Registers.Byte.A),
                0xA0 => res_u3_r8(state, 4, Registers.Byte.B),
                0xA1 => res_u3_r8(state, 4, Registers.Byte.C),
                0xA2 => res_u3_r8(state, 4, Registers.Byte.D),
                0xA3 => res_u3_r8(state, 4, Registers.Byte.E),
                0xA4 => res_u3_r8(state, 4, Registers.Byte.H),
                0xA5 => res_u3_r8(state, 4, Registers.Byte.L),
                0xA6 => res_u3_vhl(state, 4),
                0xA7 => res_u3_r8(state, 4, Registers.Byte.A),
                0xA8 => res_u3_r8(state, 5, Registers.Byte.B),
                0xA9 => res_u3_r8(state, 5, Registers.Byte.C),
                0xAA => res_u3_r8(state, 5, Registers.Byte.D),
                0xAB => res_u3_r8(state, 5, Registers.Byte.E),
                0xAC => res_u3_r8(state, 5, Registers.Byte.H),
                0xAD => res_u3_r8(state, 5, Registers.Byte.L),
                0xAE => res_u3_vhl(state, 5),
                0xAF => res_u3_r8(state, 5, Registers.Byte.A),
                0xB0 => res_u3_r8(state, 6, Registers.Byte.B),
                0xB1 => res_u3_r8(state, 6, Registers.Byte.C),
                0xB2 => res_u3_r8(state, 6, Registers.Byte.D),
                0xB3 => res_u3_r8(state, 6, Registers.Byte.E),
                0xB4 => res_u3_r8(state, 6, Registers.Byte.H),
                0xB5 => res_u3_r8(state, 6, Registers.Byte.L),
                0xB6 => res_u3_vhl(state, 6),
                0xB7 => res_u3_r8(state, 6, Registers.Byte.A),
                0xB8 => res_u3_r8(state, 7, Registers.Byte.B),
                0xB9 => res_u3_r8(state, 7, Registers.Byte.C),
                0xBA => res_u3_r8(state, 7, Registers.Byte.D),
                0xBB => res_u3_r8(state, 7, Registers.Byte.E),
                0xBC => res_u3_r8(state, 7, Registers.Byte.H),
                0xBD => res_u3_r8(state, 7, Registers.Byte.L),
                0xBE => res_u3_vhl(state, 7),
                0xBF => res_u3_r8(state, 7, Registers.Byte.A),
                0xC0 => set_u3_r8(state, 0, Registers.Byte.B),
                0xC1 => set_u3_r8(state, 0, Registers.Byte.C),
                0xC2 => set_u3_r8(state, 0, Registers.Byte.D),
                0xC3 => set_u3_r8(state, 0, Registers.Byte.E),
                0xC4 => set_u3_r8(state, 0, Registers.Byte.H),
                0xC5 => set_u3_r8(state, 0, Registers.Byte.L),
                0xC6 => set_u3_vhl(state, 0),
                0xC7 => set_u3_r8(state, 0, Registers.Byte.A),
                0xC8 => set_u3_r8(state, 1, Registers.Byte.B),
                0xC9 => set_u3_r8(state, 1, Registers.Byte.C),
                0xCA => set_u3_r8(state, 1, Registers.Byte.D),
                0xCB => set_u3_r8(state, 1, Registers.Byte.E),
                0xCC => set_u3_r8(state, 1, Registers.Byte.H),
                0xCD => set_u3_r8(state, 1, Registers.Byte.L),
                0xCE => set_u3_vhl(state, 1),
                0xCF => set_u3_r8(state, 1, Registers.Byte.A),
                0xD0 => set_u3_r8(state, 2, Registers.Byte.B),
                0xD1 => set_u3_r8(state, 2, Registers.Byte.C),
                0xD2 => set_u3_r8(state, 2, Registers.Byte.D),
                0xD3 => set_u3_r8(state, 2, Registers.Byte.E),
                0xD4 => set_u3_r8(state, 2, Registers.Byte.H),
                0xD5 => set_u3_r8(state, 2, Registers.Byte.L),
                0xD6 => set_u3_vhl(state, 2),
                0xD7 => set_u3_r8(state, 2, Registers.Byte.A),
                0xD8 => set_u3_r8(state, 3, Registers.Byte.B),
                0xD9 => set_u3_r8(state, 3, Registers.Byte.C),
                0xDA => set_u3_r8(state, 3, Registers.Byte.D),
                0xDB => set_u3_r8(state, 3, Registers.Byte.E),
                0xDC => set_u3_r8(state, 3, Registers.Byte.H),
                0xDD => set_u3_r8(state, 3, Registers.Byte.L),
                0xDE => set_u3_vhl(state, 3),
                0xDF => set_u3_r8(state, 3, Registers.Byte.A),
                0xE0 => set_u3_r8(state, 4, Registers.Byte.B),
                0xE1 => set_u3_r8(state, 4, Registers.Byte.C),
                0xE2 => set_u3_r8(state, 4, Registers.Byte.D),
                0xE3 => set_u3_r8(state, 4, Registers.Byte.E),
                0xE4 => set_u3_r8(state, 4, Registers.Byte.H),
                0xE5 => set_u3_r8(state, 4, Registers.Byte.L),
                0xE6 => set_u3_vhl(state, 4),
                0xE7 => set_u3_r8(state, 4, Registers.Byte.A),
                0xE8 => set_u3_r8(state, 5, Registers.Byte.B),
                0xE9 => set_u3_r8(state, 5, Registers.Byte.C),
                0xEA => set_u3_r8(state, 5, Registers.Byte.D),
                0xEB => set_u3_r8(state, 5, Registers.Byte.E),
                0xEC => set_u3_r8(state, 5, Registers.Byte.H),
                0xED => set_u3_r8(state, 5, Registers.Byte.L),
                0xEE => set_u3_vhl(state, 5),
                0xEF => set_u3_r8(state, 5, Registers.Byte.A),
                0xF0 => set_u3_r8(state, 6, Registers.Byte.B),
                0xF1 => set_u3_r8(state, 6, Registers.Byte.C),
                0xF2 => set_u3_r8(state, 6, Registers.Byte.D),
                0xF3 => set_u3_r8(state, 6, Registers.Byte.E),
                0xF4 => set_u3_r8(state, 6, Registers.Byte.H),
                0xF5 => set_u3_r8(state, 6, Registers.Byte.L),
                0xF6 => set_u3_vhl(state, 6),
                0xF7 => set_u3_r8(state, 6, Registers.Byte.A),
                0xF8 => set_u3_r8(state, 7, Registers.Byte.B),
                0xF9 => set_u3_r8(state, 7, Registers.Byte.C),
                0xFA => set_u3_r8(state, 7, Registers.Byte.D),
                0xFB => set_u3_r8(state, 7, Registers.Byte.E),
                0xFC => set_u3_r8(state, 7, Registers.Byte.H),
                0xFD => set_u3_r8(state, 7, Registers.Byte.L),
                0xFE => set_u3_vhl(state, 7),
                0xFF => set_u3_r8(state, 7, Registers.Byte.A),
            };
        }, // TODO: prefix
        0xCC => call_cc_n16(state, &fetch16(state), ConditionCode.Z),
        0xCD => call_n16(state, &fetch16(state)),
        0xCE => adc_a_n8(state, &fetch(state)),
        0xCF => rst_vec(state, 0x08),
        0xD0 => ret_cc(state, ConditionCode.NC),
        0xD1 => pop_r16(state, Registers.Word.DE),
        0xD2 => jp_cc_n16(state, &fetch16(state), ConditionCode.NC),
        0xD3 => 0,
        0xD4 => call_cc_n16(state, &fetch16(state), ConditionCode.NC),
        0xD5 => push_r16(state, Registers.Word.DE),
        0xD6 => sub_a_n8(state, &fetch(state)),
        0xD7 => rst_vec(state, 0x10),
        0xD8 => ret_cc(state, ConditionCode.C),
        0xD9 => reti(state),
        0xDA => jp_cc_n16(state, &fetch16(state), ConditionCode.C),
        0xDB => 0,
        0xDC => call_cc_n16(state, &fetch16(state), ConditionCode.C),
        0xDD => 0,
        0xDE => sbc_a_n8(state, &fetch(state)),
        0xDF => rst_vec(state, 0x18),
        0xE0 => ldh_n8_a(state, &fetch(state)),
        0xE1 => pop_r16(state, Registers.Word.HL),
        0xE2 => ldh_vc_a(state),
        0xE3 => 0,
        0xE4 => 0,
        0xE5 => push_r16(state, Registers.Word.HL),
        0xE6 => and_a_n8(state, &fetch(state)),
        0xE7 => rst_vec(state, 0x20),
        0xE8 => add_sp_e8(state, &@as(i8, @bitCast(fetch(state)))),
        0xE9 => jp_hl(state),
        0xEA => ld_n16_a(state, &fetch16(state)),
        0xEB => 0,
        0xEC => 0,
        0xED => 0,
        0xEE => xor_a_n8(state, &fetch(state)),
        0xEF => rst_vec(state, 0x28),
        0xF0 => ldh_a_n8(state, &fetch(state)),
        0xF1 => pop_af(state),
        0xF2 => ldh_a_vc(state),
        0xF3 => di(state),
        0xF4 => 0,
        0xF5 => push_af(state),
        0xF6 => or_a_n8(state, &fetch(state)),
        0xF7 => rst_vec(state, 0x30),
        0xF8 => ld_hl_sp_plus_e8(state, @as(i8, @bitCast(fetch(state)))),
        0xF9 => ld_sp_hl(state),
        0xFA => ld_a_vn16(state, fetch16(state)),
        0xFB => ei(state),
        0xFC => 0,
        0xFD => 0,
        0xFE => cp_a_n8(state, &fetch(state)),
        0xFF => rst_vec(state, 0x38),
    };
}

pub fn main() !void {
    var gameBoyState: GameBoyState = GameBoyState.init(null);

    // Load bootrom to memory
    const data = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, "src/dmg_boot.bin", 256);
    defer std.heap.page_allocator.free(data);

    for (data, 0..) |byte, index| {
        gameBoyState.mmu.write(@as(u16, @intCast(index)), byte);
    }

    {
        errdefer |err| if (err == error.SdlError) std.log.err("SDL error: {s}", .{c.SDL_GetError()});

        const window, const renderer = try graphics.createWindow();
        defer graphics.destroyWindow(window, renderer);

        const framebufferTexture = c.SDL_CreateTexture(@ptrCast(renderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 160, 144);

        mainLoop: while (true) {
            var ev: c.SDL_Event = undefined;
            while (c.SDL_PollEvent(&ev)) {
                if (ev.type == c.SDL_EVENT_QUIT) {
                    break :mainLoop;
                }
            }

            gameBoyState.step();
            _ = c.SDL_UpdateTexture(framebufferTexture, null, &gameBoyState.ppu.framebuffer, 160 * @sizeOf(u32));
            std.log.info("{any}", .{gameBoyState.registers});

            _ = c.SDL_RenderClear(@ptrCast(renderer));
            _ = c.SDL_RenderTexture(@ptrCast(renderer), framebufferTexture, null, null);
            _ = c.SDL_RenderPresent(@ptrCast(renderer));
        }
    }
}
