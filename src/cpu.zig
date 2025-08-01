const std = @import("std");
const mmuz = @import("mmu.zig");

// Two types of cycles: T selfs and M cycles
// M cycles (“machine” cycles, 1:4 clock) are the base unit of CPU instructions
// T selfs or T cycles (“transistor”(?) selfs, 1:1 clock) are the base unit of system operation and many components are clocked directly on T self
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
        _ = flags;
        return .{
            .A = 0x01,
            .B = 0x00,
            .C = 0x13,
            .D = 0,
            .E = 0xD8,
            .H = 0x01,
            .L = 0x4D,
            .F = Flags.init(.{
                .Z = true,
            }),
            .PC = 0x100,
            .SP = 0xFFFE,
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

pub const CPU = struct {
    /// General Purpose registers
    registers: Registers,
    /// Memory Management Unit
    mmu: *mmuz.MMU,
    /// Interrupt Master Enable
    ime: bool,
    /// EI delay (1 instruction delay before turning on IME)
    ei_delay: bool,
    /// Halt
    halt: bool,

    pub fn init(registers: ?Registers, mmu: *mmuz.MMU, ime: ?bool, ei_delay: ?bool, halt: ?bool) CPU {
        return .{
            .registers = registers orelse Registers.init(),
            .mmu = mmu,
            .ime = ime orelse false,
            .ei_delay = ei_delay orelse false,
            .halt = halt orelse false,
        };
    }

    pub fn step(self: *CPU) u32 {
        var cycles: u32 = 0;
        if (self.halt) {
            cycles += self.checkInterrups();
            return cycles;
        }

        // If EI delay is set, set IME and clear EI delay
        if (self.ei_delay) {
            self.ei_delay = false;
            self.ime = true;
        }

        const op = self.fetch();
        cycles += self.execute(op);

        cycles += self.checkInterrups();

        return cycles;
    }

    fn fetch(self: *CPU) u8 {
        const byte = self.mmu.read(self.registers.PC);
        self.registers.PC +%= 1;
        return byte;
    }

    fn fetch16(self: *CPU) u16 {
        const val = self.mmu.read16(self.registers.PC);
        self.registers.PC +%= 2;
        return val;
    }

    fn execute(self: *CPU, opcode: u8) u32 {
        return switch (opcode) {
            0x00 => noop(),
            0x01 => ld_r16_n16(self, Registers.Word.BC, &fetch16(self)),
            0x02 => ld_vr16_a(self, Registers.Word.BC),
            0x03 => inc_r16(self, Registers.Word.BC),
            0x04 => inc_r8(self, Registers.Byte.B),
            0x05 => dec_r8(self, Registers.Byte.B),
            0x06 => ld_r8_n8(self, Registers.Byte.B, &fetch(self)),
            0x07 => rlca(self),
            0x08 => ld_vn16_sp(self, &fetch16(self)),
            0x09 => add_hl_r16(self, Registers.Word.BC),
            0x0A => ld_a_vr16(self, Registers.Word.BC),
            0x0B => dec_r16(self, Registers.Word.BC),
            0x0C => inc_r8(self, Registers.Byte.C),
            0x0D => dec_r8(self, Registers.Byte.C),
            0x0E => ld_r8_n8(self, Registers.Byte.C, &fetch(self)),
            0x0F => rrca(self),
            0x10 => stop(self),
            0x11 => ld_r16_n16(self, Registers.Word.DE, &fetch16(self)),
            0x12 => ld_vr16_a(self, Registers.Word.DE),
            0x13 => inc_r16(self, Registers.Word.DE),
            0x14 => inc_r8(self, Registers.Byte.D),
            0x15 => dec_r8(self, Registers.Byte.D),
            0x16 => ld_r8_n8(self, Registers.Byte.D, &fetch(self)),
            0x17 => rla(self),
            0x18 => jr_e8(self, @as(i8, @bitCast(fetch(self)))),
            0x19 => add_hl_r16(self, Registers.Word.DE),
            0x1A => ld_a_vr16(self, Registers.Word.DE),
            0x1B => dec_r16(self, Registers.Word.DE),
            0x1C => inc_r8(self, Registers.Byte.E),
            0x1D => dec_r8(self, Registers.Byte.E),
            0x1E => ld_r8_n8(self, Registers.Byte.E, &fetch(self)),
            0x1F => rra(self),
            0x20 => jr_cc_e8(self, @as(i8, @bitCast(fetch(self))), ConditionCode.NZ),
            0x21 => ld_r16_n16(self, Registers.Word.HL, &fetch16(self)),
            0x22 => ld_hli_a(self),
            0x23 => inc_r16(self, Registers.Word.HL),
            0x24 => inc_r8(self, Registers.Byte.H),
            0x25 => dec_r8(self, Registers.Byte.H),
            0x26 => ld_r8_n8(self, Registers.Byte.H, &fetch(self)),
            0x27 => daa(self),
            0x28 => jr_cc_e8(self, @as(i8, @bitCast(fetch(self))), ConditionCode.Z),
            0x29 => add_hl_r16(self, Registers.Word.HL),
            0x2A => ld_a_hli(self),
            0x2B => dec_r16(self, Registers.Word.HL),
            0x2C => inc_r8(self, Registers.Byte.L),
            0x2D => dec_r8(self, Registers.Byte.L),
            0x2E => ld_r8_n8(self, Registers.Byte.L, &fetch(self)),
            0x2F => cpl(self),
            0x30 => jr_cc_e8(self, @as(i8, @bitCast(self.fetch())), ConditionCode.NC),
            0x31 => ld_sp_n16(self, &fetch16(self)),
            0x32 => ld_hld_a(self),
            0x33 => inc_sp(self),
            0x34 => inc_vhl(self),
            0x35 => dec_vhl(self),
            0x36 => ld_vhl_n8(self, &fetch(self)),
            0x37 => scf(self),
            0x38 => jr_cc_e8(self, @as(i8, @bitCast(fetch(self))), ConditionCode.C),
            0x39 => add_hl_sp(self),
            0x3A => ld_a_hld(self),
            0x3B => dec_sp(self),
            0x3C => inc_r8(self, Registers.Byte.A),
            0x3D => dec_r8(self, Registers.Byte.A),
            0x3E => ld_r8_n8(self, Registers.Byte.A, &fetch(self)),
            0x3F => ccf(self),
            0x40 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.B),
            0x41 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.C),
            0x42 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.D),
            0x43 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.E),
            0x44 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.H),
            0x45 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.L),
            0x46 => ld_r8_vhl(self, Registers.Byte.B),
            0x47 => ld_r8_r8(self, Registers.Byte.B, Registers.Byte.A),
            0x48 => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.B),
            0x49 => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.C),
            0x4A => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.D),
            0x4B => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.E),
            0x4C => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.H),
            0x4D => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.L),
            0x4E => ld_r8_vhl(self, Registers.Byte.C),
            0x4F => ld_r8_r8(self, Registers.Byte.C, Registers.Byte.A),
            0x50 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.B),
            0x51 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.C),
            0x52 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.D),
            0x53 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.E),
            0x54 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.H),
            0x55 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.L),
            0x56 => ld_r8_vhl(self, Registers.Byte.D),
            0x57 => ld_r8_r8(self, Registers.Byte.D, Registers.Byte.A),
            0x58 => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.B),
            0x59 => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.C),
            0x5A => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.D),
            0x5B => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.E),
            0x5C => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.H),
            0x5D => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.L),
            0x5E => ld_r8_vhl(self, Registers.Byte.E),
            0x5F => ld_r8_r8(self, Registers.Byte.E, Registers.Byte.A),
            0x60 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.B),
            0x61 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.C),
            0x62 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.D),
            0x63 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.E),
            0x64 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.H),
            0x65 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.L),
            0x66 => ld_r8_vhl(self, Registers.Byte.H),
            0x67 => ld_r8_r8(self, Registers.Byte.H, Registers.Byte.A),
            0x68 => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.B),
            0x69 => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.C),
            0x6A => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.D),
            0x6B => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.E),
            0x6C => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.H),
            0x6D => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.L),
            0x6E => ld_r8_vhl(self, Registers.Byte.L),
            0x6F => ld_r8_r8(self, Registers.Byte.L, Registers.Byte.A),
            0x70 => ld_vhl_r8(self, Registers.Byte.B),
            0x71 => ld_vhl_r8(self, Registers.Byte.C),
            0x72 => ld_vhl_r8(self, Registers.Byte.D),
            0x73 => ld_vhl_r8(self, Registers.Byte.E),
            0x74 => ld_vhl_r8(self, Registers.Byte.H),
            0x75 => ld_vhl_r8(self, Registers.Byte.L),
            0x76 => halt_op(self),
            0x77 => ld_vhl_r8(self, Registers.Byte.A),
            0x78 => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.B),
            0x79 => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.C),
            0x7A => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.D),
            0x7B => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.E),
            0x7C => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.H),
            0x7D => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.L),
            0x7E => ld_r8_vhl(self, Registers.Byte.A),
            0x7F => ld_r8_r8(self, Registers.Byte.A, Registers.Byte.A),
            0x80 => add_a_r8(self, Registers.Byte.B),
            0x81 => add_a_r8(self, Registers.Byte.C),
            0x82 => add_a_r8(self, Registers.Byte.D),
            0x83 => add_a_r8(self, Registers.Byte.E),
            0x84 => add_a_r8(self, Registers.Byte.H),
            0x85 => add_a_r8(self, Registers.Byte.L),
            0x86 => add_a_vhl(self),
            0x87 => add_a_r8(self, Registers.Byte.A),
            0x88 => adc_a_r8(self, Registers.Byte.B),
            0x89 => adc_a_r8(self, Registers.Byte.C),
            0x8A => adc_a_r8(self, Registers.Byte.D),
            0x8B => adc_a_r8(self, Registers.Byte.E),
            0x8C => adc_a_r8(self, Registers.Byte.H),
            0x8D => adc_a_r8(self, Registers.Byte.L),
            0x8E => adc_a_vhl(self),
            0x8F => adc_a_r8(self, Registers.Byte.A),
            0x90 => sub_a_r8(self, Registers.Byte.B),
            0x91 => sub_a_r8(self, Registers.Byte.C),
            0x92 => sub_a_r8(self, Registers.Byte.D),
            0x93 => sub_a_r8(self, Registers.Byte.E),
            0x94 => sub_a_r8(self, Registers.Byte.H),
            0x95 => sub_a_r8(self, Registers.Byte.L),
            0x96 => sub_a_vhl(self),
            0x97 => sub_a_r8(self, Registers.Byte.A),
            0x98 => sbc_a_r8(self, Registers.Byte.B),
            0x99 => sbc_a_r8(self, Registers.Byte.C),
            0x9A => sbc_a_r8(self, Registers.Byte.D),
            0x9B => sbc_a_r8(self, Registers.Byte.E),
            0x9C => sbc_a_r8(self, Registers.Byte.H),
            0x9D => sbc_a_r8(self, Registers.Byte.L),
            0x9E => sbc_a_vhl(self),
            0x9F => sbc_a_r8(self, Registers.Byte.A),
            0xA0 => and_a_r8(self, Registers.Byte.B),
            0xA1 => and_a_r8(self, Registers.Byte.C),
            0xA2 => and_a_r8(self, Registers.Byte.D),
            0xA3 => and_a_r8(self, Registers.Byte.E),
            0xA4 => and_a_r8(self, Registers.Byte.H),
            0xA5 => and_a_r8(self, Registers.Byte.L),
            0xA6 => and_a_vhl(self),
            0xA7 => and_a_r8(self, Registers.Byte.A),
            0xA8 => xor_a_r8(self, Registers.Byte.B),
            0xA9 => xor_a_r8(self, Registers.Byte.C),
            0xAA => xor_a_r8(self, Registers.Byte.D),
            0xAB => xor_a_r8(self, Registers.Byte.E),
            0xAC => xor_a_r8(self, Registers.Byte.H),
            0xAD => xor_a_r8(self, Registers.Byte.L),
            0xAE => xor_a_vhl(self),
            0xAF => xor_a_r8(self, Registers.Byte.A),
            0xB0 => or_a_r8(self, Registers.Byte.B),
            0xB1 => or_a_r8(self, Registers.Byte.C),
            0xB2 => or_a_r8(self, Registers.Byte.D),
            0xB3 => or_a_r8(self, Registers.Byte.E),
            0xB4 => or_a_r8(self, Registers.Byte.H),
            0xB5 => or_a_r8(self, Registers.Byte.L),
            0xB6 => or_a_vhl(self),
            0xB7 => or_a_r8(self, Registers.Byte.A),
            0xB8 => cp_a_r8(self, Registers.Byte.B),
            0xB9 => cp_a_r8(self, Registers.Byte.C),
            0xBA => cp_a_r8(self, Registers.Byte.D),
            0xBB => cp_a_r8(self, Registers.Byte.E),
            0xBC => cp_a_r8(self, Registers.Byte.H),
            0xBD => cp_a_r8(self, Registers.Byte.L),
            0xBE => cp_a_vhl(self),
            0xBF => cp_a_r8(self, Registers.Byte.A),
            0xC0 => ret_cc(self, ConditionCode.NZ),
            0xC1 => pop_r16(self, Registers.Word.BC),
            0xC2 => jp_cc_n16(self, &fetch16(self), ConditionCode.NZ),
            0xC3 => jp_n16(self, &fetch16(self)),
            0xC4 => call_cc_n16(self, &fetch16(self), ConditionCode.NZ),
            0xC5 => push_r16(self, Registers.Word.BC),
            0xC6 => add_a_n8(self, &fetch(self)),
            0xC7 => rst_vec(self, 0x00),
            0xC8 => ret_cc(self, ConditionCode.Z),
            0xC9 => ret(self),
            0xCA => jp_cc_n16(self, &fetch16(self), ConditionCode.Z),
            0xCB => {
                const cbOpcode = fetch(self);
                return switch (cbOpcode) {
                    0x00 => rlc_r8(self, Registers.Byte.B),
                    0x01 => rlc_r8(self, Registers.Byte.C),
                    0x02 => rlc_r8(self, Registers.Byte.D),
                    0x03 => rlc_r8(self, Registers.Byte.E),
                    0x04 => rlc_r8(self, Registers.Byte.H),
                    0x05 => rlc_r8(self, Registers.Byte.L),
                    0x06 => rlc_vhl(self),
                    0x07 => rlc_r8(self, Registers.Byte.A),
                    0x08 => rrc_r8(self, Registers.Byte.B),
                    0x09 => rrc_r8(self, Registers.Byte.C),
                    0x0A => rrc_r8(self, Registers.Byte.D),
                    0x0B => rrc_r8(self, Registers.Byte.E),
                    0x0C => rrc_r8(self, Registers.Byte.H),
                    0x0D => rrc_r8(self, Registers.Byte.L),
                    0x0E => rrc_vhl(self),
                    0x0F => rrc_r8(self, Registers.Byte.A),
                    0x10 => rl_r8(self, Registers.Byte.B),
                    0x11 => rl_r8(self, Registers.Byte.C),
                    0x12 => rl_r8(self, Registers.Byte.D),
                    0x13 => rl_r8(self, Registers.Byte.E),
                    0x14 => rl_r8(self, Registers.Byte.H),
                    0x15 => rl_r8(self, Registers.Byte.L),
                    0x16 => rl_vhl(self),
                    0x17 => rl_r8(self, Registers.Byte.A),
                    0x18 => rr_r8(self, Registers.Byte.B),
                    0x19 => rr_r8(self, Registers.Byte.C),
                    0x1A => rr_r8(self, Registers.Byte.D),
                    0x1B => rr_r8(self, Registers.Byte.E),
                    0x1C => rr_r8(self, Registers.Byte.H),
                    0x1D => rr_r8(self, Registers.Byte.L),
                    0x1E => rr_vhl(self),
                    0x1F => rr_r8(self, Registers.Byte.A),
                    0x20 => sla_r8(self, Registers.Byte.B),
                    0x21 => sla_r8(self, Registers.Byte.C),
                    0x22 => sla_r8(self, Registers.Byte.D),
                    0x23 => sla_r8(self, Registers.Byte.E),
                    0x24 => sla_r8(self, Registers.Byte.H),
                    0x25 => sla_r8(self, Registers.Byte.L),
                    0x26 => sla_vhl(self),
                    0x27 => sla_r8(self, Registers.Byte.A),
                    0x28 => sra_r8(self, Registers.Byte.B),
                    0x29 => sra_r8(self, Registers.Byte.C),
                    0x2A => sra_r8(self, Registers.Byte.D),
                    0x2B => sra_r8(self, Registers.Byte.E),
                    0x2C => sra_r8(self, Registers.Byte.H),
                    0x2D => sra_r8(self, Registers.Byte.L),
                    0x2E => sra_vhl(self),
                    0x2F => sra_r8(self, Registers.Byte.A),
                    0x30 => swap_r8(self, Registers.Byte.B),
                    0x31 => swap_r8(self, Registers.Byte.C),
                    0x32 => swap_r8(self, Registers.Byte.D),
                    0x33 => swap_r8(self, Registers.Byte.E),
                    0x34 => swap_r8(self, Registers.Byte.H),
                    0x35 => swap_r8(self, Registers.Byte.L),
                    0x36 => swap_vhl(self),
                    0x37 => swap_r8(self, Registers.Byte.A),
                    0x38 => srl_r8(self, Registers.Byte.B),
                    0x39 => srl_r8(self, Registers.Byte.C),
                    0x3A => srl_r8(self, Registers.Byte.D),
                    0x3B => srl_r8(self, Registers.Byte.E),
                    0x3C => srl_r8(self, Registers.Byte.H),
                    0x3D => srl_r8(self, Registers.Byte.L),
                    0x3E => srl_vhl(self),
                    0x3F => srl_r8(self, Registers.Byte.A),
                    0x40 => bit_u3_r8(self, 0, Registers.Byte.B),
                    0x41 => bit_u3_r8(self, 0, Registers.Byte.C),
                    0x42 => bit_u3_r8(self, 0, Registers.Byte.D),
                    0x43 => bit_u3_r8(self, 0, Registers.Byte.E),
                    0x44 => bit_u3_r8(self, 0, Registers.Byte.H),
                    0x45 => bit_u3_r8(self, 0, Registers.Byte.L),
                    0x46 => bit_u3_vhl(self, 0),
                    0x47 => bit_u3_r8(self, 0, Registers.Byte.A),
                    0x48 => bit_u3_r8(self, 1, Registers.Byte.B),
                    0x49 => bit_u3_r8(self, 1, Registers.Byte.C),
                    0x4A => bit_u3_r8(self, 1, Registers.Byte.D),
                    0x4B => bit_u3_r8(self, 1, Registers.Byte.E),
                    0x4C => bit_u3_r8(self, 1, Registers.Byte.H),
                    0x4D => bit_u3_r8(self, 1, Registers.Byte.L),
                    0x4E => bit_u3_vhl(self, 1),
                    0x4F => bit_u3_r8(self, 1, Registers.Byte.A),
                    0x50 => bit_u3_r8(self, 2, Registers.Byte.B),
                    0x51 => bit_u3_r8(self, 2, Registers.Byte.C),
                    0x52 => bit_u3_r8(self, 2, Registers.Byte.D),
                    0x53 => bit_u3_r8(self, 2, Registers.Byte.E),
                    0x54 => bit_u3_r8(self, 2, Registers.Byte.H),
                    0x55 => bit_u3_r8(self, 2, Registers.Byte.L),
                    0x56 => bit_u3_vhl(self, 2),
                    0x57 => bit_u3_r8(self, 2, Registers.Byte.A),
                    0x58 => bit_u3_r8(self, 3, Registers.Byte.B),
                    0x59 => bit_u3_r8(self, 3, Registers.Byte.C),
                    0x5A => bit_u3_r8(self, 3, Registers.Byte.D),
                    0x5B => bit_u3_r8(self, 3, Registers.Byte.E),
                    0x5C => bit_u3_r8(self, 3, Registers.Byte.H),
                    0x5D => bit_u3_r8(self, 3, Registers.Byte.L),
                    0x5E => bit_u3_vhl(self, 3),
                    0x5F => bit_u3_r8(self, 3, Registers.Byte.A),
                    0x60 => bit_u3_r8(self, 4, Registers.Byte.B),
                    0x61 => bit_u3_r8(self, 4, Registers.Byte.C),
                    0x62 => bit_u3_r8(self, 4, Registers.Byte.D),
                    0x63 => bit_u3_r8(self, 4, Registers.Byte.E),
                    0x64 => bit_u3_r8(self, 4, Registers.Byte.H),
                    0x65 => bit_u3_r8(self, 4, Registers.Byte.L),
                    0x66 => bit_u3_vhl(self, 4),
                    0x67 => bit_u3_r8(self, 4, Registers.Byte.A),
                    0x68 => bit_u3_r8(self, 5, Registers.Byte.B),
                    0x69 => bit_u3_r8(self, 5, Registers.Byte.C),
                    0x6A => bit_u3_r8(self, 5, Registers.Byte.D),
                    0x6B => bit_u3_r8(self, 5, Registers.Byte.E),
                    0x6C => bit_u3_r8(self, 5, Registers.Byte.H),
                    0x6D => bit_u3_r8(self, 5, Registers.Byte.L),
                    0x6E => bit_u3_vhl(self, 5),
                    0x6F => bit_u3_r8(self, 5, Registers.Byte.A),
                    0x70 => bit_u3_r8(self, 6, Registers.Byte.B),
                    0x71 => bit_u3_r8(self, 6, Registers.Byte.C),
                    0x72 => bit_u3_r8(self, 6, Registers.Byte.D),
                    0x73 => bit_u3_r8(self, 6, Registers.Byte.E),
                    0x74 => bit_u3_r8(self, 6, Registers.Byte.H),
                    0x75 => bit_u3_r8(self, 6, Registers.Byte.L),
                    0x76 => bit_u3_vhl(self, 6),
                    0x77 => bit_u3_r8(self, 6, Registers.Byte.A),
                    0x78 => bit_u3_r8(self, 7, Registers.Byte.B),
                    0x79 => bit_u3_r8(self, 7, Registers.Byte.C),
                    0x7A => bit_u3_r8(self, 7, Registers.Byte.D),
                    0x7B => bit_u3_r8(self, 7, Registers.Byte.E),
                    0x7C => bit_u3_r8(self, 7, Registers.Byte.H),
                    0x7D => bit_u3_r8(self, 7, Registers.Byte.L),
                    0x7E => bit_u3_vhl(self, 7),
                    0x7F => bit_u3_r8(self, 7, Registers.Byte.A),
                    0x80 => res_u3_r8(self, 0, Registers.Byte.B),
                    0x81 => res_u3_r8(self, 0, Registers.Byte.C),
                    0x82 => res_u3_r8(self, 0, Registers.Byte.D),
                    0x83 => res_u3_r8(self, 0, Registers.Byte.E),
                    0x84 => res_u3_r8(self, 0, Registers.Byte.H),
                    0x85 => res_u3_r8(self, 0, Registers.Byte.L),
                    0x86 => res_u3_vhl(self, 0),
                    0x87 => res_u3_r8(self, 0, Registers.Byte.A),
                    0x88 => res_u3_r8(self, 1, Registers.Byte.B),
                    0x89 => res_u3_r8(self, 1, Registers.Byte.C),
                    0x8A => res_u3_r8(self, 1, Registers.Byte.D),
                    0x8B => res_u3_r8(self, 1, Registers.Byte.E),
                    0x8C => res_u3_r8(self, 1, Registers.Byte.H),
                    0x8D => res_u3_r8(self, 1, Registers.Byte.L),
                    0x8E => res_u3_vhl(self, 1),
                    0x8F => res_u3_r8(self, 1, Registers.Byte.A),
                    0x90 => res_u3_r8(self, 2, Registers.Byte.B),
                    0x91 => res_u3_r8(self, 2, Registers.Byte.C),
                    0x92 => res_u3_r8(self, 2, Registers.Byte.D),
                    0x93 => res_u3_r8(self, 2, Registers.Byte.E),
                    0x94 => res_u3_r8(self, 2, Registers.Byte.H),
                    0x95 => res_u3_r8(self, 2, Registers.Byte.L),
                    0x96 => res_u3_vhl(self, 2),
                    0x97 => res_u3_r8(self, 2, Registers.Byte.A),
                    0x98 => res_u3_r8(self, 3, Registers.Byte.B),
                    0x99 => res_u3_r8(self, 3, Registers.Byte.C),
                    0x9A => res_u3_r8(self, 3, Registers.Byte.D),
                    0x9B => res_u3_r8(self, 3, Registers.Byte.E),
                    0x9C => res_u3_r8(self, 3, Registers.Byte.H),
                    0x9D => res_u3_r8(self, 3, Registers.Byte.L),
                    0x9E => res_u3_vhl(self, 3),
                    0x9F => res_u3_r8(self, 3, Registers.Byte.A),
                    0xA0 => res_u3_r8(self, 4, Registers.Byte.B),
                    0xA1 => res_u3_r8(self, 4, Registers.Byte.C),
                    0xA2 => res_u3_r8(self, 4, Registers.Byte.D),
                    0xA3 => res_u3_r8(self, 4, Registers.Byte.E),
                    0xA4 => res_u3_r8(self, 4, Registers.Byte.H),
                    0xA5 => res_u3_r8(self, 4, Registers.Byte.L),
                    0xA6 => res_u3_vhl(self, 4),
                    0xA7 => res_u3_r8(self, 4, Registers.Byte.A),
                    0xA8 => res_u3_r8(self, 5, Registers.Byte.B),
                    0xA9 => res_u3_r8(self, 5, Registers.Byte.C),
                    0xAA => res_u3_r8(self, 5, Registers.Byte.D),
                    0xAB => res_u3_r8(self, 5, Registers.Byte.E),
                    0xAC => res_u3_r8(self, 5, Registers.Byte.H),
                    0xAD => res_u3_r8(self, 5, Registers.Byte.L),
                    0xAE => res_u3_vhl(self, 5),
                    0xAF => res_u3_r8(self, 5, Registers.Byte.A),
                    0xB0 => res_u3_r8(self, 6, Registers.Byte.B),
                    0xB1 => res_u3_r8(self, 6, Registers.Byte.C),
                    0xB2 => res_u3_r8(self, 6, Registers.Byte.D),
                    0xB3 => res_u3_r8(self, 6, Registers.Byte.E),
                    0xB4 => res_u3_r8(self, 6, Registers.Byte.H),
                    0xB5 => res_u3_r8(self, 6, Registers.Byte.L),
                    0xB6 => res_u3_vhl(self, 6),
                    0xB7 => res_u3_r8(self, 6, Registers.Byte.A),
                    0xB8 => res_u3_r8(self, 7, Registers.Byte.B),
                    0xB9 => res_u3_r8(self, 7, Registers.Byte.C),
                    0xBA => res_u3_r8(self, 7, Registers.Byte.D),
                    0xBB => res_u3_r8(self, 7, Registers.Byte.E),
                    0xBC => res_u3_r8(self, 7, Registers.Byte.H),
                    0xBD => res_u3_r8(self, 7, Registers.Byte.L),
                    0xBE => res_u3_vhl(self, 7),
                    0xBF => res_u3_r8(self, 7, Registers.Byte.A),
                    0xC0 => set_u3_r8(self, 0, Registers.Byte.B),
                    0xC1 => set_u3_r8(self, 0, Registers.Byte.C),
                    0xC2 => set_u3_r8(self, 0, Registers.Byte.D),
                    0xC3 => set_u3_r8(self, 0, Registers.Byte.E),
                    0xC4 => set_u3_r8(self, 0, Registers.Byte.H),
                    0xC5 => set_u3_r8(self, 0, Registers.Byte.L),
                    0xC6 => set_u3_vhl(self, 0),
                    0xC7 => set_u3_r8(self, 0, Registers.Byte.A),
                    0xC8 => set_u3_r8(self, 1, Registers.Byte.B),
                    0xC9 => set_u3_r8(self, 1, Registers.Byte.C),
                    0xCA => set_u3_r8(self, 1, Registers.Byte.D),
                    0xCB => set_u3_r8(self, 1, Registers.Byte.E),
                    0xCC => set_u3_r8(self, 1, Registers.Byte.H),
                    0xCD => set_u3_r8(self, 1, Registers.Byte.L),
                    0xCE => set_u3_vhl(self, 1),
                    0xCF => set_u3_r8(self, 1, Registers.Byte.A),
                    0xD0 => set_u3_r8(self, 2, Registers.Byte.B),
                    0xD1 => set_u3_r8(self, 2, Registers.Byte.C),
                    0xD2 => set_u3_r8(self, 2, Registers.Byte.D),
                    0xD3 => set_u3_r8(self, 2, Registers.Byte.E),
                    0xD4 => set_u3_r8(self, 2, Registers.Byte.H),
                    0xD5 => set_u3_r8(self, 2, Registers.Byte.L),
                    0xD6 => set_u3_vhl(self, 2),
                    0xD7 => set_u3_r8(self, 2, Registers.Byte.A),
                    0xD8 => set_u3_r8(self, 3, Registers.Byte.B),
                    0xD9 => set_u3_r8(self, 3, Registers.Byte.C),
                    0xDA => set_u3_r8(self, 3, Registers.Byte.D),
                    0xDB => set_u3_r8(self, 3, Registers.Byte.E),
                    0xDC => set_u3_r8(self, 3, Registers.Byte.H),
                    0xDD => set_u3_r8(self, 3, Registers.Byte.L),
                    0xDE => set_u3_vhl(self, 3),
                    0xDF => set_u3_r8(self, 3, Registers.Byte.A),
                    0xE0 => set_u3_r8(self, 4, Registers.Byte.B),
                    0xE1 => set_u3_r8(self, 4, Registers.Byte.C),
                    0xE2 => set_u3_r8(self, 4, Registers.Byte.D),
                    0xE3 => set_u3_r8(self, 4, Registers.Byte.E),
                    0xE4 => set_u3_r8(self, 4, Registers.Byte.H),
                    0xE5 => set_u3_r8(self, 4, Registers.Byte.L),
                    0xE6 => set_u3_vhl(self, 4),
                    0xE7 => set_u3_r8(self, 4, Registers.Byte.A),
                    0xE8 => set_u3_r8(self, 5, Registers.Byte.B),
                    0xE9 => set_u3_r8(self, 5, Registers.Byte.C),
                    0xEA => set_u3_r8(self, 5, Registers.Byte.D),
                    0xEB => set_u3_r8(self, 5, Registers.Byte.E),
                    0xEC => set_u3_r8(self, 5, Registers.Byte.H),
                    0xED => set_u3_r8(self, 5, Registers.Byte.L),
                    0xEE => set_u3_vhl(self, 5),
                    0xEF => set_u3_r8(self, 5, Registers.Byte.A),
                    0xF0 => set_u3_r8(self, 6, Registers.Byte.B),
                    0xF1 => set_u3_r8(self, 6, Registers.Byte.C),
                    0xF2 => set_u3_r8(self, 6, Registers.Byte.D),
                    0xF3 => set_u3_r8(self, 6, Registers.Byte.E),
                    0xF4 => set_u3_r8(self, 6, Registers.Byte.H),
                    0xF5 => set_u3_r8(self, 6, Registers.Byte.L),
                    0xF6 => set_u3_vhl(self, 6),
                    0xF7 => set_u3_r8(self, 6, Registers.Byte.A),
                    0xF8 => set_u3_r8(self, 7, Registers.Byte.B),
                    0xF9 => set_u3_r8(self, 7, Registers.Byte.C),
                    0xFA => set_u3_r8(self, 7, Registers.Byte.D),
                    0xFB => set_u3_r8(self, 7, Registers.Byte.E),
                    0xFC => set_u3_r8(self, 7, Registers.Byte.H),
                    0xFD => set_u3_r8(self, 7, Registers.Byte.L),
                    0xFE => set_u3_vhl(self, 7),
                    0xFF => set_u3_r8(self, 7, Registers.Byte.A),
                };
            },
            0xCC => call_cc_n16(self, &fetch16(self), ConditionCode.Z),
            0xCD => call_n16(self, &fetch16(self)),
            0xCE => adc_a_n8(self, &fetch(self)),
            0xCF => rst_vec(self, 0x08),
            0xD0 => ret_cc(self, ConditionCode.NC),
            0xD1 => pop_r16(self, Registers.Word.DE),
            0xD2 => jp_cc_n16(self, &fetch16(self), ConditionCode.NC),
            0xD3 => 0,
            0xD4 => call_cc_n16(self, &fetch16(self), ConditionCode.NC),
            0xD5 => push_r16(self, Registers.Word.DE),
            0xD6 => sub_a_n8(self, &fetch(self)),
            0xD7 => rst_vec(self, 0x10),
            0xD8 => ret_cc(self, ConditionCode.C),
            0xD9 => reti(self),
            0xDA => jp_cc_n16(self, &fetch16(self), ConditionCode.C),
            0xDB => 0,
            0xDC => call_cc_n16(self, &fetch16(self), ConditionCode.C),
            0xDD => 0,
            0xDE => sbc_a_n8(self, &fetch(self)),
            0xDF => rst_vec(self, 0x18),
            0xE0 => ldh_n8_a(self, &fetch(self)),
            0xE1 => pop_r16(self, Registers.Word.HL),
            0xE2 => ldh_vc_a(self),
            0xE3 => 0,
            0xE4 => 0,
            0xE5 => push_r16(self, Registers.Word.HL),
            0xE6 => and_a_n8(self, &fetch(self)),
            0xE7 => rst_vec(self, 0x20),
            0xE8 => add_sp_e8(self, &@as(i8, @bitCast(fetch(self)))),
            0xE9 => jp_hl(self),
            0xEA => ld_n16_a(self, &fetch16(self)),
            0xEB => 0,
            0xEC => 0,
            0xED => 0,
            0xEE => xor_a_n8(self, &fetch(self)),
            0xEF => rst_vec(self, 0x28),
            0xF0 => ldh_a_n8(self, &fetch(self)),
            0xF1 => pop_af(self),
            0xF2 => ldh_a_vc(self),
            0xF3 => di(self),
            0xF4 => 0,
            0xF5 => push_af(self),
            0xF6 => or_a_n8(self, &fetch(self)),
            0xF7 => rst_vec(self, 0x30),
            0xF8 => ld_hl_sp_plus_e8(self, @as(i8, @bitCast(fetch(self)))),
            0xF9 => ld_sp_hl(self),
            0xFA => ld_a_vn16(self, fetch16(self)),
            0xFB => ei(self),
            0xFC => 0,
            0xFD => 0,
            0xFE => cp_a_n8(self, &fetch(self)),
            0xFF => rst_vec(self, 0x38),
        };
    }

    fn checkInterrups(self: *CPU) u32 {
        var cycles: u32 = 1;
        const pending: u8 = self.mmu.read(0xFFFF) & self.mmu.read(0xFF0F) & 0x1F;
        if (pending != 0) {
            // Interrupt halt if there's an interrupt pending
            if (self.halt) {
                self.halt = false;
            }

            // Handle interrupts if IME is enabled
            if (self.ime) {
                const IF: *mmuz.InterruptFlags = @ptrCast(self.mmu.readPtr(0xFF0F));

                if (IF.VBlank and self.mmu.ie_register.VBlank) {
                    self.ime = false;
                    IF.VBlank = false;
                    cycles += self.handleInterrupt(0x40);
                } else if (IF.Timer and self.mmu.ie_register.Timer) {
                    self.ime = false;
                    IF.Timer = false;

                    cycles += self.handleInterrupt(0x50);
                } else if (IF.Serial and self.mmu.ie_register.Serial) {
                    self.ime = false;
                    IF.Serial = false;

                    cycles += self.handleInterrupt(0x58);
                } else if (IF.LCD and self.mmu.ie_register.LCD) {
                    self.ime = false;
                    IF.LCD = false;

                    cycles += self.handleInterrupt(0x48);
                } else if (IF.Joypad and self.mmu.ie_register.Joypad) {
                    self.ime = false;
                    IF.Joypad = false;

                    cycles += self.handleInterrupt(0x60);
                }
            }
        }

        return cycles;
    }

    fn handleInterrupt(self: *CPU, addr: u16) u32 {
        // Save the next instruction address
        const next_instr_addr = self.registers.PC;

        // Split 16-bit value into high and low 8-bit values
        const high: u8 = @intCast(next_instr_addr >> 8);
        const low: u8 = @intCast(next_instr_addr & 0xFF);

        // Decrement SP by 2 (16 bits)
        self.registers.SP -= 2;

        // Write high and low bytes to stack
        self.mmu.write(self.registers.SP + 1, high);
        self.mmu.write(self.registers.SP, low);

        self.registers.PC = addr;

        return 5;
    }

    /// No operation
    fn noop() u32 {
        return MCycle;
    }

    fn adc_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const vr8 = self.registers.getByte(r8);
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));
        const r1, const ov1 = @addWithOverflow(self.registers.A, vr8);
        const r2, const ov2 = @addWithOverflow(r1, carry);

        // Half carry when bit 3 overflows (including carry flag)
        self.registers.F.H = ((self.registers.A & 0x0F) + (vr8 & 0x0F) + carry) > 0x0F;

        self.registers.A = r2;

        self.registers.F.Z = r2 == 0;
        self.registers.F.N = false;
        self.registers.F.C = ov1 == 1 or ov2 == 1;

        return MCycle;
    }

    fn adc_a_vhl(self: *CPU) u32 {
        const vhl = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));
        const r1, const ov1 = @addWithOverflow(self.registers.A, vhl);
        const r2, const ov2 = @addWithOverflow(r1, carry);

        // Half carry when bit 3 overflows (including carry flag)
        self.registers.F.H = ((self.registers.A & 0x0F) + (vhl & 0x0F) + carry) > 0x0F;

        self.registers.A = r2;

        self.registers.F.Z = r2 == 0;
        self.registers.F.N = false;
        self.registers.F.C = ov1 == 1 or ov2 == 1;

        return MCycle * 2;
    }

    fn adc_a_n8(self: *CPU, n8: *const u8) u32 {
        const r1, const ov1 = @addWithOverflow(self.registers.A, n8.*);
        const r2, const ov2 = @addWithOverflow(r1, @as(u8, @as(u1, @bitCast(self.registers.F.C))));

        // Half carry when bit 3 overflows (including carry flag)
        self.registers.F.H = ((self.registers.A & 0x0F) + (n8.* & 0x0F) + @as(u8, @as(u1, @bitCast(self.registers.F.C)))) > 0x0F;

        self.registers.A = r2;

        self.registers.F.Z = r2 == 0;
        self.registers.F.N = false;
        self.registers.F.C = ov1 == 1 or ov2 == 1;

        return MCycle * 2;
    }

    fn add_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const vr8 = self.registers.getByte(r8);
        const result, const overflow = @addWithOverflow(self.registers.A, vr8);

        // Half carry when bit 3 overflows
        self.registers.F.H = ((self.registers.A & 0x0F) + (vr8 & 0x0F)) > 0x0F;

        self.registers.A = result;

        self.registers.F.Z = result == 0;
        self.registers.F.N = false;
        self.registers.F.C = overflow == 1;

        return MCycle;
    }

    fn add_a_vhl(self: *CPU) u32 {
        const vhl = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const result, const overflow = @addWithOverflow(self.registers.A, vhl);

        // Half carry when bit 3 overflows
        self.registers.F.H = ((self.registers.A & 0x0F) + (vhl & 0x0F)) > 0x0F;

        self.registers.A = result;

        self.registers.F.Z = result == 0;
        self.registers.F.N = false;
        self.registers.F.C = overflow == 1;

        return MCycle * 2;
    }

    fn add_a_n8(self: *CPU, n8: *const u8) u32 {
        const result, const overflow = @addWithOverflow(self.registers.A, n8.*);

        // Half carry when bit 3 overflows
        self.registers.F.H = ((self.registers.A & 0x0F) + (n8.* & 0x0F)) > 0x0F;

        self.registers.A = result;

        self.registers.F.Z = result == 0;
        self.registers.F.N = false;
        self.registers.F.C = overflow == 1;

        return MCycle * 2;
    }

    fn add_hl_r16(self: *CPU, r16: Registers.Word) u32 {
        const hl = self.registers.getWord(Registers.Word.HL);
        const vr16 = self.registers.getWord(r16);
        const result, const overflow = @addWithOverflow(hl, vr16);
        self.registers.setWord(Registers.Word.HL, result);

        self.registers.F.N = false;
        self.registers.F.H = ((hl & 0x7FF) + (vr16 & 0x7FF)) > 0x7FF;
        self.registers.F.C = overflow == 1;

        return MCycle * 2;
    }

    fn add_hl_sp(self: *CPU) u32 {
        const hl = self.registers.getWord(Registers.Word.HL);
        const result, const overflow = @addWithOverflow(hl, self.registers.SP);
        self.registers.setWord(Registers.Word.HL, result);

        self.registers.F.N = false;
        self.registers.F.H = ((hl & 0x7FF) + (self.registers.SP & 0x7FF)) > 0x7FF;
        self.registers.F.C = overflow == 1;

        return MCycle * 2;
    }

    fn add_sp_e8(self: *CPU, e8: *const i8) u32 {
        const result, _ = @addWithOverflow(@as(i16, @bitCast(self.registers.SP)), e8.*);

        // bit 3 overflow
        self.registers.F.H = (self.registers.SP & 0x0F) + (@as(u8, @bitCast(e8.*)) & 0x0F) > 0x0F;
        // bit 7 overflow
        self.registers.F.C = (self.registers.SP & 0xFF) + (@as(u16, @intCast(@as(u8, @bitCast(e8.*)))) & 0xFF) > 0xFF;

        self.registers.SP = @bitCast(result);

        self.registers.F.Z = false;
        self.registers.F.N = false;

        return MCycle * 4;
    }

    fn and_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        self.registers.A = self.registers.A & val;
        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = true;
        self.registers.F.C = false;
        return MCycle;
    }

    fn and_a_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.registers.A = self.registers.A & val;
        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = true;
        self.registers.F.C = false;
        return MCycle * 2;
    }

    fn and_a_n8(self: *CPU, n8: *const u8) u32 {
        const val = n8.*;
        self.registers.A = self.registers.A & val;
        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = true;
        self.registers.F.C = false;
        return MCycle * 2;
    }

    fn bit_u3_r8(self: *CPU, comptime vu3: u3, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        self.registers.F.Z = (val & (1 << vu3)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = true;
        return MCycle * 2;
    }

    fn bit_u3_vhl(self: *CPU, comptime vu3: u3) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.registers.F.Z = (val & (1 << vu3)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = true;
        return MCycle * 3;
    }

    fn call_n16(self: *CPU, n16: *const u16) u32 {
        // Save the next instruction address
        const next_instr_addr = self.registers.PC;

        // Split 16-bit value into high and low 8-bit values
        const high: u8 = @intCast(next_instr_addr >> 8);
        const low: u8 = @intCast(next_instr_addr & 0xFF);

        // Decrement SP by 2 (16 bits)
        self.registers.SP -= 2;

        // Write high and low bytes to stack
        self.mmu.write(self.registers.SP + 1, high);
        self.mmu.write(self.registers.SP, low);

        // jump to the new address
        self.registers.PC = n16.*;

        return MCycle * 6;
    }

    fn call_cc_n16(self: *CPU, n16: *const u16, cc: ConditionCode) u32 {
        switch (cc) {
            .Z => {
                if (self.registers.F.Z) {
                    return call_n16(self, n16);
                }
            },
            .C => {
                if (self.registers.F.C) {
                    return call_n16(self, n16);
                }
            },
            .NZ => {
                if (!self.registers.F.Z) {
                    return call_n16(self, n16);
                }
            },
            .NC => {
                if (!self.registers.F.C) {
                    return call_n16(self, n16);
                }
            },
        }

        return MCycle * 3;
    }

    fn ccf(self: *CPU) u32 {
        self.registers.F.C = !self.registers.F.C;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle;
    }

    fn cp_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const result = self.registers.A -% val;

        self.registers.F.Z = result == 0;
        self.registers.F.N = true;
        self.registers.F.H = (self.registers.A & 0xF) < (val & 0xF);
        self.registers.F.C = self.registers.A < val;
        return MCycle;
    }

    fn cp_a_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const result = self.registers.A -% val;

        self.registers.F.Z = result == 0;
        self.registers.F.N = true;
        self.registers.F.H = (self.registers.A & 0xF) < (val & 0xF);
        self.registers.F.C = self.registers.A < val;
        return MCycle * 2;
    }

    fn cp_a_n8(self: *CPU, n8: *const u8) u32 {
        const val = n8.*;
        const result = self.registers.A -% val;

        self.registers.F.Z = result == 0;
        self.registers.F.N = true;
        self.registers.F.H = (self.registers.A & 0xF) < (val & 0xF);
        self.registers.F.C = self.registers.A < val;
        return MCycle * 2;
    }

    fn cpl(self: *CPU) u32 {
        self.registers.A = ~self.registers.A;
        self.registers.F.N = true;
        self.registers.F.H = true;
        return MCycle;
    }

    fn daa(self: *CPU) u32 {
        var adjustment: u8 = 0;
        if (self.registers.F.N) {
            if (self.registers.F.C) {
                adjustment += 0x60;
            }
            if (self.registers.F.H) {
                adjustment += 0x6;
            }

            self.registers.A -%= adjustment;
        } else {
            if (self.registers.F.C or self.registers.A > 0x99) {
                adjustment += 0x60;
                self.registers.F.C = true;
            }
            if (self.registers.F.H or (self.registers.A & 0xF) > 0x9) {
                adjustment += 0x6;
            }

            self.registers.A +%= adjustment;
        }

        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.H = false;
        return MCycle;
    }

    fn dec_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        self.registers.setByte(r8, val -% 1);
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = true;
        self.registers.F.H = (val & 0xF) < 1;
        return MCycle;
    }

    fn dec_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.mmu.write(self.registers.getWord(Registers.Word.HL), val -% 1);
        self.registers.F.Z = (val -% 1) == 0;
        self.registers.F.N = true;
        self.registers.F.H = (val & 0xF) < 1;
        return MCycle * 3;
    }

    fn dec_r16(self: *CPU, r16: Registers.Word) u32 {
        const val = self.registers.getWord(r16);
        self.registers.setWord(r16, val -% 1);
        return MCycle * 2;
    }

    fn dec_sp(self: *CPU) u32 {
        self.registers.SP -%= 1;
        return MCycle * 2;
    }

    fn di(self: *CPU) u32 {
        self.ei_delay = false;
        self.ime = false;
        return MCycle;
    }

    fn ei(self: *CPU) u32 {
        self.ei_delay = true;
        return MCycle;
    }

    /// Enter CPU low-power consumption mode until an interrupt occurs.
    ///
    /// The exact behavior of this instruction depends on the self of the IME flag, and whether interrupts are pending (i.e. whether ‘[IE] & [IF]’ is non-zero):
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
    fn halt_op(self: *CPU) u32 {
        self.halt = true;
        return 0;
    }

    fn inc_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        self.registers.setByte(r8, val +% 1);
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = ((val & 0xF) + 1) > 0xF;
        return MCycle;
    }

    fn inc_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.mmu.write(self.registers.getWord(Registers.Word.HL), val +% 1);
        self.registers.F.Z = (val +% 1) == 0;
        self.registers.F.N = false;
        self.registers.F.H = ((val & 0xF) + 1) > 0xF;
        return MCycle * 3;
    }

    fn inc_r16(self: *CPU, r16: Registers.Word) u32 {
        const val = self.registers.getWord(r16);
        self.registers.setWord(r16, val +% 1);
        return MCycle * 2;
    }

    fn inc_sp(self: *CPU) u32 {
        self.registers.SP +%= 1;
        return MCycle * 2;
    }

    fn jp_n16(self: *CPU, n16: *const u16) u32 {
        self.registers.PC = n16.*;
        return MCycle * 4;
    }

    fn jp_cc_n16(self: *CPU, n16: *const u16, cc: ConditionCode) u32 {
        switch (cc) {
            .Z => {
                if (self.registers.F.Z) {
                    return jp_n16(self, n16);
                }
            },
            .C => {
                if (self.registers.F.C) {
                    return jp_n16(self, n16);
                }
            },
            .NZ => {
                if (!self.registers.F.Z) {
                    return jp_n16(self, n16);
                }
            },
            .NC => {
                if (!self.registers.F.C) {
                    return jp_n16(self, n16);
                }
            },
        }
        return MCycle * 3;
    }

    fn jp_hl(self: *CPU) u32 {
        self.registers.PC = self.registers.getWord(Registers.Word.HL);
        return MCycle;
    }

    fn jr_e8(self: *CPU, e8: i8) u32 {
        self.registers.PC +%= @as(u16, @bitCast(@as(i16, e8)));
        return MCycle * 3;
    }

    fn jr_cc_e8(self: *CPU, e8: i8, cc: ConditionCode) u32 {
        switch (cc) {
            .Z => {
                if (self.registers.F.Z) {
                    return jr_e8(self, e8);
                }
            },
            .C => {
                if (self.registers.F.C) {
                    return jr_e8(self, e8);
                }
            },
            .NZ => {
                if (!self.registers.F.Z) {
                    return jr_e8(self, e8);
                }
            },
            .NC => {
                if (!self.registers.F.C) {
                    return jr_e8(self, e8);
                }
            },
        }
        return MCycle * 2;
    }

    /// Copy the value of the register in the right to the register in the left
    fn ld_r8_r8(self: *CPU, r8_left: Registers.Byte, r8_right: Registers.Byte) u32 {
        //Storing a register into itself is a no-op; however, some Game Boy emulators interpret LD B,B as a breakpoint, or LD D,D as a debug message (such as BGB).
        if (r8_left == r8_right) {
            if (r8_left == Registers.Byte.B) {
                // breakpoint
            }

            return noop();
        }

        self.registers.setByte(r8_left, self.registers.getByte(r8_right));

        // LD r8,r8 takes 4 cycles (1 M-cycle)
        return MCycle;
    }

    /// Copy the value of the immediate byte to the register
    fn ld_r8_n8(self: *CPU, r8: Registers.Byte, n8: *const u8) u32 {
        self.registers.setByte(r8, n8.*);
        return MCycle * 2;
    }

    /// Copy the value of the immediate word to the register
    fn ld_r16_n16(self: *CPU, r16: Registers.Word, n16: *const u16) u32 {
        self.registers.setWord(r16, n16.*);
        return MCycle * 3;
    }

    /// Copy the value of the register to the memory location specified by the registers H and L
    fn ld_vhl_r8(self: *CPU, r8: Registers.Byte) u32 {
        self.mmu.write(self.registers.getWord(Registers.Word.HL), self.registers.getByte(r8));
        return MCycle * 2;
    }

    /// Copy the value of the immediate byte to the memory location specified by the registers H and L
    fn ld_vhl_n8(self: *CPU, n8: *const u8) u32 {
        self.mmu.write(self.registers.getWord(Registers.Word.HL), n8.*);
        return MCycle * 3;
    }

    /// Copy the value of the memory location specified by the registers H and L to the register
    fn ld_r8_vhl(self: *CPU, r8: Registers.Byte) u32 {
        self.registers.setByte(r8, self.mmu.read(self.registers.getWord(Registers.Word.HL)));
        return MCycle * 2;
    }

    /// Copy the value in the register A to the byte pointed to by r16
    fn ld_vr16_a(self: *CPU, r16: Registers.Word) u32 {
        self.mmu.write(self.registers.getWord(r16), self.registers.A);
        return MCycle * 2;
    }

    /// Copy the value in the register A to the byte pointed to by n16
    fn ld_n16_a(self: *CPU, n16: *const u16) u32 {
        self.mmu.write(n16.*, self.registers.A);

        return MCycle * 4;
    }

    /// Copy the value in the register A to the byte pointed to by n16
    ///
    /// n16 must be in the range 0xFF00-0xFFFF
    fn ldh_n16_a(self: *CPU, n16: u16) u32 {
        if (n16 >= 0xFF00 and n16 <= 0xFFFF) {
            self.mmu.write(n16, self.registers.A);
        }

        return MCycle * 3;
    }

    /// Copy the value in the register A to the byte pointed to by 0xFF00 + n8
    fn ldh_n8_a(self: *CPU, n8: *const u8) u32 {
        return self.ldh_n16_a(0xFF00 + @as(u16, @intCast(n8.*)));

        // self.mmu.write(0xFF00 + @as(u16, @intCast(n8.*)), self.registers.A);
        // return MCycle * 3;
    }

    /// Copy the value in the register A to the byte pointed to by 0xFF00 + C
    fn ldh_vc_a(self: *CPU) u32 {
        self.mmu.write(0xFF00 + @as(u16, @intCast(self.registers.getByte(Registers.Byte.C))), self.registers.A);
        return MCycle * 2;
    }

    /// Copy the value in the byte pointed to by r16 to the register A
    fn ld_a_vr16(self: *CPU, r16: Registers.Word) u32 {
        self.registers.A = self.mmu.read(self.registers.getWord(r16));
        return MCycle * 2;
    }

    /// Copy the value in the byte pointed to by n16 to the register A
    fn ld_a_vn16(self: *CPU, n16: u16) u32 {
        self.registers.A = self.mmu.read(n16);
        return MCycle * 4;
    }

    /// Copy the value in the byte pointed to by n16 to the register A
    ///
    /// n16 must be in the range 0xFF00-0xFFFF
    fn ldh_a_vn16(self: *CPU, n16: u16) u32 {
        if (n16 >= 0xFF00 and n16 <= 0xFFFF) {
            self.registers.A = self.mmu.read(n16);
        }
        return MCycle * 3;
    }

    /// Copy the value in the byte pointed to by 0xFF00 + n8 to the register A
    fn ldh_a_n8(self: *CPU, n8: *const u8) u32 {
        self.registers.A = self.mmu.read(0xFF00 + @as(u16, @intCast(n8.*)));
        return MCycle * 3;
    }

    /// Copy the value in the byte pointed to by 0xFF00 + C to the register A
    fn ldh_a_vc(self: *CPU) u32 {
        self.registers.A = self.mmu.read(0xFF00 + @as(u16, @intCast(self.registers.getByte(Registers.Byte.C))));
        return MCycle * 2;
    }

    /// Copy the value in the register A to the byte pointed to by HL and increment HL
    fn ld_hli_a(self: *CPU) u32 {
        self.mmu.write(self.registers.getWord(Registers.Word.HL), self.registers.A);
        self.registers.setWord(Registers.Word.HL, self.registers.getWord(Registers.Word.HL) + 1);
        return MCycle * 2;
    }

    /// Copy the value in the register A to the byte pointed to by HL and decrement HL
    fn ld_hld_a(self: *CPU) u32 {
        self.mmu.write(self.registers.getWord(Registers.Word.HL), self.registers.A);
        self.registers.setWord(Registers.Word.HL, self.registers.getWord(Registers.Word.HL) - 1);
        return MCycle * 2;
    }

    /// Copy the value in the byte pointed to by HL to the register A and decrement HL
    fn ld_a_hld(self: *CPU) u32 {
        self.registers.A = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.registers.setWord(Registers.Word.HL, self.registers.getWord(Registers.Word.HL) - 1);
        return MCycle * 2;
    }

    /// Copy the value in the byte pointed to by HL to the register A and increment HL
    fn ld_a_hli(self: *CPU) u32 {
        self.registers.A = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.registers.setWord(Registers.Word.HL, self.registers.getWord(Registers.Word.HL) + 1);
        return MCycle * 2;
    }

    /// Copy the value n16 into the register SP
    fn ld_sp_n16(self: *CPU, n16: *const u16) u32 {
        self.registers.SP = n16.*;
        return MCycle * 3;
    }

    /// Copy SP & 0xFF to address n16 and SP >> 8 to address n16 + 1
    fn ld_vn16_sp(self: *CPU, n16: *const u16) u32 {
        self.mmu.write(n16.*, @intCast(self.registers.SP & 0xFF));
        self.mmu.write(n16.* + 1, @intCast(self.registers.SP >> 8));
        return MCycle * 5;
    }

    /// Add the signed value e8 to SP and copy the result into HL
    fn ld_hl_sp_plus_e8(self: *CPU, e8: i8) u32 {
        // TODO: all of this might not be necessary.

        const sp = self.registers.SP;
        const offset = @as(u16, @bitCast(@as(i16, e8)));

        // Wrapping addition (+%) when it doesn't fit wraps around
        const result, _ = @addWithOverflow(sp, offset);

        // Reset zero flag
        self.registers.F.Z = false;
        // Reset subtract flag
        self.registers.F.N = false;

        // Half carry occurs if bit 3 carries into bit 4
        self.registers.F.H = ((sp & 0x0F) + (offset & 0x0F)) > 0x0F;
        // Carry occurs if bit 7 carries into bit 8
        self.registers.F.C = ((sp & 0xFF) + (offset & 0xFF)) > 0xFF;

        // store result in HL
        self.registers.setWord(Registers.Word.HL, result);

        return MCycle * 3;
    }

    /// Copy the value in the register HL into the register SP
    fn ld_sp_hl(self: *CPU) u32 {
        self.registers.SP = self.registers.getWord(Registers.Word.HL);
        return MCycle * 2;
    }

    /// Set the value of register A to the bitwise OR between A and the value in register r8
    fn or_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        self.registers.A = self.registers.A | self.registers.getByte(r8);

        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle;
    }

    /// Set the value of register A to the bitwise OR between A and the value in memory at the address in register HL
    fn or_a_vhl(self: *CPU) u32 {
        self.registers.A = self.registers.A | self.mmu.read(self.registers.getWord(Registers.Word.HL));

        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 2;
    }

    /// Set the value of register A to the bitwise OR between A and the value in n8
    fn or_a_n8(self: *CPU, n8: *const u8) u32 {
        self.registers.A = self.registers.A | n8.*;

        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 2;
    }

    /// Pop register AF from the stack
    fn pop_af(self: *CPU) u32 {
        // Sets F to the value in the stack but keeps the lower nibble grounded to 0
        self.registers.F = @as(Flags, @bitCast(self.mmu.read(self.registers.SP) & 0xF0));
        self.registers.A = self.mmu.read(self.registers.SP + 1);
        self.registers.SP += 2;

        return MCycle * 3;
    }

    /// Pop register r16 from the stack
    fn pop_r16(self: *CPU, r16: Registers.Word) u32 {
        switch (r16) {
            Registers.Word.BC => {
                self.registers.B = self.mmu.read(self.registers.SP + 1);
                self.registers.C = self.mmu.read(self.registers.SP);
                self.registers.SP += 2;
            },
            Registers.Word.DE => {
                self.registers.D = self.mmu.read(self.registers.SP + 1);
                self.registers.E = self.mmu.read(self.registers.SP);
                self.registers.SP += 2;
            },
            Registers.Word.HL => {
                self.registers.H = self.mmu.read(self.registers.SP + 1);
                self.registers.L = self.mmu.read(self.registers.SP);
                self.registers.SP += 2;
            },
            else => {},
        }

        return MCycle * 3;
    }

    /// Push register AF to the stack
    fn push_af(self: *CPU) u32 {
        self.registers.SP -= 2;
        self.mmu.write(self.registers.SP + 1, self.registers.A);
        self.mmu.write(self.registers.SP, @as(u8, @bitCast(self.registers.F)));

        return MCycle * 4;
    }

    /// Push register r16 to the stack
    fn push_r16(self: *CPU, r16: Registers.Word) u32 {
        self.registers.SP -= 2;
        switch (r16) {
            Registers.Word.BC => {
                self.mmu.write(self.registers.SP + 1, self.registers.B);
                self.mmu.write(self.registers.SP, self.registers.C);
            },
            Registers.Word.DE => {
                self.mmu.write(self.registers.SP + 1, self.registers.D);
                self.mmu.write(self.registers.SP, self.registers.E);
            },
            Registers.Word.HL => {
                self.mmu.write(self.registers.SP + 1, self.registers.H);
                self.mmu.write(self.registers.SP, self.registers.L);
            },
            else => {},
        }

        return MCycle * 4;
    }

    /// Reset bit vu3 of register r8
    fn res_u3_r8(self: *CPU, vu3: u3, r8: Registers.Byte) u32 {
        // Create a mask with 1 at the position vu3 by left-shifting 1 by vu3 (e.g. 0b00000001 << 6 -> 0b01000000)
        const mask = @as(u8, 1) << vu3;

        // Applies a bitwise not to the mask (e.g. ~0b01000000 -> 0b10111111)
        const invertedMask = ~mask;

        // Applies a bitwise AND between the register value and the inverted mask (e.g. 0b11111111 & 0b10111111 -> 0b10111111)
        self.registers.setByte(r8, self.registers.getByte(r8) & invertedMask);
        return MCycle * 2;
    }

    /// Reset bit vu3 of memory at HL
    fn res_u3_vhl(self: *CPU, vu3: u3) u32 {
        // Create a mask with 1 at the position vu3 by left-shifting 1 by vu3 (e.g. 0b00000001 << 6 -> 0b01000000)
        const mask = @as(u8, 1) << vu3;

        // Applies a bitwise not to the mask (e.g. ~0b01000000 -> 0b10111111)
        const invertedMask = ~mask;

        // Applies a bitwise AND between the register value and the inverted mask (e.g. 0b11111111 & 0b10111111 -> 0b10111111)
        self.mmu.write(self.registers.getWord(Registers.Word.HL), self.mmu.read(self.registers.getWord(Registers.Word.HL)) & invertedMask);
        return MCycle * 4;
    }

    /// Returns from a function call
    fn ret(self: *CPU) u32 {
        self.registers.PC = @as(u16, self.mmu.read(self.registers.SP));
        self.registers.PC |= @as(u16, self.mmu.read(self.registers.SP + 1)) << 8;
        self.registers.SP += 2;
        return MCycle * 4;
    }

    /// Returns from a function call if condition is met
    fn ret_cc(self: *CPU, cc: ConditionCode) u32 {
        switch (cc) {
            ConditionCode.Z => {
                if (self.registers.F.Z) {
                    return ret(self) + MCycle;
                }
            },
            ConditionCode.C => {
                if (self.registers.F.C) {
                    return ret(self) + MCycle;
                }
            },
            ConditionCode.NZ => {
                if (!self.registers.F.Z) {
                    return ret(self) + MCycle;
                }
            },
            ConditionCode.NC => {
                if (!self.registers.F.C) {
                    return ret(self) + MCycle;
                }
            },
        }

        return MCycle * 2;
    }

    /// Returns from a function call and enables interrupts
    fn reti(self: *CPU) u32 {
        const cycles = ret(self);
        self.ime = true;
        return cycles;
    }

    /// Rotates the value of register r8 left, through the carry flag
    fn rl_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));
        // Shift left and set least significant bit to carry flag
        self.registers.setByte(r8, (val << 1) | (carry));

        // Set carry flag to the most significant bit of the original register value
        self.registers.F.C = (val >> 7) == 1;

        // Set zero flag if register value is 0
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 2;
    }

    /// Rotates the value of memory location specified by HL left, through the carry flag
    fn rl_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));
        // Shift left and set least significant bit to carry flag
        self.mmu.write(self.registers.getWord(Registers.Word.HL), (val << 1) | (carry));

        // Set carry flag to the most significant bit of the original register value
        self.registers.F.C = (val >> 7) == 1;

        // Set zero flag if register value is 0
        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 4;
    }

    /// Rotates the value of register A left, through the carry flag
    fn rla(self: *CPU) u32 {
        const val = self.registers.A;
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));

        // Shift left and set least significant bit to carry flag (aka rotate left)
        self.registers.A = (val << 1) | (carry);

        // Set carry flag to the most significant bit of the original register value
        self.registers.F.C = (val >> 7) == 1;

        self.registers.F.Z = false;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle;
    }

    /// Rotates the value of register r8 left and sets the carry flag
    fn rlc_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);

        // Rotate the register left
        self.registers.setByte(r8, (val << 1) | (val >> 7));

        // Set carry flag to the most significant bit of the original register value
        self.registers.F.C = (val >> 7) == 1;

        // Set zero flag if register value is 0
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 2;
    }

    /// Rotates the value of memory location specified by HL left and sets the carry flag
    fn rlc_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));

        // Rotate the memory location left
        self.mmu.write(self.registers.getWord(Registers.Word.HL), (val << 1) | (val >> 7));

        // Set carry flag to the most significant bit of the original register value
        self.registers.F.C = (val >> 7) == 1;

        // Set zero flag if register value is 0
        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 4;
    }

    /// Rotates the value of register A left and sets the carry flag
    fn rlca(self: *CPU) u32 {
        const val = self.registers.A;

        // Rotate the register left
        self.registers.A = (val << 1) | (val >> 7);

        // Set carry flag to the most significant bit of the original register value
        self.registers.F.C = (val >> 7) == 1;

        self.registers.F.Z = false;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle;
    }

    /// Rotates the value of register r8 right through the carry flag
    fn rr_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C))) << 7;

        // Rotate the register right
        self.registers.setByte(r8, (val >> 1) | carry);

        // Set carry flag to the least significant bit of the original register value
        self.registers.F.C = (val & 1) == 1;

        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 2;
    }

    /// Rotates the value of memory location specified by HL right through the carry flag
    fn rr_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C))) << 7;

        // Write the rotated value to memory
        self.mmu.write(self.registers.getWord(Registers.Word.HL), (val >> 1) | carry);

        // Set carry flag to the least significant bit of the original register value
        self.registers.F.C = (val & 1) == 1;

        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 4;
    }

    /// Rotates the value of register A right through the carry flag
    fn rra(self: *CPU) u32 {
        const val = self.registers.A;
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C))) << 7;

        // Rotate the register right
        self.registers.A = (val >> 1) | carry;

        // Set carry flag to the least significant bit of the original register value
        self.registers.F.C = (val & 1) == 1;

        self.registers.F.Z = false;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle;
    }

    /// Rotates the value of register r8 right and sets the carry flag
    fn rrc_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);

        // Rotate the register right
        self.registers.setByte(r8, (val >> 1) | (val << 7));

        // Set carry flag to the result
        self.registers.F.C = (val & 1) == 1;

        // Set if result is zero
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 2;
    }

    /// Rotates the value of memory location specified by HL right and sets the carry flag
    fn rrc_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));

        // Write the rotated value to memory
        self.mmu.write(self.registers.getWord(Registers.Word.HL), (val >> 1) | (val << 7));

        // Set carry flag to the least significant bit of the original register value
        self.registers.F.C = (val & 1) == 1;

        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle * 4;
    }

    /// Rotates the value of register A right and sets the carry flag
    fn rrca(self: *CPU) u32 {
        const val = self.registers.A;

        // Rotate the register right
        self.registers.A = (val >> 1) | (val << 7);

        // Set carry flag to the least significant bit of the original register value
        self.registers.F.C = (val & 1) == 1;

        self.registers.F.Z = false;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle;
    }

    /// Jump to the specified vector
    fn rst_vec(self: *CPU, vec: u16) u32 {
        switch (vec) {
            0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38 => {
                // Save the next instruction address
                const next_instr_addr = self.registers.PC;

                // Split 16-bit value into high and low 8-bit values
                const high: u8 = @intCast(next_instr_addr >> 8);
                const low: u8 = @intCast(next_instr_addr & 0xFF);

                // Decrement SP by 2 (16 bits)
                self.registers.SP -= 2;

                // Write high and low bytes to stack
                self.mmu.write(self.registers.SP + 1, high);
                self.mmu.write(self.registers.SP, low);

                // jump to the new address
                self.registers.PC = vec;
            },
            else => {},
        }

        return MCycle * 4;
    }

    /// Subtract the value in r8 from register A
    fn sbc_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));

        const r1, const ov1 = @subWithOverflow(self.registers.A, val);
        const r2, const ov2 = @subWithOverflow(r1, carry);

        self.registers.F.H = (self.registers.A & 0xF) < ((val & 0xF) + carry);

        self.registers.A = r2;

        // Set if borrow (i.e. if (r8 + carry) > A)
        self.registers.F.C = ov1 != 0 or ov2 != 0;
        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = true;

        return MCycle;
    }

    /// Subtract the value in memory at the address in register HL from register A
    fn sbc_a_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));

        const r1, const ov1 = @subWithOverflow(self.registers.A, val);
        const r2, const ov2 = @subWithOverflow(r1, carry);

        self.registers.F.H = (self.registers.A & 0xF) < ((val & 0xF) + carry);
        self.registers.A = r2;

        // Set if borrow (i.e. if (r8 + carry) > A)
        self.registers.F.C = ov1 != 0 or ov2 != 0;
        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = true;

        return MCycle * 2;
    }

    /// Subtract the value in n8 from register A
    fn sbc_a_n8(self: *CPU, n8: *const u8) u32 {
        const carry = @as(u8, @as(u1, @bitCast(self.registers.F.C)));

        const r1, const ov1 = @subWithOverflow(self.registers.A, n8.*);
        const r2, const ov2 = @subWithOverflow(r1, carry);

        self.registers.F.H = (self.registers.A & 0xF) < ((n8.* & 0xF) + carry);

        self.registers.A = r2;

        // Set if borrow (i.e. if (r8 + carry) > A)
        self.registers.F.C = ov1 == 1 or ov2 == 1;
        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = true;

        return MCycle * 2;
    }

    /// Set the carry flag
    fn scf(self: *CPU) u32 {
        self.registers.F.C = true;
        self.registers.F.N = false;
        self.registers.F.H = false;

        return MCycle;
    }

    /// Set the bit vu3 in register r8 to 1.
    fn set_u3_r8(self: *CPU, comptime vu3: u3, r8: Registers.Byte) u32 {
        self.registers.setByte(r8, self.registers.getByte(r8) | (1 << vu3));
        return MCycle * 2;
    }

    fn set_u3_vhl(self: *CPU, comptime vu3: u3) u32 {
        self.mmu.write(self.registers.getWord(Registers.Word.HL), self.mmu.read(self.registers.getWord(Registers.Word.HL)) | (1 << vu3));
        return MCycle * 4;
    }

    fn sla_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const carry = val & 0x80 != 0;
        self.registers.setByte(r8, val << 1);
        self.registers.F.C = carry;
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle * 2;
    }

    fn sla_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = val & 0x80 != 0;
        self.mmu.write(self.registers.getWord(Registers.Word.HL), val << 1);
        self.registers.F.C = carry;
        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle * 4;
    }

    fn sra_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const carry = val & 0x1 != 0;
        // Shift right but keep bit 7 unchanged
        self.registers.setByte(r8, (val >> 1) | (val & 0x80));
        self.registers.F.C = carry;
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle * 2;
    }

    fn sra_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = val & 0x1 != 0;
        // Shift right but keep bit 7 unchanged
        self.mmu.write(self.registers.getWord(Registers.Word.HL), (val >> 1) | (val & 0x80));
        self.registers.F.C = carry;
        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle * 4;
    }

    fn srl_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const carry = val & 0x1 != 0;
        self.registers.setByte(r8, val >> 1);
        self.registers.F.C = carry;
        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle * 2;
    }

    fn srl_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const carry = val & 0x1 != 0;
        self.mmu.write(self.registers.getWord(Registers.Word.HL), val >> 1);
        self.registers.F.C = carry;
        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        return MCycle * 4;
    }

    /// TODO: Implement stop (https://gbdev.io/pandocs/Reducing_Power_Consumption.html#using-the-stop-instruction)
    fn stop(self: *CPU) u32 {
        _ = self;

        return noop();
    }

    fn sub_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        const result, const overflow = @subWithOverflow(self.registers.A, val);
        // Set if half-borrow
        self.registers.F.H = (self.registers.A & 0xF) < (val & 0xF);
        self.registers.setByte(Registers.Byte.A, result);
        self.registers.F.C = overflow == 1;
        self.registers.F.Z = result == 0;
        self.registers.F.N = true;
        return MCycle;
    }

    fn sub_a_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        const result, const overflow = @subWithOverflow(self.registers.A, val);
        // Set if half-borrow
        self.registers.F.H = (self.registers.A & 0xF) < (val & 0xF);
        self.registers.setByte(Registers.Byte.A, result);
        self.registers.F.C = overflow == 1;
        self.registers.F.Z = result == 0;
        self.registers.F.N = true;
        return MCycle * 2;
    }

    fn sub_a_n8(self: *CPU, val: *const u8) u32 {
        const result, const overflow = @subWithOverflow(self.registers.A, val.*);
        // Set if half-borrow
        self.registers.F.H = (self.registers.A & 0xF) < (val.* & 0xF);
        self.registers.setByte(Registers.Byte.A, result);
        self.registers.F.C = overflow == 1;
        self.registers.F.Z = result == 0;
        self.registers.F.N = true;
        return MCycle * 2;
    }

    fn swap_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        self.registers.setByte(r8, (val >> 4) | (val << 4));

        self.registers.F.Z = self.registers.getByte(r8) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 2;
    }

    fn swap_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.mmu.write(self.registers.getWord(Registers.Word.HL), (val >> 4) | (val << 4));

        self.registers.F.Z = self.mmu.read(self.registers.getWord(Registers.Word.HL)) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 4;
    }

    fn xor_a_r8(self: *CPU, r8: Registers.Byte) u32 {
        const val = self.registers.getByte(r8);
        self.registers.setByte(Registers.Byte.A, self.registers.A ^ val);

        self.registers.F.Z = self.registers.getByte(Registers.Byte.A) == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 1;
    }

    fn xor_a_vhl(self: *CPU) u32 {
        const val = self.mmu.read(self.registers.getWord(Registers.Word.HL));
        self.registers.A = self.registers.A ^ val;

        self.registers.F.Z = self.registers.A == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 2;
    }

    fn xor_a_n8(self: *CPU, val: *const u8) u32 {
        const result = self.registers.A ^ val.*;
        self.registers.setByte(Registers.Byte.A, result);

        self.registers.F.Z = result == 0;
        self.registers.F.N = false;
        self.registers.F.H = false;
        self.registers.F.C = false;

        return MCycle * 2;
    }
};
