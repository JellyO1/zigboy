const std = @import("std");

// Interrupt Flags
pub const InterruptFlags = packed struct(u8) {
    VBlank: bool = false,
    LCD: bool = false,
    Timer: bool = false,
    Serial: bool = false,
    Joypad: bool = false,
    _: u3 = 0,

    pub fn init(val: ?u8) InterruptFlags {
        if (val) |v| {
            return @as(InterruptFlags, @bitCast(v));
        }

        return .{
            .VBlank = false,
            .LCD = false,
            .Timer = false,
            .Serial = false,
            .Joypad = false,
            ._ = 0,
        };
    }
};

pub const Tile = struct {
    data: [8][8]u2,
};

/// Memory Management Unit
pub const MMU = struct {
    // Boot ROM (256 bytes)
    boot_rom: [0x100]u8,
    // Boot ROM enabled
    boot_rom_enabled: bool,
    // ROM bank 0 (16KB)
    rom_bank0: [0x4000]u8,
    // ROM bank 1-N (16KB), used for bank switching
    rom_bank_n: [0x4000]u8,
    // Video RAM (8KB)
    vram: [0x2000]u8,
    // External RAM (8KB)
    external_ram: [0x2000]u8,
    // Work RAM (8KB)
    work_ram: [0x2000]u8,
    // Echo RAM (mirror of work RAM)
    echo_ram: [0x2000]u8,
    // Object Attribute Memory (OAM) sprite RAM
    // oam: [0xA0]u8,
    oam: [0x100]u8,
    // Not usable
    // unused: [0x60]u8,
    // IO registers
    io_registers: [0x80]u8,
    // High RAM
    hram: [0x7F]u8,
    // Interrupt Enable register
    ie_register: InterruptFlags,
    // Tile cache
    tileset: [384]Tile,

    // Memory Bank Controller
    // mbc: ?MBC = null,

    pub fn init(boot_rom: ?[0x100]u8, game_rom: ?[]u8) MMU {
        const rom_bank0 = if (game_rom) |rom| rom[0x0000..0x4000].* else std.mem.zeroes([0x4000]u8);
        const rom_bank_n = if (game_rom) |rom| rom[0x4000..0x8000].* else std.mem.zeroes([0x4000]u8);

        return .{
            .boot_rom = boot_rom orelse std.mem.zeroes([0x100]u8),
            .boot_rom_enabled = if (boot_rom == null) false else true,
            .rom_bank0 = rom_bank0,
            .rom_bank_n = rom_bank_n,
            .vram = std.mem.zeroes([0x2000]u8),
            .external_ram = std.mem.zeroes([0x2000]u8),
            .work_ram = std.mem.zeroes([0x2000]u8),
            .echo_ram = std.mem.zeroes([0x2000]u8),
            // .oam = std.mem.zeroes([0xA0]u8),
            .oam = std.mem.zeroes([0x100]u8),
            .io_registers = std.mem.zeroes([0x80]u8),
            .hram = std.mem.zeroes([0x7F]u8),
            .ie_register = InterruptFlags.init(null),
            .tileset = std.mem.zeroes([384]Tile),
        };
    }

    pub fn read(self: *MMU, addr: u16) u8 {
        return switch (addr) {
            0x0000...0x3FFF => {
                if (self.boot_rom_enabled and addr < 0x0100) {
                    return self.boot_rom[addr]; // Boot ROM
                }

                // TODO: Handle with MBC
                return self.rom_bank0[addr]; // ROM Bank 0
            },
            0x4000...0x7FFF => self.rom_bank_n[addr - 0x4000],
            0x8000...0x9FFF => self.vram[addr - 0x8000],
            0xA000...0xBFFF => self.external_ram[addr - 0xA000],
            0xC000...0xDFFF => self.work_ram[addr - 0xC000],
            0xE000...0xFDFF => self.echo_ram[addr - 0xE000],
            // 0xFE00...0xFE9F => self.oam[addr - 0xFE00],
            // 0xFEA0...0xFEFF => 0x00, // Not Usable
            0xFE00...0xFEFF => self.oam[addr - 0xFE00],
            0xFF00...0xFF7F => self.io_registers[addr - 0xFF00],
            0xFF80...0xFFFE => self.hram[addr - 0xFF80],
            0xFFFF => @bitCast(self.ie_register),
            // else => 0xFF,
        };
    }

    pub fn readPtr(self: *MMU, addr: u16) *u8 {
        return switch (addr) {
            0x0000...0x3FFF => {
                if (self.boot_rom_enabled and addr < 0x0100) {
                    return &self.boot_rom[addr]; // Boot ROM
                }

                return &self.rom_bank0[addr]; // ROM Bank 0
            },
            0x4000...0x7FFF => &self.rom_bank_n[addr - 0x4000],
            0x8000...0x9FFF => &self.vram[addr - 0x8000],
            0xA000...0xBFFF => &self.external_ram[addr - 0xA000],
            0xC000...0xDFFF => &self.work_ram[addr - 0xC000],
            0xE000...0xFDFF => &self.echo_ram[addr - 0xE000],
            // 0xFE00...0xFE9F => &self.oam[addr - 0xFE00],
            // 0xFEA0...0xFEFF => 0x00, // Not Usable
            0xFE00...0xFEFF => &self.oam[addr - 0xFE00],
            0xFF00...0xFF7F => &self.io_registers[addr - 0xFF00],
            0xFF80...0xFFFE => &self.hram[addr - 0xFF80],
            0xFFFF => @constCast(@as(*const u8, &@bitCast(self.ie_register))),
            // else => 0xFF,
        };
    }

    pub fn read16(self: *MMU, addr: u16) u16 {
        const high: u16 = @as(u16, @intCast(self.read(addr + 1))) << 8;
        const low: u16 = @intCast(self.read(addr));
        return high | low;
    }

    pub fn write(self: *MMU, addr: u16, value: u8) void {
        switch (addr) {
            0x0000...0x3FFF => self.rom_bank0[addr] = value,
            0x4000...0x7FFF => self.rom_bank_n[addr - 0x4000] = value,
            0x8000...0x9FFF => {
                self.vram[addr - 0x8000] = value;

                // If the address is outside of the tileset range, return
                if (addr >= 0x9800) return;

                // Normalize addr 0..6144
                const normAddr = addr - 0x8000;

                // A tile is 8x8 with each row being 2 bytes, that is 16 bytes total. 0..383
                const tileIndex = normAddr / 16;
                // Every two bytes is a row. 0..7
                const tileY = (normAddr % 16) / 2;

                // We need both bytes of the row to decode the tile data
                // The first byte (low bits) is at the current address
                // The second byte (high bits) is at the next address
                if (normAddr % 2 == 0) {
                    // This is the first byte of the row, we need to wait for the second byte
                    return;
                }

                // Now we have both bytes, decode the row
                const highByte = value; // Current byte
                const lowByte = self.vram[normAddr - 1]; // Previous byte

                // Loop through every pixel in the row
                inline for (0..8) |tileX| {
                    const bitIndex: u8 = 1 << (7 - tileX);
                    const lowBit: u8 = lowByte & bitIndex;
                    const highBit: u8 = highByte & bitIndex;
                    const color = if (highBit == 0) @as(u2, 0b00) else @as(u2, 0b10) | if (lowBit == 0) @as(u2, 0b00) else @as(u2, 0b01);
                    // const color: u2 = @as(u2, @truncate(highBit)) << 1 | @as(u2, @truncate(lowBit));
                    self.tileset[tileIndex].data[tileY][tileX] = color;
                }
            },
            0xA000...0xBFFF => self.external_ram[addr - 0xA000] = value,
            0xC000...0xDFFF => {
                self.work_ram[addr - 0xC000] = value;
                self.echo_ram[addr - 0xC000] = value;
            },
            0xE000...0xFDFF => self.echo_ram[addr - 0xE000] = value,
            // 0xFE00...0xFE9F => self.oam[addr - 0xFE00] = value,
            // 0xFEA0...0xFEFF => {}, // Not Usable
            0xFE00...0xFEFF => self.oam[addr - 0xFE00] = value,
            0xFF00...0xFF7F => {
                // Writing to the DIV register resets it
                if (addr == 0xFF04) {
                    self.io_registers[addr - 0xFF00] = 0;
                    return;
                }

                self.io_registers[addr - 0xFF00] = value;

                // Check if bit 7 (transfer enable) of SC (Serial Transfer Control) is set
                if (self.io_registers[0xFF02 - 0xFF00] == 0x81) {
                    std.debug.print("{c}", .{self.io_registers[1]});

                    // Request serial interrupt that signals we've handled it.
                    const IF: *InterruptFlags = @ptrCast(&self.io_registers[0xFF0F - 0xFF00]);
                    IF.Serial = true;

                    // Signal that we've handled it by setting it back to disabled
                    // ~0x80 is every bit as 1 except the 7th, & this causes it to go
                    // low while keeping every other bit the same.
                    self.io_registers[2] = self.io_registers[0xFF02 - 0xFF00] & ~@as(u8, 0x80);
                }

                if (addr == 0xFF50) {
                    self.boot_rom_enabled = value == 0x01; // 0x01 = Boot ROM enabled, 0x00 = Boot ROM disabled
                }
            },
            0xFF80...0xFFFE => self.hram[addr - 0xFF80] = value,
            0xFFFF => self.ie_register = @as(InterruptFlags, @bitCast(value)),
            // else => {},
        }
    }
};
