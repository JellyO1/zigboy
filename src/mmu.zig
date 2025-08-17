const std = @import("std");
const ppuz = @import("ppu.zig");
const mbcz = @import("mbc.zig");

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

pub const Joypad = struct {
    A: bool = false,
    B: bool = false,
    Select: bool = false,
    Start: bool = false,
    Right: bool = false,
    Left: bool = false,
    Up: bool = false,
    Down: bool = false,
    EnableDpad: bool = false,
    EnableButtons: bool = false,

    inline fn toU8(b: bool) u8 {
        return @as(u8, @as(u1, @bitCast(b)));
    }

    pub fn read(self: *Joypad) u8 {
        if (!self.EnableButtons and !self.EnableDpad) return 0xFF;

        var ret: u8 = 0x00;
        if (self.EnableButtons) {
            ret = @as(u8, (toU8(self.A) << 0 | toU8(self.B) << 1 | toU8(self.Select) << 2 | toU8(self.Start) << 3 | toU8(self.EnableDpad) << 4 | toU8(self.EnableButtons) << 5));
        } else if (self.EnableDpad) {
            ret = @as(u8, (toU8(self.Right) << 0 | toU8(self.Left) << 1 | toU8(self.Up) << 2 | toU8(self.Down) << 3 | toU8(self.EnableDpad) << 4 | toU8(self.EnableButtons) << 5));
        }

        return ~ret;
    }

    pub fn write(self: *Joypad, value: u8) void {
        self.EnableDpad = (value & 0x10) == 0;
        self.EnableButtons = (value & 0x20) == 0;
    }
};

pub const Tile = struct {
    data: [8][8]u2,
};

/// Memory Management Unit
pub const MMU = struct {
    pub const INT_VBLANK_ADDR = 0x40;
    pub const INT_LCD_ADDR = 0x48;
    pub const INT_TIMER_ADDR = 0x50;
    pub const INT_SERIAL_ADDR = 0x58;
    pub const INT_JOYPAD_ADDR = 0x60;

    pub const JOYP_ADDR = 0xFF00;
    pub const SB_ADDR = 0xFF01;
    pub const SC_ADDR = 0xFF02;
    pub const DIV_ADDR = 0xFF04;
    pub const TIMA_ADDR = 0xFF05;
    pub const TMA_ADDR = 0xFF06;
    pub const TAC_ADDR = 0xFF07;
    pub const IF_ADDR = 0xFF0F;
    pub const LCDC_ADDR = 0xFF40;
    pub const STAT_ADDR = 0xFF41;
    pub const BG_SCROLL_Y_ADDR = 0xFF42;
    pub const BG_SCROLL_X_ADDR = 0xFF43;
    pub const LY_ADDR = 0xFF44;
    pub const LYC_ADDR = 0xFF45;
    pub const DMA_ADDR = 0xFF46;
    pub const BG_PALETTE_ADDR = 0xFF47;
    pub const OBJ_0_PALETTE_ADDR = 0xFF48;
    pub const OBJ_1_PALETTE_ADDR = 0xFF49;
    pub const OBJ_WIN_Y_ADDR = 0xFF4A;
    pub const OBJ_WIN_X_ADDR = 0xFF4B;
    pub const BOOT_ROM_ENABLE_ADDR = 0xFF50;
    pub const IE_ADDR = 0xFFFF;

    pub const OAM_START_ADDR = 0xFE00;

    // Boot ROM (256 bytes)
    boot_rom: [0x100]u8,
    // Boot ROM enabled
    boot_rom_enabled: bool,

    // Video RAM (8KB)
    vram: [0x2000]u8,
    // External RAM (8KB)
    external_ram: [0x2000]u8,
    // Work RAM (8KB)
    work_ram: [0x2000]u8,
    // Echo RAM (mirror of work RAM)
    echo_ram: [0x2000]u8,
    // Object Attribute Memory (OAM) sprite RAM
    oam: [0xA0]u8,
    // oam: [0x100]u8,
    // Not usable
    // unused: [0x60]u8,
    // IO registers
    joypad: Joypad,
    io_registers: [0x7F]u8,
    // High RAM
    hram: [0x7F]u8,
    // Interrupt Enable register
    ie_register: InterruptFlags,
    // Tile cache
    tileset: [384]Tile,
    needs_dma: bool,

    // Memory Bank Controller
    mbc: *mbcz.MBC,

    pub fn init(boot_rom: ?[]u8, mbc: *mbcz.MBC) MMU {
        return .{
            .boot_rom = if (boot_rom) |br| br[0..0x100].* else std.mem.zeroes([0x100]u8),
            .boot_rom_enabled = if (boot_rom == null) false else true,
            .vram = std.mem.zeroes([0x2000]u8),
            .external_ram = std.mem.zeroes([0x2000]u8),
            .work_ram = std.mem.zeroes([0x2000]u8),
            .echo_ram = std.mem.zeroes([0x2000]u8),
            .oam = std.mem.zeroes([0xA0]u8),
            .joypad = Joypad{},
            .io_registers = std.mem.zeroes([0x7F]u8),
            .hram = std.mem.zeroes([0x7F]u8),
            .ie_register = InterruptFlags.init(null),
            .tileset = std.mem.zeroes([384]Tile),
            .needs_dma = false,
            .mbc = mbc,
        };
    }

    pub fn read(self: *MMU, addr: u16) u8 {
        return switch (addr) {
            0x0000...0x7FFF => {
                if (self.boot_rom_enabled and addr < 0x0100) {
                    return self.boot_rom[addr]; // Boot ROM
                }

                return self.mbc.read(addr);
            },
            0x8000...0x9FFF => self.vram[addr - 0x8000],
            0xA000...0xBFFF => self.mbc.read(addr),
            0xC000...0xDFFF => self.work_ram[addr - 0xC000],
            0xE000...0xFDFF => self.echo_ram[addr - 0xE000],
            0xFE00...0xFE9F => self.oam[addr - 0xFE00],
            0xFEA0...0xFEFF => 0x00, // Not Usable
            // 0xFE00...0xFEFF => self.oam[addr - 0xFE00],
            0xFF00 => self.joypad.read(),
            0xFF01...0xFF7F => self.io_registers[addr - 0xFF01],
            0xFF80...0xFFFE => self.hram[addr - 0xFF80],
            0xFFFF => @bitCast(self.ie_register),
            // else => 0xFF,
        };
    }

    pub fn readPtr(self: *MMU, addr: u16) *u8 {
        return switch (addr) {
            0x0000...0x7FFF => {
                if (self.boot_rom_enabled and addr < 0x0100) {
                    return &self.boot_rom[addr]; // Boot ROM
                }

                @panic("not implemented");
            },
            0x8000...0x9FFF => &self.vram[addr - 0x8000],
            0xA000...0xBFFF => @panic("not implemented"),
            0xC000...0xDFFF => &self.work_ram[addr - 0xC000],
            0xE000...0xFDFF => &self.echo_ram[addr - 0xE000],
            0xFE00...0xFE9F => &self.oam[addr - 0xFE00],
            0xFEA0...0xFEFF => @constCast(&@as(u8, 0x00)), // Not Usable
            // 0xFE00...0xFEFF => &self.oam[addr - 0xFE00],
            0xFF00 => @ptrCast(&self.joypad),
            0xFF01...0xFF7F => &self.io_registers[addr - 0xFF01],
            0xFF80...0xFFFE => &self.hram[addr - 0xFF80],
            0xFFFF => @ptrCast(&self.ie_register),
            // else => 0xFF,
        };
    }

    pub fn readObj(self: *MMU, index: u8) ppuz.Object {
        // There's 40 objects worth of memory
        std.debug.assert(index < 40);

        const start = index * @sizeOf(u32);
        // const end = start + @sizeOf(u32);

        // NOTE: I'd rather return a pointer to the OAM memory obj but can't find how.
        return .{
            .posY = self.oam[start],
            .posX = self.oam[start + 1],
            .tileIndex = self.oam[start + 2],
            .attributes = @bitCast(self.oam[start + 3]),
        };
    }

    pub fn read16(self: *MMU, addr: u16) u16 {
        const high: u16 = @as(u16, @intCast(self.read(addr + 1))) << 8;
        const low: u16 = @intCast(self.read(addr));
        return high | low;
    }

    pub fn write(self: *MMU, addr: u16, value: u8) void {
        switch (addr) {
            0x0000...0x7FFF => self.mbc.write(addr, value),
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
                    const color = (if (highBit == 0) @as(u2, 0b00) else @as(u2, 0b10)) | (if (lowBit == 0) @as(u2, 0b00) else @as(u2, 0b01));
                    // const color: u2 = @as(u2, @truncate(highBit)) << 1 | @as(u2, @truncate(lowBit));
                    self.tileset[tileIndex].data[tileY][tileX] = color;
                }
            },
            0xA000...0xBFFF => self.mbc.write(addr, value),
            0xC000...0xDFFF => {
                self.work_ram[addr - 0xC000] = value;
                self.echo_ram[addr - 0xC000] = value;
            },
            0xE000...0xFDFF => self.echo_ram[addr - 0xE000] = value,
            0xFE00...0xFE9F => self.oam[addr - 0xFE00] = value,
            0xFEA0...0xFEFF => {}, // Not Usable
            // 0xFE00...0xFEFF => self.oam[addr - 0xFE00] = value,
            0xFF00 => self.joypad.write(value),
            0xFF01...0xFF7F => {
                const start_addr = 0xFF01;
                const normalized_addr = addr - start_addr;

                // Writing to the DIV register resets it
                if (addr == DIV_ADDR) {
                    self.io_registers[normalized_addr] = 0;
                    return;
                }

                self.io_registers[normalized_addr] = value;

                if (addr == DMA_ADDR) {
                    self.needs_dma = true;
                }

                // Check if bit 7 (transfer enable) of SC (Serial Transfer Control) is set
                if (self.io_registers[SC_ADDR - start_addr] == 0x81) {
                    std.debug.print("{c}", .{self.io_registers[SB_ADDR - start_addr]});

                    // Request serial interrupt that signals we've handled it.
                    const IF: *InterruptFlags = @ptrCast(&self.io_registers[IF_ADDR - start_addr]);
                    IF.Serial = true;

                    // Signal that we've handled it by setting it back to disabled
                    // ~0x80 is every bit as 1 except the 7th, & this causes it to go
                    // low while keeping every other bit the same.
                    self.io_registers[SC_ADDR - start_addr] = self.io_registers[SC_ADDR - start_addr] & ~@as(u8, 0x80);
                }

                if (addr == BOOT_ROM_ENABLE_ADDR) {
                    self.boot_rom_enabled = value == 0x01; // 0x01 = Boot ROM enabled, 0x00 = Boot ROM disabled
                }
            },
            0xFF80...0xFFFE => self.hram[addr - 0xFF80] = value,
            0xFFFF => self.ie_register = @as(InterruptFlags, @bitCast(value)),
        }
    }
};
