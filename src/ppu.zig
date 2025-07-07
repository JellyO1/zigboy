const std = @import("std");
const GameBoyState = @import("main.zig").GameBoyState;
const MMU = @import("mmu.zig").MMU;

const LCDControl = packed struct(u8) {
    /// 0: Off, 1: On
    BgWinEnable: bool = false,
    /// 0: Off, 1: On
    ObjEnable: bool = false,
    /// 0: 8x8, 1: 8x16
    ObjSize: bool = false,
    /// 0: 0x9800–9BFF; 1: 0x9C00–9FFF
    BgTileMapArea: bool = false,
    /// 0: 0x8800–97FF; 1: 0x8000–8FFF
    BgWinTileDataArea: bool = false,
    /// 0: Off, 1: On
    WinEnable: bool = false,
    /// 0: 0x9800-9BFF, 1: 0x9C00-9FFF
    WinTileMapArea: bool = false,
    /// 0: Off, 1: On
    LcdPpuEnable: bool = false,
};

const LCDStatus = packed struct(u8) {
    /// 0: HBlank, 1: VBlank, 2: OAM, 3: Drawing pixels
    Mode: u2 = 0,
    /// 0: LY != LYC, 1: LY == LYC
    LYCEqualsLY: bool = false,
    /// If set, selects the Mode 0 (HBlank) condition for the STAT interrupt
    Mode0: bool = false,
    /// If set, selects the Mode 1 (VBlank) condition for the STAT interrupt
    Mode1: bool = false,
    /// If set, selects the Mode 2 (OAM) condition for the STAT interrupt
    Mode2: bool = false,
    /// If set, selects the LYC == LY condition for the STAT interrupt
    LYC: bool = false,
    _: u1 = 0,
};

pub const RGBA = packed struct {
    r: u8,
    g: u8,
    b: u8,
    a: u8,
};

const PaletteColors = [4]RGBA{
    .{ .r = 255, .g = 255, .b = 255, .a = 255 },
    .{ .r = 192, .g = 192, .b = 192, .a = 255 },
    .{ .r = 96, .g = 96, .b = 96, .a = 255 },
    .{ .r = 0, .g = 0, .b = 0, .a = 255 },
};

const Color = enum(u2) {
    White,
    LightGray,
    DarkGray,
    Black,
};

const Palette = packed struct(u8) {
    ID0: Color = .White,
    ID1: Color = .White,
    ID2: Color = .White,
    ID3: Color = .White,

    pub fn get(self: Palette, index: u2) Color {
        return switch (index) {
            0 => self.ID0,
            1 => self.ID1,
            2 => self.ID2,
            3 => self.ID3,
        };
    }
};

const Mode = enum(u2) {
    /// This mode depends on the amount of time the CPU spent on the Drawing mode
    /// with a minimum of 87 cycles and max of 204 cycles
    HBlank,
    VBlank,
    OAM,
    /// This mode can take between 172 and 289 cycles
    Drawing,
};

pub const PPU = struct {
    modeClock: u32,
    mode: Mode,
    mmu: *MMU,
    framebuffer: [160 * 144]RGBA,
    /// LCD Control register
    lcdc: *LCDControl,
    /// LCD Status register
    stat: *LCDStatus,
    /// Background Y scroll (SCX)
    bgScrollY: *u8,
    /// Background X scroll (SCY)
    bgScrollX: *u8,
    /// Current scanline (LY) (0-153)
    scanline: *u8,
    /// LY compare register
    lyc: *u8,
    /// Background palette
    bgp: *Palette,
    /// Object palette 0
    op0: *Palette,
    /// Object palette 1
    op1: *Palette,
    /// Window Y position
    winY: *u8,
    /// Window X position
    winX: *u8,

    pub fn init(mmu: *MMU) PPU {
        return .{
            .mmu = mmu,
            .modeClock = 0,
            .mode = .OAM,
            .framebuffer = std.mem.zeroes([160 * 144]RGBA),
            .lcdc = @ptrCast(mmu.readPtr(0xFF40)),
            .stat = @ptrCast(mmu.readPtr(0xFF41)),
            .bgScrollY = mmu.readPtr(0xFF42),
            .bgScrollX = mmu.readPtr(0xFF43),
            .scanline = mmu.readPtr(0xFF44),
            .lyc = mmu.readPtr(0xFF45),
            .bgp = @ptrCast(mmu.readPtr(0xFF47)),
            .op0 = @ptrCast(mmu.readPtr(0xFF48)),
            .op1 = @ptrCast(mmu.readPtr(0xFF49)),
            .winY = mmu.readPtr(0xFF4A),
            .winX = mmu.readPtr(0xFF4B),
        };
    }

    pub fn step(self: *PPU, cycles: u32) void {
        for (0..cycles) |_| {
            self.modeClock += 1;

            switch (self.mode) {
                .OAM => {
                    if (self.modeClock % 80 == 0) {
                        self.modeClock = 0;
                        self.mode = .Drawing;
                    }
                },
                .Drawing => {
                    if (self.modeClock % 172 == 0) {
                        self.modeClock = 0;
                        self.mode = .HBlank;

                        // TODO: Render scanline to framebuffer
                        self.renderScanline();
                    }
                },
                .HBlank => {
                    if (self.modeClock % 204 == 0) {
                        self.modeClock = 0;
                        self.scanline.* += 1;
                        self.mode = .OAM;

                        if (self.scanline.* == 144) {
                            self.mode = .VBlank;
                        }
                    }
                },
                .VBlank => {
                    if (self.modeClock % 456 == 0) {
                        self.modeClock = 0;
                        self.scanline.* += 1;

                        if (self.scanline.* == 153) {
                            self.mode = .OAM;
                            self.scanline.* = 0;
                        }
                    }
                },
            }
        }
    }

    pub fn renderScanline(self: *PPU) void {
        for (0..160) |x| {
            //const as_u8: u8 = @bitCast(self.lcdc.*);
            //std.debug.print("{b:0>8}\n", .{as_u8});
            //if (self.lcdc.BgWinEnable) {
            // const bgX = (x + @as(usize, @intCast(self.bgScrollX.*))) % 256;
            // const bgY = (self.scanline.* + @as(usize, @intCast(self.bgScrollY.*))) % 256;
            const bgX = x;
            const bgY = self.scanline.*;

            const tileX = bgX % 8;
            const tileY = bgY % 8;

            const tileMapAddr: u16 = if (self.lcdc.BgTileMapArea) 0x9C00 else 0x9800;
            const tileIndex = self.mmu.read(tileMapAddr + @as(u16, @intCast(tileY * 32 + tileY)));

            const tile = self.mmu.tileset[tileIndex];

            const color = tile.data[tileY][tileX];
            // const palette = self.bgp;
            // const colorValue = palette.get(color);

            const frameBufferIndex = x + @as(usize, @intCast((self.scanline.*))) * 160;
            if (frameBufferIndex < self.framebuffer.len) {
                self.framebuffer[frameBufferIndex] = PaletteColors[color];
            }
            //}

            if (self.lcdc.WinEnable) {}
        }
    }

    pub fn renderBackground(self: *PPU) void {
        for (0..20) |renderTileIndex| {
            const tileMapAddr: u16 = if (self.lcdc.BgTileMapArea) 0x9800 else 0x9C00;

            var tileIndex = self.mmu.read(tileMapAddr + @as(u16, @intCast(renderTileIndex)));

            if (!self.lcdc.BgWinTileDataArea) {
                tileIndex += 128;
            }

            const tile = self.mmu.tileset[tileIndex];
            for (0..8) |row| {
                for (0..8) |col| {
                    const color = tile.data[row][col];
                    const palette = self.bgp;
                    const colorValue = palette.get(color);
                    const x = renderTileIndex * 8 + col;
                    const y = row;
                    self.framebuffer[x + y * 160] = PaletteColors[@intFromEnum(colorValue)];
                }
            }
        }
    }
};
