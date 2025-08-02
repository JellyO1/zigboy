const std = @import("std");
const GameBoyState = @import("main.zig").GameBoyState;
const mmuz = @import("mmu.zig");

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
    Mode: Mode = .OAM,
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

pub const RGBA = packed struct(u32) {
    a: u8,
    b: u8,
    g: u8,
    r: u8,
};

const BGPaletteColors = [4]RGBA{
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

pub const Tilemap = enum(u1) { T9800, T9C00 };

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
    mmu: *mmuz.MMU,
    mode: Mode,
    framebuffer: [160 * 144]RGBA,
    /// LCD Control register
    lcdc: *LCDControl,
    /// LCD Status register
    stat: *LCDStatus,
    /// Background Y scroll (SCY)
    bgScrollY: *u8,
    /// Background X scroll (SCX)
    bgScrollX: *u8,
    /// Current scanline (LY) (0-153)
    scanline: *u8,
    /// Internal window LY
    winLY: u8,
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
    IF: *mmuz.InterruptFlags,

    pub fn init(mmu: *mmuz.MMU) PPU {
        const lcdc: *LCDControl = @ptrCast(mmu.readPtr(mmuz.MMU.LCDC_ADDR));

        return .{
            .mode = Mode.OAM,
            .mmu = mmu,
            .modeClock = 0,
            .framebuffer = std.mem.zeroes([160 * 144]RGBA),
            .lcdc = lcdc,
            .stat = @ptrCast(mmu.readPtr(mmuz.MMU.STAT_ADDR)),
            .bgScrollY = mmu.readPtr(mmuz.MMU.BG_SCROLL_Y_ADDR),
            .bgScrollX = mmu.readPtr(mmuz.MMU.BG_SCROLL_X_ADDR),
            .scanline = mmu.readPtr(mmuz.MMU.LY_ADDR),
            .winLY = 0,
            .lyc = mmu.readPtr(mmuz.MMU.LYC_ADDR),
            .bgp = @ptrCast(mmu.readPtr(mmuz.MMU.BG_PALETTE_ADDR)),
            .op0 = @ptrCast(mmu.readPtr(mmuz.MMU.OBJ_0_PALETTE_ADDR)),
            .op1 = @ptrCast(mmu.readPtr(mmuz.MMU.OBJ_1_PALETTE_ADDR)),
            .winY = mmu.readPtr(mmuz.MMU.OBJ_WIN_Y_ADDR),
            .winX = mmu.readPtr(mmuz.MMU.OBJ_WIN_X_ADDR),
            .IF = @ptrCast(mmu.readPtr(mmuz.MMU.IF_ADDR)),
        };
    }

    pub fn step(self: *PPU, cycles: u32) void {
        if (!self.lcdc.LcdPpuEnable) {
            self.modeClock += cycles;
            return;
        }

        for (0..cycles) |_| {
            self.modeClock += 1;

            switch (self.mode) {
                .OAM => {
                    if (self.modeClock % 80 == 0) {
                        self.modeClock = 0;
                        self.mode = .Drawing;
                        self.stat.Mode = self.mode;
                    }
                },
                .Drawing => {
                    if (self.modeClock % 172 == 0) {
                        self.modeClock = 0;

                        // TODO: Render scanline to framebuffer
                        self.renderScanline();

                        self.mode = .HBlank;
                        self.stat.Mode = self.mode;

                        if (self.stat.Mode0) {
                            self.IF.LCD = true;
                        }
                    }
                },
                .HBlank => {
                    if (self.modeClock % 204 == 0) {
                        self.modeClock = 0;
                        self.scanline.* +%= 1;

                        self.setLYCEqualsLY();

                        if (self.scanline.* == 143) {
                            self.mode = .VBlank;
                            self.stat.Mode = self.mode;

                            if (self.stat.Mode1) {
                                self.IF.LCD = true;
                            }

                            self.IF.VBlank = true;
                        } else {
                            self.mode = .OAM;
                            self.stat.Mode = self.mode;

                            if (self.stat.Mode2) {
                                self.IF.LCD = true;
                            }
                        }
                    }
                },
                .VBlank => {
                    if (self.modeClock % 456 == 0) {
                        self.modeClock = 0;
                        self.scanline.* +%= 1;

                        self.setLYCEqualsLY();

                        if (self.scanline.* == 154) {
                            self.mode = .OAM;
                            self.stat.Mode = self.mode;

                            if (self.stat.Mode2) {
                                self.IF.LCD = true;
                            }

                            self.scanline.* = 0;
                            self.winLY = 0;
                        }
                    }
                },
            }
        }
    }

    fn renderScanline(self: *PPU) void {
        var winYIncremented = false;
        for (0..160) |x| {
            self.renderBackground(x);

            // Increment the internal winLY
            // NOTE: Why is there a missing row when using 'self.scanline.* >= self.winY.*'
            if (self.lcdc.WinEnable and
                self.scanline.* > self.winY.* and @as(u8, @intCast(x)) >= (self.winX.* -| 7) and
                !winYIncremented)
            {
                self.winLY +%= 1;
                winYIncremented = true;
            }

            self.renderWindow(x);
            self.renderSprites(x);
        }
    }

    fn renderBackground(self: *PPU, x: usize) void {
        const xu8: u8 = @intCast(x);

        if (self.lcdc.BgWinEnable) {
            const bgX: u8 = xu8 +% self.bgScrollX.*;
            const bgY: u8 = self.scanline.* +% self.bgScrollY.*;

            const tileX: u16 = bgX / 8;
            const tileY: u16 = bgY / 8;

            const pixelX: u8 = bgX % 8;
            const pixelY: u8 = bgY % 8;

            const tileMapAddr: u16 = if (self.lcdc.BgTileMapArea) 0x9C00 else 0x9800;
            const tileIndex: u8 = @intCast(self.mmu.read(tileMapAddr + @as(u16, @intCast(tileY * 32 + tileX))));

            var cacheIndex: usize = 0;
            if (self.lcdc.BgWinTileDataArea) {
                cacheIndex = tileIndex; // 0..255
            } else {
                const signedIndex: i8 = @bitCast(tileIndex); // -128..127
                cacheIndex = @as(usize, @bitCast(@as(isize, signedIndex) + 256)); // 128..383
            }

            const tile = self.mmu.tileset[cacheIndex];
            const color = tile.data[pixelY][pixelX];
            const palette = self.bgp;
            const colorValue = palette.get(color);

            const frameBufferIndex = x + @as(usize, @intCast((self.scanline.*))) * 160;
            self.framebuffer[frameBufferIndex] = BGPaletteColors[@intFromEnum(colorValue)];
        } else {
            // TODO: change framebuffer init to be 0xFF instead of zeroes
            const palette = self.bgp;
            const colorValue = palette.get(0);

            const frameBufferIndex = x + @as(usize, @intCast((self.scanline.*))) * 160;
            self.framebuffer[frameBufferIndex] = BGPaletteColors[@intFromEnum(colorValue)];
        }
    }

    fn renderWindow(self: *PPU, x: usize) void {
        const xu8: u8 = @intCast(x);

        if (self.lcdc.WinEnable and self.lcdc.BgWinEnable and self.scanline.* >= self.winY.* and xu8 >= (self.winX.* -| 7)) {
            // -| is saturating subtraction, meaning it clamps to 0
            const wX: u16 = xu8 - (@as(u16, self.winX.*) -| 7);
            const wY: u16 = self.scanline.* - self.winY.*;

            // std.log.info("winY:{} winX:{} wX:{} wY:{} winLY:{} scanline: {}\n", .{ self.winY.*, self.winX.*, wX, wY, self.winLY, self.scanline.* });

            const tileX: u16 = wX / 8;
            const tileY: u16 = self.winLY / 8;

            // std.log.info("tileX:{} tileY:{} other:{}\n", .{ &tileX, &tileY, tileY * 32 + tileX });

            const pixelX = wX % 8;
            const pixelY = wY % 8;

            const tileMapAddr: u16 = if (self.lcdc.WinTileMapArea) 0x9C00 else 0x9800;
            const tileIndex: u8 = @intCast(self.mmu.read(tileMapAddr + @as(u16, @intCast(tileY * 32 + tileX))));

            var cacheIndex: usize = 0;
            if (self.lcdc.BgWinTileDataArea) {
                cacheIndex = tileIndex; // 0..255
            } else {
                const signedIndex: i8 = @bitCast(tileIndex); // -128..127
                cacheIndex = @as(usize, @bitCast(@as(isize, signedIndex) + 256)); // 128..383
            }

            const tile = self.mmu.tileset[cacheIndex];
            const color = tile.data[pixelY][pixelX];
            const palette = self.bgp;
            const colorValue = palette.get(color);

            const frameBufferIndex = xu8 + @as(usize, @intCast((self.scanline.*))) * 160;
            self.framebuffer[frameBufferIndex] = BGPaletteColors[@intFromEnum(colorValue)];
        }
    }

    fn renderSprites(self: *PPU, x: usize) void {
        _ = self;
        _ = x;
    }

    fn setLYCEqualsLY(self: *PPU) void {
        self.stat.LYCEqualsLY = self.scanline.* == self.lyc.*;
        if (self.stat.LYCEqualsLY and self.stat.LYC) {
            self.IF.LCD = true;
        }
    }

    pub fn debugTileset(state: *PPU, buffer: *[128 * 192]RGBA) void {
        const tilesPerRow = 16; // 16 tiles per row
        const tileSize = 8; // 8x8 pixels per tile

        for (state.mmu.tileset, 0..) |tile, tileIndex| {
            const tileX = tileIndex % tilesPerRow; // 0..15
            const tileY = tileIndex / tilesPerRow; // 0..23

            // Loop through every pixel
            inline for (0..8) |pixelXIndex| {
                inline for (0..8) |pixelYIndex| {
                    const colorIndex = tile.data[pixelYIndex][pixelXIndex];
                    const color = BGPaletteColors[colorIndex];

                    const pixelX = tileX * tileSize + pixelXIndex;
                    const pixelY = tileY * tileSize + pixelYIndex;
                    const bufferIndex = pixelY * (tilesPerRow * tileSize) + pixelX;
                    buffer[bufferIndex] = color;
                }
            }
        }
    }

    pub fn debugTilemap(state: *PPU, buffer: *[256 * 256]RGBA, tilemap: Tilemap) void {
        if (tilemap == Tilemap.T9800) {
            for (0x9800..0x9C00) |addr| {
                drawDebugTilemap(state, addr, buffer, tilemap);
            }
        } else {
            for (0x9C00..0xA000) |addr| {
                drawDebugTilemap(state, addr, buffer, tilemap);
            }
        }
    }

    fn drawDebugTilemap(state: *PPU, addr: usize, buffer: *[256 * 256]RGBA, tilemap: Tilemap) void {
        const u16Addr: u16 = @intCast(addr);
        const tilesPerRow: u16 = 32; // 32 tiles per row
        const tileSize: u16 = 8; // 8x8 pixels per tile

        const tileI = if (tilemap == Tilemap.T9800) u16Addr - 0x9800 else u16Addr - 0x9C00;
        const tileX: u16 = tileI % tilesPerRow; // 0..31
        const tileY: u16 = tileI / tilesPerRow; // 0..31

        const tileIndex: u8 = state.mmu.read(u16Addr);
        const tile = state.mmu.tileset[tileIndex];

        // Loop through every pixel
        inline for (0..8) |pixelXIndex| {
            inline for (0..8) |pixelYIndex| {
                const colorIndex = tile.data[pixelYIndex][pixelXIndex];
                const color = BGPaletteColors[colorIndex];

                const pixelX = tileX * tileSize + pixelXIndex;
                const pixelY = tileY * tileSize + pixelYIndex;
                const bufferIndex = pixelY * (tilesPerRow * tileSize) + pixelX;

                buffer[bufferIndex] = color;
            }
        }
    }
};
