const std = @import("std");
const Emulator = @import("main.zig").Emulator;
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
    .{ .r = 232, .g = 252, .b = 204, .a = 255 },
    .{ .r = 172, .g = 212, .b = 144, .a = 255 },
    .{ .r = 84, .g = 140, .b = 112, .a = 255 },
    .{ .r = 20, .g = 44, .b = 56, .a = 255 },
};

const OBJPaletteColors = [4]RGBA{
    .{ .r = 232, .g = 252, .b = 204, .a = 255 },
    .{ .r = 172, .g = 212, .b = 144, .a = 255 },
    .{ .r = 84, .g = 140, .b = 112, .a = 255 },
    .{ .r = 20, .g = 44, .b = 56, .a = 255 },
};

const Color = enum(u2) {
    White,
    LightGray,
    DarkGray,
    Black,
};

pub const Tilemap = enum { Auto, T9800, T9C00 };
pub const TileDataArea = enum { Auto, T8000, T8800 };

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

pub const ObjectAttributes = packed struct(u8) {
    /// CGB only. Which of OBP0–7 to use
    CGBPalette: u3,
    /// CGB only. 0 = Fetch tile from VRAM bank 0, 1 = Fetch tile from VRAM bank 1
    Bank: bool,
    /// 0 = OBP0, 1 = OBP1
    DMGPalette: bool,
    /// 0 = Normal, 1 = Entire OBJ is horizontally mirrored
    XFlip: bool,
    /// 0 = Normal, 1 = Entire OBJ is vertically mirrored
    YFlip: bool,
    /// 0 = No, 1 = BG and Window color indices 1–3 are drawn over this OBJ
    Priority: bool,
};

pub const Object = packed struct(u32) {
    posY: u8,
    posX: u8,
    tileIndex: u8,
    attributes: ObjectAttributes,
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

    /// OAM Scan results
    objects: std.ArrayList(Object),

    pub fn init(mmu: *mmuz.MMU, allocator: std.mem.Allocator) PPU {
        const lcdc: *LCDControl = @ptrCast(mmu.readPtr(mmuz.MMU.LCDC_ADDR));
        const objects = std.ArrayList(Object).initCapacity(allocator, 10) catch |err| std.debug.panic("{}", .{err});

        // Default state after boot_rom
        lcdc.BgTileMapArea = true;
        lcdc.BgWinEnable = true;
        lcdc.LcdPpuEnable = true;

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
            .objects = objects,
        };
    }

    pub fn deinit(self: *PPU) void {
        self.objects.deinit();
    }

    pub fn step(self: *PPU, tCycles: u32) void {
        if (!self.lcdc.LcdPpuEnable) {
            // self.modeClock += tCycles;
            self.scanline.* = 0;

            self.stat.Mode = .HBlank;

            return;
        }

        for (0..tCycles) |_| {
            self.modeClock += 1;

            switch (self.mode) {
                .OAM => {
                    if (self.modeClock % 80 == 0) {
                        self.modeClock = 0;
                        self.OAMScan();
                        self.mode = .Drawing;
                        self.stat.Mode = self.mode;
                    }
                },
                .Drawing => {
                    if (self.modeClock % 172 == 0) {
                        self.modeClock = 0;

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

                        if (self.scanline.* == 144) {
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

    /// Scans the OAM (Object Attribute Memory) to find up to 10 sprites
    /// that are visible on the current scanline.
    fn OAMScan(self: *PPU) void {
        // Clear the list of objects from the previous scanline.
        self.objects.clearRetainingCapacity();

        // Iterate through all 40 possible objects in OAM.
        for (0..40) |index| {
            const obj = self.mmu.readObj(@intCast(index));

            // Determine the sprite's vertical position and height.
            // Y is offset by -16 and X by -8 in hardware.
            const screenY: i16 = @as(i16, obj.posY) - 16;
            const height: u8 = if (self.lcdc.ObjSize) 16 else 8;

            // Check if the sprite is vertically visible on the current scanline.
            if (@as(i16, self.scanline.*) < screenY or @as(i16, self.scanline.*) >= screenY + height) continue;

            // Add the visible object to the list for rendering.
            self.objects.append(obj) catch |err| std.debug.panic("{}", .{err});

            // The PPU can only render a maximum of 10 sprites per scanline.
            if (self.objects.items.len == 10) {
                return;
            }
        }

        // Sort the visible objects by priority.
        // Lower X-coordinate has higher priority.
        // If X-coordinates are equal, the one that appeared first in OAM has higher priority.
        std.mem.sort(Object, self.objects.items, {}, compareObjects);
    }

    /// The smaller the X coordinate, the higher the priority. When X coordinates are
    /// identical, the object located first in OAM has higher priority.
    fn compareObjects(context: void, a: Object, b: Object) bool {
        _ = context;

        // Sort by X coordinate (smaller X = higher priority)
        if (a.posX != b.posX) {
            return a.posX < b.posX;
        }

        // Keep the original order for equal X coordinates.
        return false;
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
            self.renderObjects(x);
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

            const tileX: u16 = wX / 8;
            const tileY: u16 = self.winLY / 8;

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

    /// Renders the sprites that have been selected by OAMScan for the current scanline.
    fn renderObjects(self: *PPU, x: usize) void {
        const xu8: u8 = @intCast(x);

        if (self.lcdc.ObjEnable) {
            // Draw back to front to preserve priority
            var it = std.mem.reverseIterator(self.objects.items);
            while (it.next()) |obj| {
                // Y is offset by -16 and X by -8 in hardware.
                const screenX: i16 = @as(i16, obj.posX) - 8;
                const screenY: i16 = @as(i16, obj.posY) - 16;
                const height: u8 = if (self.lcdc.ObjSize) 16 else 8;

                // Filter out pixels of the sprite that are not at the current X coordinate.
                if (@as(i16, xu8) < screenX or @as(i16, xu8) >= screenX + 8) continue;

                // Calculate the pixel's X and Y coordinate within the sprite's 8x8 or 8x16 frame.
                var pixelX: u8 = @intCast(@as(i16, xu8) - screenX);
                var pixelY: u8 = @intCast(@as(i16, self.scanline.*) - screenY);

                var tileIndex: u8 = obj.tileIndex;

                // For 8x16 sprites, we need to select the correct tile from the pair.
                if (self.lcdc.ObjSize) {
                    tileIndex = getSpriteTileIndex(obj, pixelY);
                }

                // Handle horizontal and vertical flipping of the sprite.
                if (obj.attributes.XFlip) {
                    pixelX = 7 - pixelX;
                }
                if (obj.attributes.YFlip) {
                    pixelY = height - 1 - pixelY;
                }

                // Get the tile from the tileset.
                const tile = self.mmu.tileset[tileIndex];
                // Get the color index for the pixel from the tile data.
                // For 8x16 sprites, pixelY can be > 7, so we use modulo 8.
                const color = tile.data[pixelY % 8][pixelX];

                // Color index 0 is transparent for sprites.
                if (color == 0) continue;

                // Get the actual color from the palette.
                const palette = if (obj.attributes.DMGPalette) self.op1 else self.op0;
                const colorValue = palette.get(color);

                const framebufferIndex = xu8 + @as(usize, @intCast((self.scanline.*))) * 160;

                const bgColor = self.framebuffer[framebufferIndex];

                // Handle sprite-to-background priority.
                // If the priority bit is 0, the sprite always appears on top of the background.
                // If the priority bit is 1, the sprite is behind the background and window,
                // unless the background color is white (color 0).
                if (!obj.attributes.Priority or bgColor == BGPaletteColors[@intFromEnum(Color.White)]) {
                    self.framebuffer[framebufferIndex] = OBJPaletteColors[@intFromEnum(colorValue)];
                }
            }
        }
    }

    fn setLYCEqualsLY(self: *PPU) void {
        self.stat.LYCEqualsLY = self.scanline.* == self.lyc.*;
        if (self.stat.LYCEqualsLY and self.stat.LYC) {
            self.IF.LCD = true;
        }
    }

    /// Returns the correct tile index for 8x16 sprites based on Y-flip and scanline position
    fn getSpriteTileIndex(obj: Object, pixelY: u8) u8 {
        const bottomTileIndex = obj.tileIndex | 0x01;
        const topTileIndex = obj.tileIndex & 0xFE;

        // For 8x16 sprites, the tile index is determined by the Y position
        // and whether the sprite is flipped vertically.
        if (obj.attributes.YFlip) {
            // For a flipped sprite, the top half of the screen pixels
            // correspond to the bottom tile, and vice-versa.
            if (pixelY < 8) {
                return bottomTileIndex;
            } else {
                return topTileIndex;
            }
        } else {
            // For a normal sprite, the top half corresponds to the top tile.
            if (pixelY < 8) {
                return topTileIndex;
            } else {
                return bottomTileIndex;
            }
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

    pub fn drawFramebuffer(self: *PPU, buffer: *[160 * 144]RGBA) void {
        @memcpy(buffer, &self.framebuffer);
    }

    pub fn debugTilemap(state: *PPU, buffer: *[256 * 256]RGBA, tilemap: Tilemap, tileDataAreaEnum: TileDataArea) void {
        const tileMapAddrStart: u16 = switch (tilemap) {
            Tilemap.T9800 => 0x9800,
            Tilemap.T9C00 => 0x9C00,
            Tilemap.Auto => if (state.lcdc.BgTileMapArea) 0x9C00 else 0x9800,
        };
        const tileMapAddrEnd = tileMapAddrStart + 0x400;

        const tileDataArea = switch (tileDataAreaEnum) {
            TileDataArea.T8000 => true,
            TileDataArea.T8800 => false,
            TileDataArea.Auto => state.lcdc.BgWinTileDataArea,
        };

        for (tileMapAddrStart..tileMapAddrEnd) |addr| {
            drawDebugTilemap(state, addr, buffer, tileMapAddrStart, tileDataArea);
        }
    }

    fn drawDebugTilemap(state: *PPU, addr: usize, buffer: *[256 * 256]RGBA, tileMapAddr: u16, tileDataArea: bool) void {
        const u16Addr: u16 = @intCast(addr);
        const tilesPerRow: u16 = 32; // 32 tiles per row
        const tileSize: u16 = 8; // 8x8 pixels per tile

        const tileI = u16Addr - tileMapAddr;
        const tileX: u16 = tileI % tilesPerRow; // 0..31
        const tileY: u16 = tileI / tilesPerRow; // 0..31

        const tileIndex: u8 = @intCast(state.mmu.read(tileMapAddr + @as(u16, @intCast(tileY * 32 + tileX))));

        var cacheIndex: usize = 0;

        if (tileDataArea) {
            cacheIndex = tileIndex; // 0..255
        } else {
            const signedIndex: i8 = @bitCast(tileIndex); // -128..127
            cacheIndex = @as(usize, @bitCast(@as(isize, signedIndex) + 256)); // 128..383
        }

        // Loop through every pixel
        inline for (0..8) |pixelXIndex| {
            inline for (0..8) |pixelYIndex| {
                const tile = state.mmu.tileset[cacheIndex];
                const colorIndex = tile.data[pixelYIndex][pixelXIndex];
                const palette = state.bgp;
                const colorValue = palette.get(colorIndex);
                // const colorIndex = tile.data[pixelYIndex][pixelXIndex];
                const color = BGPaletteColors[@intFromEnum(colorValue)];

                const pixelX = tileX * tileSize + pixelXIndex;
                const pixelY = tileY * tileSize + pixelYIndex;
                const bufferIndex = pixelY * (tilesPerRow * tileSize) + pixelX;

                buffer[bufferIndex] = color;
            }
        }
    }
};
