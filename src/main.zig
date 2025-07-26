//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.
const std = @import("std");
const c = @cImport({
    @cDefine("SDL_DISABLE_OLD_NAMEs", {});
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_revision.h");

    @cInclude("dcimgui.h");
    @cInclude("dcimgui_internal.h");
    @cInclude("backends/dcimgui_impl_sdl3.h");
    @cInclude("backends/dcimgui_impl_sdlrenderer3.h");
});

const graphics = @import("graphics.zig");
const cpu = @import("cpu.zig");
const CPU = cpu.CPU;
const ppu = @import("ppu.zig");
const PPU = ppu.PPU;
const mmu = @import("mmu.zig");
const MMU = mmu.MMU;
const tracy = @import("tracy");

pub const GameBoyState = struct {
    /// Picture Processing Unit
    ppu: *PPU,
    /// Central Processing Unit
    cpu: *CPU,
    /// Memory management unit
    mmu: *MMU,
    /// Total cycles elapsed
    cycles: u64,

    pub fn init(flags: ?cpu.Flags) !GameBoyState {
        // Load bootrom to memory
        const boot_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, "src/dmg_boot.bin", 256);
        defer std.heap.page_allocator.free(boot_rom);
        const boot_rom_slice = boot_rom[0..0x100].*;

        const game_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, "src/cpu_instrs.gb", 1024 * 1024);
        defer std.heap.page_allocator.free(game_rom);

        var mmui = MMU.init(boot_rom_slice, game_rom);
        return .{
            .cpu = @constCast(
                &CPU.init(cpu.Registers.init(flags), &mmui, null, null, null),
            ),
            .mmu = &mmui,
            .ppu = @constCast(&PPU.init(&mmui)),
            .cycles = 0,
        };
    }

    pub fn initTest(registers: cpu.Registers, ime: bool, ei_delay: bool) GameBoyState {
        var mmui = MMU.init(null, null);
        return .{
            .cpu = @constCast(
                &CPU.init(registers, &mmui, ime, ei_delay, null),
            ),
            .mmu = mmui,
            .ppu = PPU.init(&mmui),
            .cycles = 0,
        };
    }

    pub fn step(self: *GameBoyState) void {
        const cycles = self.cpu.step();
        self.cycles += cycles;
        self.ppu.step(cycles);

        // sleep for the number of nanoseconds equivalent to the number of cycles
        // std.time.sleep((cycles / CPUClockRate) * std.time.ns_per_s);
    }
};

pub fn debugTileData(state: *GameBoyState, buffer: *[192 * 128]ppu.RGBA) void {
    const colors = [4]ppu.RGBA{
        .{ .r = 255, .g = 255, .b = 255, .a = 255 },
        .{ .r = 192, .g = 192, .b = 192, .a = 255 },
        .{ .r = 96, .g = 96, .b = 96, .a = 255 },
        .{ .r = 0, .g = 0, .b = 0, .a = 255 },
    };

    const tilesPerRow = 24; // 24 tiles per row
    const tileSize = 8; // 8x8 pixels per tile

    for (0x8000..0x97FF) |addr| {
        const normalizedAddr = addr - 0x8000;

        if (normalizedAddr % 2 != 0) continue;
        //const row = normalizedAddr / 2;

        const tileIndex = normalizedAddr / 16; // Each tile is 16 bytes, 2 bytes per each row of pixels (8 pixels)
        const rowInTile = (normalizedAddr % 16) / 2; // 0..7

        const tileX = tileIndex % tilesPerRow; // 0..23
        const tileY = tileIndex / tilesPerRow; // 0..15

        const highByte = state.mmu.read(@intCast(addr));
        const lowByte = state.mmu.read(@intCast(addr + 1));

        // Loop through every pixel in the row
        inline for (0..8) |pixelIndex| {
            const bitIndex: u8 = 1 << (7 - pixelIndex);
            const lowBit: u8 = lowByte & bitIndex;
            const highBit: u8 = highByte & bitIndex;
            const colorIndex = if (highBit == 0) @as(u2, 0b00) else @as(u2, 0b10) | if (lowBit == 0) @as(u2, 0b00) else @as(u2, 0b01);
            const color = colors[colorIndex];

            const pixelX = tileX * tileSize + pixelIndex;
            const pixelY = tileY * tileSize + rowInTile;
            const bufferIndex = pixelY * (tilesPerRow * tileSize) + pixelX;
            buffer[bufferIndex] = color;
        }
    }
}

pub fn debugTileset(state: *GameBoyState, buffer: *[192 * 128]ppu.RGBA) void {
    const colors = [4]ppu.RGBA{
        .{ .r = 255, .g = 255, .b = 255, .a = 255 },
        .{ .r = 192, .g = 192, .b = 192, .a = 255 },
        .{ .r = 96, .g = 96, .b = 96, .a = 255 },
        .{ .r = 0, .g = 0, .b = 0, .a = 255 },
    };

    const tilesPerRow = 24; // 24 tiles per row
    const tileSize = 8; // 8x8 pixels per tile

    for (state.mmu.tileset, 0..) |tile, tileIndex| {
        const tileX = tileIndex % tilesPerRow; // 0..23
        const tileY = tileIndex / tilesPerRow; // 0..15

        // Loop through every pixel
        inline for (0..8) |pixelXIndex| {
            inline for (0..8) |pixelYIndex| {
                const colorIndex = tile.data[pixelYIndex][pixelXIndex];
                const color = colors[colorIndex];

                const pixelX = tileX * tileSize + pixelXIndex;
                const pixelY = tileY * tileSize + pixelYIndex;
                const bufferIndex = pixelY * (tilesPerRow * tileSize) + pixelX;
                buffer[bufferIndex] = color;
            }
        }
    }
}

pub fn debugTilemap(state: *GameBoyState, buffer: *[256 * 256]ppu.RGBA) void {
    const colors = [4]ppu.RGBA{
        .{ .r = 255, .g = 255, .b = 255, .a = 255 },
        .{ .r = 192, .g = 192, .b = 192, .a = 255 },
        .{ .r = 96, .g = 96, .b = 96, .a = 255 },
        .{ .r = 0, .g = 0, .b = 0, .a = 255 },
    };

    const tilesPerRow = 32; // 32 tiles per row
    const tileSize = 8; // 8x8 pixels per tile

    for (0x9800..0x9FFF) |addr| {
        const tileIndex = state.mmu.read(@intCast(addr));

        const tileX = tileIndex % tilesPerRow; // 0..32
        const tileY = tileIndex / tilesPerRow; // 0..32

        const tile = state.mmu.tileset[tileIndex];

        // Loop through every pixel
        inline for (0..8) |pixelXIndex| {
            inline for (0..8) |pixelYIndex| {
                const colorIndex = tile.data[pixelYIndex][pixelXIndex];
                const color = colors[colorIndex];

                const pixelX = tileX * tileSize + pixelXIndex;
                const pixelY = tileY * tileSize + pixelYIndex;
                const bufferIndex = pixelY * (tilesPerRow * tileSize) + pixelX;
                buffer[bufferIndex] = color;
            }
        }
    }
}

fn fitAspect(img_w: f32, img_h: f32) struct { w: f32, h: f32 } {
    const avail = c.ImGui_GetContentRegionAvail();
    const aspect = img_w / img_h;
    var draw_w = avail.x;
    var draw_h = avail.y;
    if (draw_w / aspect > draw_h) {
        draw_w = draw_h * aspect;
    } else {
        draw_h = draw_w / aspect;
    }
    return .{ .w = draw_w, .h = draw_h };
}

pub fn main() !void {
    var gameBoyState: GameBoyState = try GameBoyState.init(null);

    {
        errdefer |err| if (err == error.SdlError) std.log.err("SDL error: {s}", .{c.SDL_GetError()});

        try graphics.init();

        const mainWindow, const mainRenderer = try graphics.createWindow(@as([]const u8, "ZigBoy"), 1920, 1080);
        defer graphics.destroyWindow(mainWindow, mainRenderer);
        const framebufferTexture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 160, 144);
        const tileTexture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 192, 128);
        const tilemapTexture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 256, 256);

        var tileBuffer: ?*anyopaque = null;
        var tilePitch: c_int = 0;

        var tilemapBuffer: ?*anyopaque = null;
        var tilemapPitch: c_int = 0;

        mainLoop: while (true) {
            tracy.frameMark();

            var ev: c.SDL_Event = undefined;
            while (c.SDL_PollEvent(&ev)) {
                _ = c.cImGui_ImplSDL3_ProcessEvent(&ev);
                if (ev.type == c.SDL_EVENT_QUIT) {
                    break :mainLoop;
                }
            }

            // Prepare for ImGui frame.
            // Start the Dear ImGui frame
            c.cImGui_ImplSDLRenderer3_NewFrame();
            c.cImGui_ImplSDL3_NewFrame();
            c.ImGui_NewFrame();

            gameBoyState.step();
            //std.debug.print("{}\n", .{gameBoyState.registers});

            // HACK: Only render and present at the end of the last VBlank scanline
            if (gameBoyState.ppu.mode == .VBlank and gameBoyState.ppu.scanline.* == 144) {
                _ = c.SDL_UpdateTexture(framebufferTexture, null, &gameBoyState.ppu.framebuffer, 160 * @sizeOf(ppu.RGBA));

                _ = c.SDL_LockTexture(tileTexture, null, &tileBuffer, &tilePitch);
                debugTileset(&gameBoyState, @ptrCast(@alignCast(tileBuffer)));
                _ = c.SDL_UnlockTexture(tileTexture);

                _ = c.SDL_LockTexture(tilemapTexture, null, &tilemapBuffer, &tilemapPitch);
                debugTilemap(&gameBoyState, @ptrCast(@alignCast(tilemapBuffer)));
                _ = c.SDL_UnlockTexture(tilemapTexture);
            }

            var windowFlags: c_int = c.ImGuiWindowFlags_MenuBar | c.ImGuiWindowFlags_NoDocking;
            const viewport = c.ImGui_GetMainViewport();
            c.ImGui_SetNextWindowPos(viewport.*.Pos, 0);
            c.ImGui_SetNextWindowSize(viewport.*.Size, 0);
            c.ImGui_PushStyleVar(c.ImGuiStyleVar_WindowRounding, 0.0);
            c.ImGui_PushStyleVar(c.ImGuiStyleVar_WindowBorderSize, 0.0);
            c.ImGui_PushStyleVarImVec2(c.ImGuiStyleVar_WindowPadding, c.ImVec2{ .x = 0.0, .y = 0.0 });
            windowFlags |= c.ImGuiWindowFlags_NoTitleBar | c.ImGuiWindowFlags_NoCollapse | c.ImGuiWindowFlags_NoResize | c.ImGuiWindowFlags_NoMove;
            windowFlags |= c.ImGuiWindowFlags_NoBringToFrontOnFocus | c.ImGuiWindowFlags_NoNavFocus;

            _ = c.ImGui_Begin("ZigBoy", @ptrCast(@constCast(&true)), windowFlags);
            c.ImGui_PopStyleVarEx(3);

            const dockspaceId = c.ImGui_GetID("MyDockSpace");
            _ = c.ImGui_DockSpaceEx(dockspaceId, c.ImVec2{ .x = 0.0, .y = 0.0 }, 0, 0);

            const S = struct {
                var firstTime: bool = true;
            };

            if (S.firstTime) {
                S.firstTime = false;

                // c.ImGui_DockBuilderRemoveNode(dockspaceId); // clear any previous layout
                // _ = c.ImGui_DockBuilderAddNodeEx(dockspaceId, c.ImGuiDockNodeFlags_DockSpace);
                // c.ImGui_DockBuilderSetNodeSize(dockspaceId, viewport.*.Size);

                // split the dockspace into 2 nodes -- DockBuilderSplitNode takes in the following args in the following order
                //   window ID to split, direction, fraction (between 0 and 1), the final two setting let's us choose which id we want (which ever one we DON'T set as NULL, will be returned by the function)
                //                                                              out_id_at_dir is the id of the node in the direction we specified earlier, out_id_at_opposite_dir is in the opposite direction
                const dockIdRight = c.ImGui_DockBuilderSplitNode(dockspaceId, c.ImGuiDir_Left, 0.5, null, @ptrCast(@constCast(&dockspaceId)));
                const dockIdLeft = c.ImGui_DockBuilderSplitNode(dockspaceId, c.ImGuiDir_Left, 0.5, null, @ptrCast(@constCast(&dockspaceId)));

                // we now dock our windows into the docking node we made above
                c.ImGui_DockBuilderDockWindow("Display", dockIdLeft);
                c.ImGui_DockBuilderDockWindow("Debug", dockIdRight);
                c.ImGui_DockBuilderFinish(dockspaceId);
            }

            c.ImGui_End();

            _ = c.ImGui_Begin("Display", null, 0);
            var fit = fitAspect(@floatFromInt(framebufferTexture.*.w), @floatFromInt(framebufferTexture.*.h));
            c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(framebufferTexture) }, c.ImVec2{ .x = fit.w, .y = fit.h });
            c.ImGui_End();

            // Memory TabBar
            _ = c.ImGui_Begin("Debug", null, 0);
            _ = c.ImGui_BeginChild("RegistersRow", c.ImVec2{ .x = 0, .y = 40 }, 0, 0);
            c.ImGui_Text("A: 0x%02X", gameBoyState.cpu.*.registers.A);
            c.ImGui_SameLine();
            c.ImGui_Text("B: 0x%02X", gameBoyState.cpu.*.registers.B);
            c.ImGui_SameLine();
            c.ImGui_Text("C: 0x%02X", gameBoyState.cpu.*.registers.C);
            c.ImGui_SameLine();
            c.ImGui_Text("D: 0x%02X", gameBoyState.cpu.*.registers.D);
            c.ImGui_SameLine();
            c.ImGui_Text("E: 0x%02X", gameBoyState.cpu.*.registers.E);
            c.ImGui_SameLine();
            c.ImGui_Text("H: 0x%02X", gameBoyState.cpu.*.registers.H);
            c.ImGui_SameLine();
            c.ImGui_Text("L: 0x%02X", gameBoyState.cpu.*.registers.L);
            c.ImGui_SameLine();
            c.ImGui_Text("PC: 0x%04X", gameBoyState.cpu.*.registers.PC);
            c.ImGui_SameLine();
            c.ImGui_Text("SP: 0x%04X", gameBoyState.cpu.*.registers.SP);
            c.ImGui_Text("LY: %u", gameBoyState.mmu.read(0xFF44));
            c.ImGui_EndChild();
            if (c.ImGui_BeginTabBar("DebugTabBar", 0)) {
                // Tile texture
                if (c.ImGui_BeginTabItem("VRAM", null, 0)) {
                    fit = fitAspect(@floatFromInt(tileTexture.*.w), @floatFromInt(tileTexture.*.h));
                    c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(tileTexture) }, c.ImVec2{ .x = fit.w, .y = fit.h });
                    c.ImGui_EndTabItem();
                }

                // Tilemap texture
                if (c.ImGui_BeginTabItem("Tilemap", null, 0)) {
                    fit = fitAspect(@floatFromInt(tilemapTexture.*.w), @floatFromInt(tilemapTexture.*.h));

                    c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(tilemapTexture) }, c.ImVec2{ .x = fit.w, .y = fit.h });
                    c.ImGui_EndTabItem();
                }

                c.ImGui_EndTabBar();
            }
            c.ImGui_End();

            c.ImGui_Render();
            const drawData: *c.ImDrawData = c.ImGui_GetDrawData();
            const io = c.ImGui_GetIO();
            _ = c.SDL_SetRenderScale(@ptrCast(mainRenderer), io.*.DisplayFramebufferScale.x, io.*.DisplayFramebufferScale.y);
            _ = c.SDL_RenderClear(@ptrCast(mainRenderer));
            c.cImGui_ImplSDLRenderer3_RenderDrawData(drawData, @ptrCast(mainRenderer));
            _ = c.SDL_RenderPresent(@ptrCast(mainRenderer));
        }
    }
}
