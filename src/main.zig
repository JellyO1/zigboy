//! By convention, main.zig is where your main function lives in the case that
//! you are building an executable. If you are making a library, the convention
//! is to delete this file and start with root.zig instead.
const std = @import("std");
const clap = @import("clap");
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
const Timer = @import("timer.zig").Timer;
const tracy = @import("tracy");

pub const GameBoyState = struct {
    var gpa = std.heap.GeneralPurposeAllocator(.{}).init;
    const allocator = gpa.allocator();
    /// Picture Processing Unit
    ppu: *PPU,
    /// Central Processing Unit
    cpu: *CPU,
    /// Memory management unit
    mmu: *MMU,
    timer: *Timer,
    /// Total cycles elapsed
    cycles: u64,

    pub fn init(boot_rom_path: ?[]const u8, game_rom_path: []const u8) !GameBoyState {
        var boot_rom_slice: ?[0x100]u8 = null;

        std.log.info("{?s}\n{s}\n", .{ boot_rom_path, game_rom_path });
        // Load bootrom to memory
        if (boot_rom_path != null) {
            const boot_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, boot_rom_path.?, 256);
            defer std.heap.page_allocator.free(boot_rom);
            boot_rom_slice = boot_rom[0..0x100].*;
        }

        const game_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, game_rom_path, 1024 * 1024);
        defer std.heap.page_allocator.free(game_rom);

        const mmui = try allocator.create(MMU);
        mmui.* = MMU.init(boot_rom_slice, game_rom);

        const cpui = try allocator.create(CPU);
        cpui.* = CPU.init(if (boot_rom_path == null) cpu.Registers{
            .A = 0x11,
            .B = 0x00,
            .C = 0x00,
            .D = 0xFF,
            .E = 0x56,
            .H = 0x00,
            .L = 0x0D,
            .F = cpu.Flags.init(.{
                .Z = true,
            }),
            .PC = 0x100,
            .SP = 0xFFFE,
        } else cpu.Registers.init(), mmui, null, null, null);

        const ppui = try allocator.create(PPU);
        ppui.* = PPU.init(mmui);

        const timer = try allocator.create(Timer);
        timer.* = Timer.init(mmui);
        return .{
            .cpu = cpui,
            .mmu = mmui,
            .ppu = ppui,
            .timer = timer,
            .cycles = 0,
        };
    }

    pub fn deinit(self: *GameBoyState) void {
        allocator.destroy(self.timer);
        allocator.destroy(self.ppu);
        allocator.destroy(self.cpu);
        allocator.destroy(self.mmu);
        const check = gpa.deinit();

        if (check == std.heap.Check.leak) {
            std.debug.panic("Mem leak", .{});
        }
    }

    pub fn initTest(registers: cpu.Registers, ime: bool, ei_delay: bool) !GameBoyState {
        const mmui = try allocator.create(MMU);
        mmui.* = MMU.init(null, null);

        const cpui = try allocator.create(CPU);
        cpui.* = CPU.init(registers, mmui, ime, ei_delay, null);

        const ppui = try allocator.create(PPU);
        ppui.* = PPU.init(mmui);

        const timer = try allocator.create(Timer);
        timer.* = Timer.init(mmui);
        return .{
            .cpu = cpui,
            .mmu = mmui,
            .ppu = ppui,
            .timer = timer,
            .cycles = 0,
        };
    }

    pub fn step(self: *GameBoyState) void {
        const cycles = self.cpu.step();
        self.cycles += cycles;
        self.timer.step(cycles);
        self.ppu.step(cycles);

        // sleep for the number of nanoseconds equivalent to the number of cycles
        // std.time.sleep((cycles / CPUClockRate) * std.time.ns_per_s);
    }

    pub fn keyPress(self: *GameBoyState, key: c.SDL_Keycode, down: bool) void {
        const joypad: *mmu.Joypad = @ptrCast(self.mmu.readPtr(0xFF00));

        switch (key) {
            // A, RIGHT
            c.SDLK_Z, c.SDLK_RIGHT => joypad.*.ARight = !down,
            // B, LEFT
            c.SDLK_X, c.SDLK_LEFT => joypad.*.BLeft = !down,
            // START, DOWN
            c.SDLK_RETURN, c.SDLK_DOWN => joypad.*.StartDown = !down,
            // SELECT, UP
            c.SDLK_BACKSPACE, c.SDLK_UP => joypad.*.SelectUp = !down,
            else => {
                // Don't fire any interrupt
                return;
            },
        }

        // Request Joypad interrupt
        if (!joypad.DisableButtons or !joypad.DisableDpad) {
            const IF: *mmu.InterruptFlags = @ptrCast(self.mmu.readPtr(0xFF0F));
            IF.Joypad = true;
        }
    }
};

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
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    // First we specify what parameters our program can take.
    // We can use `parseParamsComptime` to parse a string into an array of `Param(Help)`.
    const params = comptime clap.parseParamsComptime(
        \\-h, --help             Display this help and exit.
        \\-b, --boot <str>   An option parameter, which takes a value.
        \\<str>...
        \\
    );

    const res = try clap.parse(clap.Help, &params, clap.parsers.default, .{
        .allocator = gpa.allocator(),
    });
    defer res.deinit();

    var gameBoyState: GameBoyState = try GameBoyState.init(res.args.boot, res.positionals[0][0]);
    defer gameBoyState.deinit();

    {
        errdefer |err| if (err == error.SdlError) std.log.err("SDL error: {s}", .{c.SDL_GetError()});

        try graphics.init();

        const mainWindow, const mainRenderer = try graphics.createWindow(@as([]const u8, "ZigBoy"), 1920, 1080);
        defer graphics.destroyWindow(mainWindow, mainRenderer);
        const framebufferTexture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 160, 144);
        const tileTexture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 128, 192);
        const tilemapTexture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 256, 256);
        const tilemap2Texture = c.SDL_CreateTexture(@ptrCast(mainRenderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 256, 256);

        var tileBuffer: ?*anyopaque = null;
        var tilePitch: c_int = 0;

        var tilemapBuffer: ?*anyopaque = null;
        var tilemapPitch: c_int = 0;

        var tilemap2Buffer: ?*anyopaque = null;
        var tilemap2Pitch: c_int = 0;

        const TARGET_FPS = 59.7;
        const FRAME_TIME_MS: f64 = 1000.0 / TARGET_FPS; // ~16.75 ms per frame
        const CYCLES_PER_FRAME = 70224; // Game Boy cycles per frame (4.194304 MHz / 59.7 Hz)

        var lastFrameTime = c.SDL_GetTicks();

        mainLoop: while (true) {
            tracy.frameMark();

            gameBoyState.step();

            var ev: c.SDL_Event = undefined;
            while (c.SDL_PollEvent(&ev)) {
                _ = c.cImGui_ImplSDL3_ProcessEvent(&ev);
                switch (ev.type) {
                    c.SDL_EVENT_QUIT => break :mainLoop,
                    c.SDL_EVENT_KEY_DOWN, c.SDL_EVENT_KEY_UP => {
                        const keyEvent: *c.SDL_KeyboardEvent = @ptrCast(&ev);
                        gameBoyState.keyPress(keyEvent.key, keyEvent.down);
                    },
                    else => {},
                }
                if (ev.type == c.SDL_EVENT_QUIT) {
                    break :mainLoop;
                }
            }

            // delay to keep steady fps
            if (gameBoyState.cycles >= CYCLES_PER_FRAME) {
                gameBoyState.cycles -= CYCLES_PER_FRAME;

                const currentTime = c.SDL_GetTicks();
                const frameTime: f64 = @floatFromInt(currentTime - lastFrameTime);

                if (frameTime < FRAME_TIME_MS) {
                    const delayTime: u32 = @intFromFloat(FRAME_TIME_MS - frameTime);
                    c.SDL_Delay(delayTime);
                    // std.debug.print("slept for {d} ms", .{delayTime});
                }

                // Prepare for ImGui frame.
                // Start the Dear ImGui frame
                c.cImGui_ImplSDLRenderer3_NewFrame();
                c.cImGui_ImplSDL3_NewFrame();
                c.ImGui_NewFrame();
                //std.debug.print("{}\n", .{gameBoyState.registers});

                _ = c.SDL_UpdateTexture(framebufferTexture, null, &gameBoyState.ppu.framebuffer, 160 * @sizeOf(ppu.RGBA));

                _ = c.SDL_LockTexture(tileTexture, null, &tileBuffer, &tilePitch);
                gameBoyState.ppu.debugTileset(@ptrCast(@alignCast(tileBuffer)));
                _ = c.SDL_UnlockTexture(tileTexture);

                _ = c.SDL_LockTexture(tilemapTexture, null, &tilemapBuffer, &tilemapPitch);
                gameBoyState.ppu.debugTilemap(@ptrCast(@alignCast(tilemapBuffer)), ppu.Tilemap.T9800);
                _ = c.SDL_UnlockTexture(tilemapTexture);

                _ = c.SDL_LockTexture(tilemap2Texture, null, &tilemap2Buffer, &tilemap2Pitch);
                gameBoyState.ppu.debugTilemap(@ptrCast(@alignCast(tilemap2Buffer)), ppu.Tilemap.T9C00);
                _ = c.SDL_UnlockTexture(tilemap2Texture);

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
                c.ImGui_Text("LY: %u", gameBoyState.ppu.scanline.*);
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

                    // Tilemap texture
                    if (c.ImGui_BeginTabItem("Tilemap2", null, 0)) {
                        fit = fitAspect(@floatFromInt(tilemap2Texture.*.w), @floatFromInt(tilemap2Texture.*.h));

                        c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(tilemap2Texture) }, c.ImVec2{ .x = fit.w, .y = fit.h });
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

                lastFrameTime = c.SDL_GetTicks();
            }
        }
    }
}
