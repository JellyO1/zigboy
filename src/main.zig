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

const window = @import("window.zig");
const ui = @import("ui.zig");
const cpu = @import("cpu.zig");
const CPU = cpu.CPU;
const ppu = @import("ppu.zig");
const PPU = ppu.PPU;
const mmu = @import("mmu.zig");
const MMU = mmu.MMU;
const Timer = @import("timer.zig").Timer;
const MBC = @import("mbc.zig").MBC;
const tracy = @import("tracy");

pub const Emulator = struct {
    /// Picture Processing Unit
    ppu: *PPU,
    /// Central Processing Unit
    cpu: *CPU,
    /// Memory management unit
    mmu: *MMU,
    timer: *Timer,
    /// Memory Bank Controller
    mbc: *MBC,
    /// Total cycles elapsed
    cycles: u64,

    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, boot_rom_path: ?[]const u8, game_rom_path: []const u8, logToFile: ?bool) !Emulator {
        var boot_rom_slice: ?[0x100]u8 = null;

        // Load bootrom to memory
        if (boot_rom_path != null) {
            const boot_rom = try std.fs.cwd().readFileAlloc(std.heap.page_allocator, boot_rom_path.?, 256);
            defer std.heap.page_allocator.free(boot_rom);
            boot_rom_slice = boot_rom[0..0x100].*;
        }

        const mbc = try allocator.create(MBC);
        mbc.* = try MBC.init(allocator, game_rom_path);

        const mmui = try allocator.create(MMU);
        mmui.* = MMU.init(boot_rom_slice, mbc);

        const cpui = try allocator.create(CPU);
        cpui.* = try CPU.init(allocator, if (boot_rom_path == null) cpu.Registers{
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
        } else cpu.Registers.init(), mmui, null, null, null, logToFile);

        const ppui = try allocator.create(PPU);
        ppui.* = PPU.init(mmui, allocator);

        const timer = try allocator.create(Timer);
        timer.* = Timer.init(mmui);

        return .{
            .cpu = cpui,
            .mmu = mmui,
            .ppu = ppui,
            .timer = timer,
            .mbc = mbc,
            .cycles = 0,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *Emulator) void {
        self.ppu.deinit();
        self.cpu.deinit();
        self.mbc.deinit();

        self.allocator.destroy(self.timer);
        self.allocator.destroy(self.ppu);
        self.allocator.destroy(self.cpu);
        self.allocator.destroy(self.mmu);
        self.allocator.destroy(self.mbc);
    }

    pub fn initTest(allocator: std.mem.Allocator, registers: cpu.Registers, ime: bool, ei_delay: bool) !Emulator {
        const mmui = try allocator.create(MMU);
        mmui.* = MMU.init(null, null);

        const cpui = try allocator.create(CPU);
        cpui.* = try CPU.init(registers, mmui, ime, ei_delay, null);

        const ppui = try allocator.create(PPU);
        ppui.* = PPU.init(mmui, allocator);

        const timer = try allocator.create(Timer);
        timer.* = Timer.init(mmui);
        return .{
            .cpu = cpui,
            .mmu = mmui,
            .ppu = ppui,
            .timer = timer,
            .cycles = 0,
            .allocator = allocator,
        };
    }

    pub fn step(self: *Emulator) void {
        const ticks = self.cpu.step();
        self.cycles += ticks;
        self.ppu.step(ticks);
        self.timer.step(ticks);

        // sleep for the number of nanoseconds equivalent to the number of cycles
        // std.time.sleep((cycles / CPUClockRate) * std.time.ns_per_s);
    }

    pub fn keyPress(self: *Emulator, key: c.SDL_Keycode, down: bool) void {
        const joypad: *mmu.Joypad = @ptrCast(self.mmu.readPtr(mmu.MMU.JOYP_ADDR));

        switch (key) {
            // A
            c.SDLK_Z => joypad.*.A = down,
            // RIGHT
            c.SDLK_RIGHT => joypad.*.Right = down,
            // B
            c.SDLK_X => joypad.*.B = down,
            //LEFT
            c.SDLK_LEFT => joypad.*.Left = down,
            // START
            c.SDLK_RETURN => joypad.*.Start = down,
            // DOWN
            c.SDLK_DOWN => joypad.*.Down = down,
            // SELECT
            c.SDLK_BACKSPACE => joypad.*.Start = down,
            // UP
            c.SDLK_UP => joypad.*.Up = down,
            // or (joypad.DisableButtons and joypad.DisableDpad),
            else => {
                // Don't fire any interrupt
                return;
            },
        }

        // Request Joypad interrupt
        if ((!joypad.EnableButtons or !joypad.EnableDpad) and down) {
            const IF: *mmu.InterruptFlags = @ptrCast(self.mmu.readPtr(0xFF0F));
            IF.Joypad = true;
        }
    }
};

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    defer _ = gpa.deinit();

    // First we specify what parameters our program can take.
    // We can use `parseParamsComptime` to parse a string into an array of `Param(Help)`.
    const params = comptime clap.parseParamsComptime(
        \\-h, --help             Display this help and exit.
        \\-b, --boot <str>   An optional path to a DMG boot rom.
        \\-l, --log <usize>  If set logs the CPU execution to a file.
        \\<str>...
        \\
    );

    const res = try clap.parse(clap.Help, &params, clap.parsers.default, .{
        .allocator = gpa.allocator(),
    });
    defer res.deinit();

    var emulator: Emulator = try Emulator.init(gpa.allocator(), res.args.boot, res.positionals[0][0], res.args.log orelse 0 > 0);
    defer emulator.deinit();

    emulator.mbc.printInfo();

    {
        errdefer |err| if (err == error.SdlError) std.log.err("SDL error: {s}", .{c.SDL_GetError()});

        try window.init();

        const mainWindow, const mainRenderer = try window.createWindow(@as([]const u8, "ZigBoy"), 1920, 1080);
        defer window.destroyWindow(mainWindow, mainRenderer);

        var gui = ui.UI.init(&emulator, @ptrCast(mainRenderer));

        const TARGET_FPS = 59.7;
        const FRAME_TIME_MS: f64 = 1000.0 / TARGET_FPS; // ~16.75 ms per frame
        const CYCLES_PER_FRAME = 70224; // Game Boy cycles per frame (4.194304 MHz / 59.7 Hz)

        var lastFrameTime = c.SDL_GetTicks();

        mainLoop: while (true) {
            emulator.step();

            var ev: c.SDL_Event = undefined;
            while (c.SDL_PollEvent(&ev)) {
                _ = c.cImGui_ImplSDL3_ProcessEvent(&ev);
                switch (ev.type) {
                    c.SDL_EVENT_QUIT => break :mainLoop,
                    c.SDL_EVENT_KEY_DOWN, c.SDL_EVENT_KEY_UP => {
                        const keyEvent: *c.SDL_KeyboardEvent = @ptrCast(&ev);
                        emulator.keyPress(keyEvent.key, keyEvent.down);
                    },
                    else => {},
                }
                if (ev.type == c.SDL_EVENT_QUIT) {
                    break :mainLoop;
                }
            }

            // delay to keep steady fps
            if (emulator.cycles >= CYCLES_PER_FRAME) {
                emulator.cycles -= CYCLES_PER_FRAME;

                const currentTime = c.SDL_GetTicks();
                const frameTime: f64 = @floatFromInt(currentTime - lastFrameTime);

                if (frameTime < FRAME_TIME_MS) {
                    const delayTime: u32 = @intFromFloat(FRAME_TIME_MS - frameTime);
                    c.SDL_Delay(delayTime);
                    // std.debug.print("slept for {d} ms", .{delayTime});
                }

                gui.draw();

                lastFrameTime = c.SDL_GetTicks();
            }
        }
    }
}
