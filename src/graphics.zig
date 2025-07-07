const std = @import("std");
const c = @cImport({
    @cDefine("SDL_DISABLE_OLD_NAMEs", {});
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_revision.h");

    // For programs that provide their own entry points instead of relying on SDL's main function
    // macro magic, 'SDL_MAIN_HANDLED' should be defined before including 'SDL_main.h'.
    @cDefine("SDL_MAIN_HANDLED", {});
    @cInclude("SDL3/SDL_main.h");
});

pub fn init() !void {
    // For programs that provide their own entry points instead of relying on SDL's main function
    // macro magic, 'SDL_SetMainReady' should be called before calling 'SDL_Init'.
    c.SDL_SetMainReady();

    _ = c.SDL_SetAppMetadata("ZigZagZog", "0.0.1", "com.zigzagzog.zigzagzog");
    if (!c.SDL_Init(c.SDL_INIT_VIDEO | c.SDL_INIT_AUDIO | c.SDL_INIT_GAMEPAD)) {
        return error.SdlError;
    }
}

pub fn createWindow(title: []const u8, w: c_int, h: c_int) !struct { *c.SDL_Window, *c.SDL_Renderer } {
    const window = if (c.SDL_CreateWindow(title.ptr, w, h, 0)) |l| l else {
        return error.SdlError;
    };

    const renderer = if (c.SDL_CreateRenderer(window, null)) |r| r else {
        return error.SdlError;
    };

    return .{ window, renderer };
}

pub fn destroyWindow(window: *c.SDL_Window, renderer: *c.SDL_Renderer) void {
    c.SDL_Quit();
    c.SDL_DestroyRenderer(renderer);
    c.SDL_DestroyWindow(window);
}
