const std = @import("std");
const c = @cImport({
    @cDefine("SDL_DISABLE_OLD_NAMEs", {});
    @cInclude("SDL3/SDL.h");
    @cInclude("SDL3/SDL_revision.h");

    // For programs that provide their own entry points instead of relying on SDL's main function
    // macro magic, 'SDL_MAIN_HANDLED' should be defined before including 'SDL_main.h'.
    @cDefine("SDL_MAIN_HANDLED", {});
    @cInclude("SDL3/SDL_main.h");

    @cInclude("dcimgui.h");
    @cInclude("backends/dcimgui_impl_sdl3.h");
    @cInclude("backends/dcimgui_impl_sdlrenderer3.h");
});

pub fn init() !void {
    // For programs that provide their own entry points instead of relying on SDL's main function
    // macro magic, 'SDL_SetMainReady' should be called before calling 'SDL_Init'.
    c.SDL_SetMainReady();

    _ = c.SDL_SetAppMetadata("ZigBoy", "0.0.1", "dev.tobedefined.zigboy");
    try errify(c.SDL_Init(c.SDL_INIT_VIDEO | c.SDL_INIT_AUDIO | c.SDL_INIT_GAMEPAD));
}

pub fn createWindow(title: []const u8, w: c_int, h: c_int) !struct { *c.SDL_Window, *c.SDL_Renderer } {
    const scale = c.SDL_GetDisplayContentScale(c.SDL_GetPrimaryDisplay());
    const window = try errify(c.SDL_CreateWindow(title.ptr, @intFromFloat(@as(f32, @floatFromInt(w)) * scale), @intFromFloat(@as(f32, @floatFromInt(h)) * scale), c.SDL_WINDOW_RESIZABLE | c.SDL_WINDOW_HIGH_PIXEL_DENSITY));

    const renderer: *c.SDL_Renderer = try errify(c.SDL_CreateRenderer(window, null));

    // setup IMGUI
    // Setup Dear ImGui context
    if (c.ImGui_CreateContext(null) == null) return error.ImGuiCreateContextFailure;

    const io = c.ImGui_GetIO(); // (void)io;
    io.*.ConfigFlags |= c.ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.*.ConfigFlags |= c.ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Controls
    io.*.ConfigFlags |= c.ImGuiConfigFlags_DockingEnable; // Enable docking (NOTE: For some reason no ImGuiConfigFlags_EnableDocking is available)

    // Setup Dear ImGui style
    c.ImGui_StyleColorsDark(c.ImGui_GetStyle());

    // Setup scaling
    const style = c.ImGui_GetStyle();
    c.ImGuiStyle_ScaleAllSizes(style, scale); // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
    style.*.FontScaleDpi = scale; // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    try errify(c.cImGui_ImplSDL3_InitForSDLRenderer(window, renderer));
    try errify(c.cImGui_ImplSDLRenderer3_Init(renderer));

    return .{ window, renderer };
}

pub fn destroyWindow(window: *c.SDL_Window, renderer: *c.SDL_Renderer) void {
    c.SDL_Quit();
    c.cImGui_ImplSDLRenderer3_Shutdown();
    c.cImGui_ImplSDL3_Shutdown();
    c.ImGui_DestroyContext(null);
    c.SDL_DestroyRenderer(renderer);
    c.SDL_DestroyWindow(window);
}

/// Converts the return value of an SDL function to an error union.
pub inline fn errify(value: anytype) error{SdlError}!switch (@typeInfo(@TypeOf(value))) {
    .bool => void,
    .pointer, .optional => @TypeOf(value.?),
    .int => |info| switch (info.signedness) {
        .signed => @TypeOf(@max(0, value)),
        .unsigned => @TypeOf(value),
    },
    else => @compileError("unerrifiable type: " ++ @typeName(@TypeOf(value))),
} {
    return switch (@typeInfo(@TypeOf(value))) {
        .bool => if (!value) error.SdlError,
        .pointer, .optional => value orelse error.SdlError,
        .int => |info| switch (info.signedness) {
            .signed => if (value >= 0) @max(0, value) else error.SdlError,
            .unsigned => if (value != 0) value else error.SdlError,
        },
        else => comptime unreachable,
    };
}
