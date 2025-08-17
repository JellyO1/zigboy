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
const Emulator = @import("main.zig").Emulator;
const tracy = @import("tracy");
const ppui = @import("ppu.zig");
const PPU = ppui.PPU;

pub const UI = struct {
    framebufferTexture: *c.SDL_Texture,
    framebuffer: ?*anyopaque = null,
    framebufferPitch: c_int = 0,

    tilesTexture: *c.SDL_Texture,
    tilesBuffer: ?*anyopaque = null,
    tilesPitch: c_int = 0,

    tilemapTexture: *c.SDL_Texture,
    tilemapBuffer: ?*anyopaque = null,
    tilemapPitch: c_int = 0,

    showVRAMViewer: bool = false,

    emulator: *Emulator,
    renderer: *c.SDL_Renderer,

    pub fn init(emulator: *Emulator, renderer: *c.SDL_Renderer) UI {
        const framebufferTexture = c.SDL_CreateTexture(@ptrCast(renderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 160, 144);
        _ = c.SDL_SetTextureScaleMode(framebufferTexture, c.SDL_SCALEMODE_NEAREST);

        const tilesTexture = c.SDL_CreateTexture(@ptrCast(renderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 128, 192);
        _ = c.SDL_SetTextureScaleMode(tilesTexture, c.SDL_SCALEMODE_NEAREST);

        const tilemapTexture = c.SDL_CreateTexture(@ptrCast(renderer), c.SDL_PIXELFORMAT_RGBA8888, c.SDL_TEXTUREACCESS_STREAMING, 256, 256);
        _ = c.SDL_SetTextureScaleMode(tilemapTexture, c.SDL_SCALEMODE_NEAREST);

        return .{
            .framebufferTexture = framebufferTexture,
            .tilesTexture = tilesTexture,
            .tilemapTexture = tilemapTexture,
            .emulator = emulator,
            .renderer = renderer,
        };
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

    fn drawMenuBar(self: *UI) void {
        if (c.ImGui_BeginMainMenuBar()) {
            if (c.ImGui_BeginMenu("File")) {
                c.ImGui_EndMenu();
            }
            if (c.ImGui_BeginMenu("Windows")) {
                _ = c.ImGui_MenuItemBoolPtr("VRAM", null, &self.showVRAMViewer, true);
                c.ImGui_EndMenu();
            }

            c.ImGui_EndMainMenuBar();
        }
    }

    fn drawVRAMViewer(self: *UI, ppu: *PPU, open: *bool) void {
        const S = struct {
            var tilemap: ppui.Tilemap = ppui.Tilemap.T9800;
        };

        if (c.ImGui_Begin("VRAM Viewer", open, c.ImGuiWindowFlags_None)) {
            if (c.ImGui_BeginTabBar("VRAMViewerTabBar", c.ImGuiTabBarFlags_None)) {
                // BG texture
                if (c.ImGui_BeginTabItem("BG", null, c.ImGuiTabItemFlags_None)) {
                    _ = c.SDL_LockTexture(self.tilemapTexture, null, &self.tilemapBuffer, &self.tilemapPitch);
                    ppu.debugTilemap(@ptrCast(@alignCast(self.tilemapBuffer)), S.tilemap);
                    _ = c.SDL_UnlockTexture(self.tilemapTexture);

                    const fit = fitAspect(@floatFromInt(self.tilemapTexture.*.w), @floatFromInt(self.tilemapTexture.*.h));
                    c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(self.tilemapTexture) }, c.ImVec2{ .x = fit.w, .y = fit.h });

                    if (c.ImGui_BeginChild("Options", c.ImVec2{ .x = 0, .y = 0 }, c.ImGuiChildFlags_AlwaysAutoResize | c.ImGuiChildFlags_AutoResizeX | c.ImGuiChildFlags_AutoResizeY, c.ImGuiWindowFlags_None)) {
                        c.ImGui_Text("Map");
                        c.ImGui_SameLine();

                        if (c.ImGui_RadioButton("9800", S.tilemap == ppui.Tilemap.T9800)) {
                            S.tilemap = ppui.Tilemap.T9800;
                        }

                        c.ImGui_SameLine();
                        if (c.ImGui_RadioButton("9C00", S.tilemap == ppui.Tilemap.T9C00)) {
                            S.tilemap = ppui.Tilemap.T9C00;
                        }

                        c.ImGui_EndChild();
                    }

                    c.ImGui_EndTabItem();
                }

                // Tiles texture
                if (c.ImGui_BeginTabItem("Tiles", null, c.ImGuiTabItemFlags_None)) {
                    _ = c.SDL_LockTexture(self.tilesTexture, null, &self.tilesBuffer, &self.tilesPitch);
                    ppu.debugTileset(@ptrCast(@alignCast(self.tilesBuffer)));
                    _ = c.SDL_UnlockTexture(self.tilesTexture);

                    const fit = fitAspect(@floatFromInt(self.tilesTexture.*.w), @floatFromInt(self.tilesTexture.*.h));
                    c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(self.tilesTexture) }, c.ImVec2{ .x = fit.w, .y = fit.h });
                    c.ImGui_EndTabItem();
                }

                c.ImGui_EndTabBar();
            }

            c.ImGui_End();
        }
    }

    fn drawDisplay(self: *UI, ppu: *PPU) void {
        _ = c.SDL_LockTexture(self.framebufferTexture, null, &self.framebuffer, &self.framebufferPitch);
        ppu.drawFramebuffer(@ptrCast(@alignCast(self.framebuffer)));
        _ = c.SDL_UnlockTexture(self.framebufferTexture);

        _ = c.ImGui_Begin("Display", null, c.ImGuiWindowFlags_None);
        const fit = fitAspect(@floatFromInt(self.framebufferTexture.*.w), @floatFromInt(self.framebufferTexture.*.h));
        c.ImGui_Image(c.ImTextureRef{ ._TexID = @intFromPtr(self.framebufferTexture) }, c.ImVec2{ .x = fit.w, .y = fit.h });
        c.ImGui_End();
    }

    pub fn draw(self: *UI) void {
        tracy.frameMark();

        // Prepare for ImGui frame.
        // Start the Dear ImGui frame
        c.cImGui_ImplSDLRenderer3_NewFrame();
        c.cImGui_ImplSDL3_NewFrame();
        c.ImGui_NewFrame();

        const viewport = c.ImGui_GetMainViewport();
        const dockspaceId = c.ImGui_GetID("MyDockSpace");
        _ = c.ImGui_DockSpaceOverViewportEx(dockspaceId, viewport, c.ImGuiDockNodeFlags_None, null);

        if (c.ImGui_DockBuilderGetNode(dockspaceId) == null) {
            const dockIdMain: c.ImGuiID = dockspaceId; // This variable will track the document node, however we are not using it here as we aren't docking anything into it.

            // we now dock our windows into the docking node we made above
            c.ImGui_DockBuilderDockWindow("Display", dockIdMain);
            c.ImGui_DockBuilderFinish(dockspaceId);
        }
        // c.ImGui_End();

        self.drawMenuBar();
        self.drawDisplay(self.emulator.ppu);
        if (self.showVRAMViewer) self.drawVRAMViewer(self.emulator.ppu, &self.showVRAMViewer);

        c.ImGui_Render();
        const drawData: *c.ImDrawData = c.ImGui_GetDrawData();
        const io = c.ImGui_GetIO();
        _ = c.SDL_SetRenderScale(@ptrCast(self.renderer), io.*.DisplayFramebufferScale.x, io.*.DisplayFramebufferScale.y);
        _ = c.SDL_RenderClear(@ptrCast(self.renderer));
        c.cImGui_ImplSDLRenderer3_RenderDrawData(drawData, @ptrCast(self.renderer));
        _ = c.SDL_RenderPresent(@ptrCast(self.renderer));
    }
};
