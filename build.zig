const std = @import("std");
const cimgui = @import("cimgui_zig");

// Although this function looks imperative, note that its job is to
// declaratively construct a build graph that will be executed by an external
// runner.
pub fn build(b: *std.Build) void {
    // Standard target options allows the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});

    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});

    // Add a build option for tracy
    const enable_tracy = b.option(bool, "enable_tracy", "Enable Tracy profiler integration") orelse if (optimize == .Debug) true else false;

    // Only add tracy if enabled
    const tracy_dep = b.dependency("tracy", .{
        .target = target,
        .optimize = optimize,
        .tracy_enable = enable_tracy,
    });
    const tracy_mod = tracy_dep.module("tracy");

    const cimgui_dep = b.dependency("cimgui_zig", .{
        .target = target,
        .optimize = optimize,
        .platform = cimgui.Platform.SDL3,
        .renderer = cimgui.Renderer.SDL3Renderer,
    });

    const clap = b.dependency("clap", .{});

    // We will also create a module for our other entry point, 'main.zig'.
    const exe_mod = b.createModule(.{
        // `root_source_file` is the Zig "entry point" of the module. If a module
        // only contains e.g. external object files, you can make this `null`.
        // In this case the main source file is merely a path, however, in more
        // complicated build scripts, this could be a generated file.
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .imports = &.{
            .{ .name = "tracy", .module = tracy_mod },
        },
    });

    // This creates another `std.Build.Step.Compile`, but this one builds an executable
    // rather than a static library.
    const exe = b.addExecutable(.{
        .name = "zig_gbc",
        .root_module = exe_mod,
    });
    exe.linkLibC();
    exe.linkLibrary(cimgui_dep.artifact("cimgui"));
    exe.root_module.addImport("clap", clap.module("clap"));

    if (enable_tracy) {
        exe.linkLibrary(tracy_dep.artifact("tracy"));
        exe.linkLibCpp();
    }

    // Link additional system deps if we're on linux

    if (std.Target.isGnuLibC(target.result)) {
        // Add Wayland libraries (for Wayland backend)
        // exe.linkSystemLibrary("wayland-client");
        // exe.linkSystemLibrary("wayland-egl");
        // exe.linkSystemLibrary("wayland-cursor");
        // exe.linkSystemLibrary("xkbcommon");

        // Add X11 libraries (for X11 backend fallback)
        exe.linkSystemLibrary("X11");
    }

    // This declares intent for the executable to be installed into the
    // standard location when the user invokes the "install" step (the default
    // step when running `zig build`).
    b.installArtifact(exe);

    // This *creates* a Run step in the build graph, to be executed when another
    // step is evaluated that depends on it. The next line below will establish
    // such a dependency.
    const run_cmd = b.addRunArtifact(exe);

    // By making the run step depend on the install step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    // This is not necessary, however, if the application depends on other installed
    // files, this ensures they will be present and in the expected location.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    // This creates a build step. It will be visible in the `zig build --help` menu,
    // and can be selected like this: `zig build run`
    // This will evaluate the `run` step rather than the default, which is "install".
    const run_step = b.step("run", "Run the app");
    run_step.dependOn(&run_cmd.step);

    const exe_unit_tests = b.addTest(.{
        .root_module = exe_mod,
    });

    const run_exe_unit_tests = b.addRunArtifact(exe_unit_tests);

    const instr_unit_tests = b.addTest(.{
        .root_source_file = b.path("src/instr_test.zig"),
    });

    const run_instr_unit_tests = b.addRunArtifact(instr_unit_tests);

    // Similar to creating the run step earlier, this exposes a `test` step to
    // the `zig build --help` menu, providing a way for the user to request
    // running the unit tests.
    const test_step = b.step("test", "Run unit tests");
    test_step.dependOn(&run_exe_unit_tests.step);
    test_step.dependOn(&run_instr_unit_tests.step);
}
