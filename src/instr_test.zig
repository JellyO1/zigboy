const main = @import("main.zig");
const cpu = @import("cpu.zig");
const std = @import("std");

const TestFormat = struct {
    name: []const u8,
    initial: struct {
        pc: u16,
        sp: u16,
        a: u8,
        b: u8,
        c: u8,
        d: u8,
        e: u8,
        f: u8,
        h: u8,
        l: u8,
        ime: u1,
        ie: u1,
        ram: []struct { u16, u8 },
    },
    final: struct {
        pc: u16,
        sp: u16,
        a: u8,
        b: u8,
        c: u8,
        d: u8,
        e: u8,
        f: u8,
        h: u8,
        l: u8,
        ime: u1,
        ei: ?u1 = 0,
        ram: []struct { u16, u8 },
    },
    cycles: []struct { u16, u16, []const u8 },
};

pub fn runSingleTest(allocator: std.mem.Allocator, filename: []const u8) !void {
    const path = try std.fmt.allocPrint(allocator, "src/single_step_tests/{s}.json", .{filename});
    defer allocator.free(path);

    const file = try std.fs.cwd().openFile(path, .{});
    defer file.close();

    const content = try file.readToEndAlloc(allocator, std.math.maxInt(usize));
    defer allocator.free(content);

    const json = try std.json.parseFromSlice([]TestFormat, allocator, content, .{
        .ignore_unknown_fields = true,
    });
    defer json.deinit();

    for (json.value) |value| {
        // std.log.warn("{s}\n", .{value.name});

        var state = main.GameBoyState.initTest(.{
            .A = value.initial.a,
            .B = value.initial.b,
            .C = value.initial.c,
            .D = value.initial.d,
            .E = value.initial.e,
            .H = value.initial.h,
            .L = value.initial.l,
            .F = cpu.Flags.init(.{ .C = (value.initial.f >> 4 & 1) != 0, .H = (value.initial.f >> 5 & 1) != 0, .N = (value.initial.f >> 6 & 1) != 0, .Z = (value.initial.f >> 7 & 1) != 0 }),
            .PC = value.initial.pc,
            .SP = value.initial.sp,
        }, value.initial.ime == 1, false);

        const isDebug = std.mem.eql(u8, value.name, "AB 02A6");

        if (isDebug) {
            std.log.warn("before: {any}\n", .{state.cpu.registers});
        }

        for (value.initial.ram) |ram| {
            if (isDebug) {
                std.log.warn("0x{X:4} {d}\n", .{ ram[0], ram[1] });
            }
            state.mmu.write(ram[0], ram[1]);
        }

        state.step();

        if (isDebug) {
            std.log.warn("after: {any}\n", .{state.cpu.registers});
        }

        try std.testing.expectEqual(value.final.pc, state.cpu.registers.PC);
        try std.testing.expectEqual(value.final.sp, state.cpu.registers.SP);
        try std.testing.expectEqual(value.final.a, state.cpu.registers.A);
        try std.testing.expectEqual(value.final.b, state.cpu.registers.B);
        try std.testing.expectEqual(value.final.c, state.cpu.registers.C);
        try std.testing.expectEqual(value.final.d, state.cpu.registers.D);
        try std.testing.expectEqual(value.final.e, state.cpu.registers.E);
        try std.testing.expectEqual(value.final.h, state.cpu.registers.H);
        try std.testing.expectEqual(value.final.l, state.cpu.registers.L);
        try std.testing.expectEqual(value.final.ime == 1, state.cpu.ime);
        try std.testing.expectEqual(value.final.ei == 1, state.cpu.ei_delay);

        for (value.final.ram) |ram| {
            if (isDebug) {
                std.log.warn("expected:{d} actual:{d} addr: 0x{X:4}\n", .{ ram[1], state.mmu.read(ram[0]), ram[0] });
            }
            try std.testing.expectEqual(ram[1], state.mmu.read(ram[0]));
        }
    }
}

test "single step tests" {
    const allocator = std.testing.allocator;

    const single_instr = [244][]const u8{
        "00", "0a", "0b", "0c", "0d", "0e", "0f",
        "01", "1a", "1b", "1c", "1d", "1e", "1f",
        "02", "2a", "2b", "2c", "2d", "2e", "2f",
        "03", "3a", "3b", "3c", "3d", "3e", "3f",
        "04", "4a", "4b", "4c", "4d", "4e", "4f",
        "05", "5a", "5b", "5c", "5d", "5e", "5f",
        "06", "6a", "6b", "6c", "6d", "6e", "6f",
        "07", "7a", "7b", "7c", "7d", "7e", "7f",
        "08", "8a", "8b", "8c", "8d", "8e", "8f",
        "09", "9a", "9b", "9c", "9d", "9e", "9f",
        "10", "11", "12", "13", "14", "15", "16",
        "17", "18", "19", "20", "21", "22", "23",
        "24", "25", "26", "27", "28", "29", "30",
        "31", "32", "33", "34", "35", "36", "37",
        "38", "39", "40", "41", "42", "43", "44",
        "45", "46", "47", "48", "49", "50", "51",
        "52", "53", "54", "55", "56", "57", "58",
        "59", "60", "61", "62", "63", "64", "65",
        "66", "67", "68", "69", "70", "71", "72",
        "73", "74", "75", "76", "77", "78", "79",
        "80", "81", "82", "83", "84", "85", "86",
        "87", "88", "89", "90", "91", "92", "93",
        "94", "95", "96", "97", "98", "99", "a0",
        "a1", "a2", "a3", "a4", "a5", "a6", "a7",
        "a8", "a9", "aa", "ab", "ac", "ad", "ae",
        "af", "b0", "b1", "b2", "b3", "b4", "b5",
        "b6", "b7", "b8", "b9", "ba", "bb", "bc",
        "bd", "be", "bf", "c0", "c1", "c2", "c3",
        "c4", "c5", "c6", "c7", "c8", "c9", "ca",
        "cc", "cd", "ce", "cf", "d0", "d1", "d2",
        "d4", "d5", "d6", "d7", "d8", "d9", "da",
        "dc", "de", "df", "e0", "e1", "e2", "e5",
        "e6", "e7", "e8", "e9", "ea", "ee", "ef",
        "f0", "f1", "f2", "f3", "f5", "f6", "f7",
        "f8", "f9", "fa", "fb", "fe", "ff",
    };

    for (single_instr) |file| {
        try runSingleTest(allocator, file);
    }

    const cb_files = [256][]const u8{
        "cb0a", "cb0b", "cb0c", "cb0d", "cb0e", "cb0f",
        "cb1a", "cb1b", "cb1c", "cb1d", "cb1e", "cb1f",
        "cb2a", "cb2b", "cb2c", "cb2d", "cb2e", "cb2f",
        "cb3a", "cb3b", "cb3c", "cb3d", "cb3e", "cb3f",
        "cb4a", "cb4b", "cb4c", "cb4d", "cb4e", "cb4f",
        "cb5a", "cb5b", "cb5c", "cb5d", "cb5e", "cb5f",
        "cb6a", "cb6b", "cb6c", "cb6d", "cb6e", "cb6f",
        "cb7a", "cb7b", "cb7c", "cb7d", "cb7e", "cb7f",
        "cb8a", "cb8b", "cb8c", "cb8d", "cb8e", "cb8f",
        "cb9a", "cb9b", "cb9c", "cb9d", "cb9e", "cb9f",
        "cba0", "cbaa", "cbab", "cbac", "cbad", "cbae",
        "cbaf", "cbb0", "cbba", "cbbb", "cbbc", "cbbd",
        "cbbe", "cbbf", "cbc0", "cbca", "cbcb", "cbcc",
        "cbcd", "cbce", "cbcf", "cbd0", "cbda", "cbdb",
        "cbdc", "cbdd", "cbde", "cbdf", "cbe0", "cbea",
        "cbeb", "cbec", "cbed", "cbee", "cbef", "cbf0",
        "cbfa", "cbfb", "cbfc", "cbfd", "cbfe", "cbff",
        "cba1", "cba2", "cba3", "cba4", "cba5", "cba6",
        "cba7", "cba8", "cba9", "cbb1", "cbb2", "cbb3",
        "cbb4", "cbb5", "cbb6", "cbb7", "cbb8", "cbb9",
        "cbc1", "cbc2", "cbc3", "cbc4", "cbc5", "cbc6",
        "cbc7", "cbc8", "cbc9", "cbd1", "cbd2", "cbd3",
        "cbd4", "cbd5", "cbd6", "cbd7", "cbd8", "cbd9",
        "cbe1", "cbe2", "cbe3", "cbe4", "cbe5", "cbe6",
        "cbe7", "cbe8", "cbe9", "cbf1", "cbf2", "cbf3",
        "cbf4", "cbf5", "cbf6", "cbf7", "cbf8", "cbf9",
        "cb00", "cb01", "cb02", "cb03", "cb04", "cb05",
        "cb06", "cb07", "cb08", "cb09", "cb10", "cb11",
        "cb12", "cb13", "cb14", "cb15", "cb16", "cb17",
        "cb18", "cb19", "cb20", "cb21", "cb22", "cb23",
        "cb24", "cb25", "cb26", "cb27", "cb28", "cb29",
        "cb30", "cb31", "cb32", "cb33", "cb34", "cb35",
        "cb36", "cb37", "cb38", "cb39", "cb40", "cb41",
        "cb42", "cb43", "cb44", "cb45", "cb46", "cb47",
        "cb48", "cb49", "cb50", "cb51", "cb52", "cb53",
        "cb54", "cb55", "cb56", "cb57", "cb58", "cb59",
        "cb60", "cb61", "cb62", "cb63", "cb64", "cb65",
        "cb66", "cb67", "cb68", "cb69", "cb70", "cb71",
        "cb72", "cb73", "cb74", "cb75", "cb76", "cb77",
        "cb78", "cb79", "cb80", "cb81", "cb82", "cb83",
        "cb84", "cb85", "cb86", "cb87", "cb88", "cb89",
        "cb90", "cb91", "cb92", "cb93", "cb94", "cb95",
        "cb96", "cb97", "cb98", "cb99",
    };

    for (cb_files) |file| {
        try runSingleTest(allocator, file);
    }
}

// comptime {
//     const files: [244][]const u8 = .{
//         "00", "0a", "0b", "0c", "0d", "0e", "0f",
//         "01", "1a", "1b", "1c", "1d", "1e", "1f",
//         "02", "2a", "2b", "2c", "2d", "2e", "2f",
//         "03", "3a", "3b", "3c", "3d", "3e", "3f",
//         "04", "4a", "4b", "4c", "4d", "4e", "4f",
//         "05", "5a", "5b", "5c", "5d", "5e", "5f",
//         "06", "6a", "6b", "6c", "6d", "6e", "6f",
//         "07", "7a", "7b", "7c", "7d", "7e", "7f",
//         "08", "8a", "8b", "8c", "8d", "8e", "8f",
//         "09", "9a", "9b", "9c", "9d", "9e", "9f",
//         "10", "11", "12", "13", "14", "15", "16",
//         "17", "18", "19", "20", "21", "22", "23",
//         "24", "25", "26", "27", "28", "29", "30",
//         "31", "32", "33", "34", "35", "36", "37",
//         "38", "39", "40", "41", "42", "43", "44",
//         "45", "46", "47", "48", "49", "50", "51",
//         "52", "53", "54", "55", "56", "57", "58",
//         "59", "60", "61", "62", "63", "64", "65",
//         "66", "67", "68", "69", "70", "71", "72",
//         "73", "74", "75", "76", "77", "78", "79",
//         "80", "81", "82", "83", "84", "85", "86",
//         "87", "88", "89", "90", "91", "92", "93",
//         "94", "95", "96", "97", "98", "99", "a0",
//         "a1", "a2", "a3", "a4", "a5", "a6", "a7",
//         "a8", "a9", "aa", "ab", "ac", "ad", "ae",
//         "af", "b0", "b1", "b2", "b3", "b4", "b5",
//         "b6", "b7", "b8", "b9", "ba", "bb", "bc",
//         "bd", "be", "bf", "c0", "c1", "c2", "c3",
//         "c4", "c5", "c6", "c7", "c8", "c9", "ca",
//         "cc", "cd", "ce", "cf", "d0", "d1", "d2",
//         "d4", "d5", "d6", "d7", "d8", "d9", "da",
//         "dc", "de", "df", "e0", "e1", "e2", "e5",
//         "e6", "e7", "e8", "e9", "ea", "ee", "ef",
//         "f0", "f1", "f2", "f3", "f5", "f6", "f7",
//         "f8", "f9", "fa", "fb", "fe", "ff",
//     };

//     for (files) |file| {
//         const path = "single_step_tests/" ++ file ++ ".json";
//         const content = @embedFile(path);

//         // Generate a test function for this file
//         _ = std.fmt.comptimePrint("single_step_{s}", .{file[0..2]});

//         _ = struct {
//             test {
//                 const json = try std.json.parseFromSlice([]TestFormat, std.testing.allocator, content, .{
//                     .ignore_unknown_fields = true,
//                 });
//                 defer json.deinit();

//                 for (json.value) |value| {
//                     std.log.warn("{s}\n", .{value.name});

//                     var state = main.GameBoyState.initTest(.{
//                         .A = value.initial.a,
//                         .B = value.initial.b,
//                         .C = value.initial.c,
//                         .D = value.initial.d,
//                         .E = value.initial.e,
//                         .H = value.initial.h,
//                         .L = value.initial.l,
//                         .F = main.Flags.init(.{ .C = (value.initial.f >> 4 & 1) != 0, .H = (value.initial.f >> 5 & 1) != 0, .N = (value.initial.f >> 6 & 1) != 0, .Z = (value.initial.f >> 7 & 1) != 0 }),
//                         .PC = value.initial.pc,
//                         .SP = value.initial.sp,
//                     }, value.initial.ime != 0, false);

//                     const isDebug = std.mem.eql(u8, value.name, "F5 0000");

//                     if (isDebug) {
//                         std.log.warn("before: {any}\n", .{state.registers});
//                     }

//                     for (value.initial.ram) |ram| {
//                         if (isDebug) {
//                             std.log.warn("{d} {d}\n", .{ ram[0], ram[1] });
//                         }
//                         state.memory.write(ram[0], ram[1]);
//                     }

//                     main.step(&state);

//                     if (isDebug) {
//                         std.log.warn("after: {any}\n", .{state.registers});
//                     }

//                     try std.testing.expectEqual(value.final.pc, state.registers.PC);
//                     try std.testing.expectEqual(value.final.sp, state.registers.SP);
//                     try std.testing.expectEqual(value.final.a, state.registers.A);
//                     try std.testing.expectEqual(value.final.b, state.registers.B);
//                     try std.testing.expectEqual(value.final.c, state.registers.C);
//                     try std.testing.expectEqual(value.final.d, state.registers.D);
//                     try std.testing.expectEqual(value.final.e, state.registers.E);
//                     try std.testing.expectEqual(value.final.h, state.registers.H);
//                     try std.testing.expectEqual(value.final.l, state.registers.L);
//                     try std.testing.expectEqual(value.final.ime != 0, state.ime);
//                     try std.testing.expectEqual(value.final.ei != 0, state.ei_delay);

//                     for (value.final.ram) |ram| {
//                         if (isDebug) {
//                             std.log.warn("expected:{d} actual:{d} addr:{d}\n", .{ ram[1], state.memory.read(ram[0]), ram[0] });
//                         }
//                         try std.testing.expectEqual(ram[1], state.memory.read(ram[0]));
//                     }
//                 }
//             }
//         };
//     }
// }
