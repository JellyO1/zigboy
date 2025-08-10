const std = @import("std");
const mmuz = @import("mmu.zig");

const kByte = 1024;
const mByte = 1024 * kByte;

const CartridgeHeader = struct {
    const Type = enum(u8) {
        ROM_ONLY,
        MBC1,
        MBC1_RAM,
        MBC1_RAM_BATTERY,
        MBC2,
        MBC2_BATTERY,
        ROM_RAM,
        ROM_RAM_BATTERY,
        MMM01,
        MMM01_RAM,
        MMM01_RAM_BATTERY,
        MBC3_TIMER_BATTERY,
        MBC3_TIMER_RAM_BATTERY,
        MBC3,
        MBC3_RAM,
        MBC3_RAM_BATTERY,
        MBC5,
        MBC5_RAM,
        MBC5_RAM_BATTERY,
        MBC5_RUMBLE,
        MBC5_RUMBLE_RAM,
        MBC5_RUMBLE_RAM_BATTERY,
        MBC6,
        MBC7_SENSOR_RUMBLE_RAM_BATTERY,
        POCKET_CAMERA,
        BANDAI_TAMA5,
        HuC3,
        HuC1_RAM_BATTERY,
    };

    logo: [48]u8,
    title: [16]u8,
    manufacturer_code: [4]u8,
    cgb_only: bool,
    licensee_code: u16,
    ctype: Type,
    rom_size: u8,
    ram_size: u8,

    /// Specifies whether this version of the game is intended to be sold in Japan or elsewhere.
    ///
    /// `0` Japan (and possibly overseas); `1` Overseas only.
    dest_code: u8,

    /// Used in older (pre-SGB) cartridges to specify the game’s publisher.
    /// However, the value $33 indicates that the New licensee codes must be considered instead.
    old_licensee_code: u8,

    /// This byte contains an 8-bit checksum computed from the cartridge header bytes $0134–014C.
    ///
    /// The boot ROM verifies this checksum.
    /// If the byte at $014D does not match the lower 8 bits of checksum,
    /// the boot ROM will lock up and the program in the cartridge won’t run.
    header_checksum: u8,

    /// These bytes contain a 16-bit (big-endian) checksum simply computed as the sum of all the
    /// bytes of the cartridge ROM (except these two checksum bytes).
    ///
    /// This checksum is not verified, except by Pokémon Stadium’s “GB Tower” emulator
    /// (presumably to detect Transfer Pak errors).
    global_checksum: u16,

    pub const SizeError = error{
        SizeUnused,
    };

    pub fn init(data: []u8) !CartridgeHeader {
        return .{
            .logo = data[0x104..0x134].*,
            .title = data[0x134..0x144].*,
            .manufacturer_code = data[0x13F..0x143].*,
            .cgb_only = data[0x143] == 0xC0,
            .licensee_code = @as(u16, data[0x144]) << 8 | @as(u16, data[0x145]),
            .ctype = @enumFromInt(data[0x147]),
            .rom_size = data[0x148],
            .ram_size = data[0x149],
            .dest_code = data[0x14A],
            .old_licensee_code = data[0x14B],
            .header_checksum = data[0x14D],
            .global_checksum = @as(u16, data[0x14E]) << 8 | @as(u16, data[0x14F]),
        };
    }

    /// Gets the ROM size in bytes
    pub fn getROMSize(self: *CartridgeHeader) error{SizeError}!usize {
        return switch (self.rom_size) {
            0 => 32 * kByte,
            1 => 64 * kByte,
            2 => 128 * kByte,
            3 => 256 * kByte,
            4 => 512 * kByte,
            5 => mByte,
            6 => 2 * mByte,
            7 => 4 * mByte,
            8 => 8 * mByte,
            else => SizeError.SizeUnused,
        };
    }

    /// Gets the RAM size in bytes
    pub fn getRAMSize(self: *CartridgeHeader) error{SizeError}!usize {
        return switch (self.ram_size) {
            0 => 0,
            2 => 8 * kByte,
            3 => 32 * kByte,
            4 => 128 * kByte,
            5 => 64 * kByte,
            else => SizeError.SizeUnused,
        };
    }
};

pub const MBC = struct {
    cartridge_header: CartridgeHeader,

    /// Any write with a value of $A in the lower nibble
    /// to the range 0000-1FFF enables the ram.
    ram_enable: bool,

    /// Any write to the range 2000-3FFF
    /// selects the ROM bank for the region 4000-7FFF.
    /// If $00 is written to this, it defaults to $01
    rom_bank_number: u5,

    /// Any write to the range 4000-5FFF
    /// selects the RAM bank number or the upper two bits (5-6) of ROM
    /// If neither RAM nor ROM is large enough this does nothing.
    ram_bank_number: u2,

    /// Any write to the range 6000-7FFF
    /// selects the banking mode.
    ///
    /// `0` is simple (default): 0000-3FFF and A000-BFFF are locked to
    /// bank 0 of ROM and SRAM
    ///
    /// `1` is advanced: 0000-3FFF and A000-BFFF can be bank switched via
    /// the ram_bank_number register.
    banking_mode: u1,

    game_rom: []u8,

    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, game_rom_path: []const u8) !MBC {
        const game_rom = try std.fs.cwd().readFileAlloc(allocator, game_rom_path, mByte * 8);

        return .{
            .cartridge_header = try CartridgeHeader.init(game_rom),
            .ram_enable = false,
            .rom_bank_number = 1,
            .ram_bank_number = 0,
            .banking_mode = 0,
            .game_rom = game_rom,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *MBC) void {
        self.allocator.free(self.game_rom);
    }

    pub fn read(self: *MBC, addr: u16) u8 {
        // NOTE: We're ignoring the banking mode right now
        return switch (addr) {
            0x0000...0x3FFF => self.game_rom[addr],
            0x4000...0x7FFF => {
                const normAddr = addr - 0x4000;
                const baseAddr = 0x4000 * @as(usize, self.rom_bank_number);
                return self.game_rom[baseAddr + normAddr];
            },
            else => 0xFF,
        };
    }

    pub fn write(self: *MBC, addr: u16, value: u8) void {
        switch (addr) {
            0x0000...0x1FFF => self.ram_enable = @as(u4, @truncate(value)) == 0xA,
            0x2000...0x3FFF => self.rom_bank_number = @as(u5, @truncate(value)) | 0x01,
            0x4000...0x5FFF => self.ram_bank_number = @as(u2, @truncate(value)),
            0x6000...0x7FFF => self.banking_mode = @as(u1, @truncate(value)),
            else => {},
        }
    }

    pub fn printInfo(self: *MBC) void {
        std.log.debug("{}", .{self.cartridge_header.ctype});
    }

    pub fn dumpLogo(self: *MBC) !void {
        const logoFile = try std.fs.cwd().createFile("logo.dump", std.fs.File.CreateFlags{ .read = true });

        const logo = try std.fmt.allocPrint(
            self.allocator,
            "{s}",
            .{self.cartridge_header.logo},
        );
        defer self.allocator.free(logo);
        try logoFile.writeAll(logo);
    }
};
