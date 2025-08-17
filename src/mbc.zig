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

    pub fn init(data: []const u8) !CartridgeHeader {
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

    pub fn empty() CartridgeHeader {
        return .{
            .logo = std.mem.zeroes([48]u8),
            .title = std.mem.zeroes([16]u8),
            .manufacturer_code = std.mem.zeroes([4]u8),
            .cgb_only = false,
            .licensee_code = 0,
            .ctype = Type.ROM_ONLY,
            .rom_size = 0,
            .ram_size = 0,
            .dest_code = 0,
            .old_licensee_code = 0,
            .header_checksum = 0,
            .global_checksum = 0,
        };
    }

    /// Gets the ROM size in bytes
    pub fn getROMSize(self: *CartridgeHeader) usize {
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
            // We wan't to panic, this shouldn't be possible and is UB
            else => unreachable,
        };
    }

    /// Gets the number of ROM banks
    pub fn getROMBankCount(self: *CartridgeHeader) usize {
        // Each bank is 16 KiB
        return self.getROMSize() / (16 * kByte);
    }

    /// Gets the RAM size in bytes
    pub fn getRAMSize(self: *CartridgeHeader) usize {
        return switch (self.ram_size) {
            0 => 0,
            2 => 8 * kByte,
            3 => 32 * kByte,
            4 => 128 * kByte,
            5 => 64 * kByte,
            // We wan't to panic, this shouldnt be possible and is UB
            else => unreachable,
        };
    }

    /// Gets the number of RAM banks
    pub fn getRAMBankCount(self: *CartridgeHeader) usize {
        // Each bank is 8 KiB
        return self.getRAMSize() / (8 * kByte);
    }

    // Gets a non null terminated string of the title
    pub fn getTitle(self: *CartridgeHeader) []u8 {
        var len: u8 = 0;

        for (self.title) |c| {
            if (c == 0) break;

            len += 1;
        }

        return self.title[0..len];
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

    game_rom: ?[]const u8,
    external_ram: ?[]u8,

    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, game_rom: ?[]const u8) !MBC {
        var header = CartridgeHeader.empty();
        var external_ram: ?[]u8 = null;

        if (game_rom) |gr| {
            header = try CartridgeHeader.init(gr);

            if (std.fs.cwd().openFile(header.getTitle(), .{})) |file| {
                // Load the previous state before the emulator was closed
                external_ram = try file.readToEndAlloc(allocator, header.getRAMSize());
            } else |_| {
                // Assume there's no previous state even if it's some other error
                external_ram = try allocator.alloc(u8, header.getRAMSize());
            }
        }

        return .{
            .cartridge_header = header,
            .ram_enable = false,
            .rom_bank_number = 1,
            .ram_bank_number = 0,
            .banking_mode = 0,
            .game_rom = game_rom,
            .external_ram = external_ram,
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *MBC) void {
        self.trySave();

        if (self.external_ram != null) self.allocator.free(self.external_ram.?);
    }

    /// If it there's external RAM it saves it to a file with the ROM title
    fn trySave(self: *MBC) void {
        if (self.external_ram == null) return;

        switch (self.cartridge_header.ctype) {
            CartridgeHeader.Type.MBC1_RAM,
            CartridgeHeader.Type.MBC1_RAM_BATTERY,
            CartridgeHeader.Type.MBC3_RAM,
            CartridgeHeader.Type.MBC3_RAM_BATTERY,
            CartridgeHeader.Type.MBC3_TIMER_RAM_BATTERY,
            CartridgeHeader.Type.MBC5_RAM,
            CartridgeHeader.Type.MBC5_RAM_BATTERY,
            CartridgeHeader.Type.MBC5_RUMBLE_RAM,
            CartridgeHeader.Type.MBC5_RUMBLE_RAM_BATTERY,
            CartridgeHeader.Type.MBC7_SENSOR_RUMBLE_RAM_BATTERY,
            CartridgeHeader.Type.MMM01_RAM,
            CartridgeHeader.Type.MMM01_RAM_BATTERY,
            CartridgeHeader.Type.ROM_RAM,
            CartridgeHeader.Type.ROM_RAM_BATTERY,
            CartridgeHeader.Type.HuC1_RAM_BATTERY,
            => {
                if (std.fs.cwd().createFile(self.cartridge_header.getTitle(), .{})) |file| {
                    file.writeAll(self.external_ram.?) catch {};
                } else |_| {}
            },
            // no op
            else => {},
        }
    }

    pub fn read(self: *MBC, addr: u16) u8 {
        return switch (addr) {
            0x0000...0x3FFF => {
                if (self.game_rom == null) return 0xFF;

                // On normal mode this is only the bank 0 and on
                // the case where it's advanced but the ROM bank size
                // is 32 banks (512 KiB) or less it's also always 0
                if (self.banking_mode == 0 or self.cartridge_header.getROMBankCount() <= 32)
                    return self.game_rom.?[addr];

                // If the cartridge is 64 banks (1 MiB)
                if (self.cartridge_header.getROMBankCount() == 64) {
                    // Shifts the ram_bank_number to be the 5th bit. So this is either $00 or $20
                    const bank = ((@as(usize, self.ram_bank_number) & 0b01) << 5);
                    const readAddr: usize = bank * 0x4000 + addr;
                    return self.game_rom.?[readAddr];
                }

                // Otherwise it's 128 banks (2 MiB) or higher
                // Shifts the ram_bank_number to the correct place. So this is either $00, $20, $40 or $60.
                const bank = @as(usize, self.ram_bank_number) << 5;
                const readAddr: usize = bank * 0x4000 + addr;
                return self.game_rom.?[readAddr];
            },
            0x4000...0x7FFF => {
                if (self.game_rom == null) return 0xFF;

                const normAddr = addr - 0x4000;
                const baseAddr = 0x4000 * @as(usize, self.rom_bank_number);
                return self.game_rom.?[baseAddr + normAddr];
            },
            0xA000...0xBFFF => {
                if (!self.ram_enable or self.external_ram == null) return 0xFF;

                const normAddr = addr - 0xA000;

                // If it's 4 banks (8 KiB) or smaller
                if (self.cartridge_header.getRAMBankCount() <= 4) {
                    const readAddr = normAddr % self.cartridge_header.getRAMSize();
                    return self.external_ram.?[readAddr];
                }

                // If it's advanced we're doing bank switching
                if (self.banking_mode == 1) {
                    const readAddr = 0x2000 * @as(usize, self.ram_bank_number) + normAddr;
                    return self.external_ram.?[readAddr];
                }

                // Otherwise we're bank 0
                return self.external_ram.?[normAddr];
            },
            else => 0xFF,
        };
    }

    pub fn write(self: *MBC, addr: u16, value: u8) void {
        switch (addr) {
            0x0000...0x1FFF => self.ram_enable = @as(u4, @truncate(value)) == 0xA,
            0x2000...0x3FFF => {
                self.rom_bank_number = @as(u5, @truncate(value));
                // Set to 1 if it's ever set to 0
                if (self.rom_bank_number == 0) self.rom_bank_number = 1;
            },
            0x4000...0x5FFF => self.ram_bank_number = @as(u2, @truncate(value)),
            0x6000...0x7FFF => self.banking_mode = @as(u1, @truncate(value)),
            0xA000...0xBFFF => {
                if (!self.ram_enable or self.external_ram == null or self.cartridge_header.getRAMSize() == 0) return;

                const normAddr = addr - 0xA000;
                // If it's 1 bank (8 KiB) or smaller
                if (self.cartridge_header.getRAMBankCount() == 1) {
                    const writeAddr = normAddr % self.cartridge_header.getRAMSize();
                    self.external_ram.?[writeAddr] = value;
                    return;
                }

                // If it's advanced we're doing bank switching
                if (self.banking_mode == 1) {
                    const writeAddr = 0x2000 * @as(usize, self.ram_bank_number) + normAddr;
                    self.external_ram.?[writeAddr] = value;
                    return;
                }

                self.external_ram.?[normAddr] = value;
            },
            else => {},
        }
    }

    pub fn printInfo(self: *MBC) void {
        const ramSize = self.cartridge_header.getRAMSize();
        const ramBanks = self.cartridge_header.getRAMBankCount();
        const romSize = self.cartridge_header.getROMSize();
        const romBanks = self.cartridge_header.getROMBankCount();
        std.log.debug("{}, RAM:{} bytes | {} banks, ROM:{} bytes | {} banks", .{ self.cartridge_header.ctype, ramSize, ramBanks, romSize, romBanks });
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
