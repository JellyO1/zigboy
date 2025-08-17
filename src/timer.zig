const std = @import("std");
const mmuz = @import("mmu.zig");
const MCycle = @import("cpu.zig").MCycle;

const TimerControl = packed struct(u8) {
    // Controls the frequency at which TIMA is incremented
    ClockSelect: u2 = 0,
    // Controls whether TIMA is incremented. Note that DIV is always counting, regardless of this bit.
    Enable: bool = false,
    _: u5 = 0,
};

fn clockSelectThreshold(clockSelect: u2) u32 {
    return switch (clockSelect) {
        0 => 256 * MCycle,
        1 => 4 * MCycle,
        2 => 16 * MCycle,
        3 => 64 * MCycle,
    };
}

pub const Timer = struct {
    // Divider register
    div: *u8,
    // Timer counter
    tima: *u8,
    // Timer modulo
    tma: *u8,
    // Timer control
    tac: *TimerControl,
    // Interrupts flags
    IF: *mmuz.InterruptFlags,

    divCycles: u32,
    timerCycles: u32,

    // Divider increments at a rate of 64 M-cycles
    const divRate: u16 = 64 * MCycle;

    pub fn init(mmu: *mmuz.MMU) Timer {
        return .{
            .div = mmu.readPtr(mmuz.MMU.DIV_ADDR),
            .tima = mmu.readPtr(mmuz.MMU.TIMA_ADDR),
            .tma = mmu.readPtr(mmuz.MMU.TMA_ADDR),
            .tac = @ptrCast(mmu.readPtr(mmuz.MMU.TAC_ADDR)),
            .IF = @ptrCast(mmu.readPtr(mmuz.MMU.IF_ADDR)),
            .timerCycles = 0,
            .divCycles = 0,
        };
    }

    pub fn step(self: *Timer, ticks: u32) void {
        self.divCycles += ticks;

        // Check if we need to increment the DIV register
        while (self.divCycles >= divRate) {
            self.divCycles -= divRate;
            self.div.* +%= 1;
        }

        // Only increment the timer if it's enabled
        if (!self.tac.Enable) {
            return;
        }

        self.timerCycles += ticks;

        const threshold = clockSelectThreshold(self.tac.ClockSelect);

        while (self.timerCycles >= threshold) {
            self.timerCycles -= threshold;

            const res: u8, const overflow: u1 = @addWithOverflow(self.tima.*, 1);
            // The counter overflowed (exceeds $FF)
            if (overflow == 1) {
                // Reset counter to TMA
                self.tima.* = self.tma.*;

                // Request timer interrupt
                self.IF.Timer = true;
            } else {
                self.tima.* = res;
            }
        }
    }
};
