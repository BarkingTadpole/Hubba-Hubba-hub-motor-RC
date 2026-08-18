"""Run the freestanding torque-vectoring test ELF under Unicorn RV32."""

from __future__ import annotations

import pathlib
import sys

try:
    from elftools.elf.elffile import ELFFile
    from unicorn import UC_ARCH_RISCV, UC_MODE_RISCV32, Uc, UcError
    from unicorn.riscv_const import (
        UC_RISCV_REG_GP,
        UC_RISCV_REG_PC,
        UC_RISCV_REG_RA,
        UC_RISCV_REG_SP,
    )
except ImportError as error:
    raise SystemExit("install the pyelftools and unicorn Python packages") from error


PAGE_SIZE = 0x1000
STACK_ADDRESS = 0x20000000
STACK_SIZE = 0x10000
RETURN_SENTINEL = 0x30000000


def run(elf_path: pathlib.Path) -> None:
    with elf_path.open("rb") as elf_file:
        elf = ELFFile(elf_file)
        if elf.get_machine_arch() != "RISC-V" or elf.elfclass != 32:
            raise SystemExit(f"expected an RV32 ELF, got {elf.get_machine_arch()} ELF{elf.elfclass}")

        load_segments = [
            segment for segment in elf.iter_segments()
            if segment.header.p_type == "PT_LOAD"
        ]
        symbol_table = elf.get_section_by_name(".symtab")
        if not load_segments or symbol_table is None:
            raise SystemExit("ELF has no load segments or symbol table")

        main_symbols = symbol_table.get_symbol_by_name("main")
        gp_symbols = symbol_table.get_symbol_by_name("__global_pointer$")
        if not main_symbols or not gp_symbols:
            raise SystemExit("ELF is missing main or __global_pointer$")

        first_address = min(segment.header.p_vaddr for segment in load_segments)
        last_address = max(
            segment.header.p_vaddr + segment.header.p_memsz
            for segment in load_segments
        )
        map_start = first_address & ~(PAGE_SIZE - 1)
        map_end = (last_address + PAGE_SIZE - 1) & ~(PAGE_SIZE - 1)

        emulator = Uc(UC_ARCH_RISCV, UC_MODE_RISCV32)
        emulator.mem_map(map_start, map_end - map_start)
        for segment in load_segments:
            emulator.mem_write(segment.header.p_vaddr, segment.data())

        emulator.mem_map(STACK_ADDRESS, STACK_SIZE)
        emulator.mem_map(RETURN_SENTINEL, PAGE_SIZE)
        main_address = main_symbols[0].entry.st_value
        emulator.reg_write(UC_RISCV_REG_PC, main_address)
        emulator.reg_write(UC_RISCV_REG_SP, STACK_ADDRESS + STACK_SIZE - 16)
        emulator.reg_write(UC_RISCV_REG_RA, RETURN_SENTINEL)
        emulator.reg_write(UC_RISCV_REG_GP, gp_symbols[0].entry.st_value)

        try:
            emulator.emu_start(main_address, RETURN_SENTINEL, count=1_000_000)
        except UcError as error:
            pc = emulator.reg_read(UC_RISCV_REG_PC)
            raise SystemExit(f"test trapped before return at {pc:#x}: {error}") from error

        pc = emulator.reg_read(UC_RISCV_REG_PC)
        if pc != RETURN_SENTINEL:
            raise SystemExit(f"test exceeded its instruction budget at {pc:#x}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: run_rv32_unicorn.py <test.elf>")
    run(pathlib.Path(sys.argv[1]))
    print("PASS: RV32 logic tests returned normally under emulation")
