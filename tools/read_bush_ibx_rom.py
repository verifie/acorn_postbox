import postbox

def main():
    with postbox.Postbox() as pb:
        # speed up ROM access.  we use the A7000 settings here:
        # - access time 4: 3 MEMCLK cycles, or 187.5 ns
        # - burst time 3: 2 MEMCLK cycles, or 125 ns
        # the best speed I've seen is just over 6500 bytes/s.
        # without speeding things up, we get more like 2000 bytes/s.
        # 6500 bytes/s means 6500 * 9 pulse groups / s = 17 us per pulse group.
        pb.iomd_rom_speed(rom_bank=0, access_time=4, burst_time=3)

        # This appears to work, giving ~7500 bytes/s (~15 us per group),
        # but results in a bad ROM image.
        # pb.iomd_rom_speed(rom_bank=0, access_time=5, burst_time=3)

        return pb.read_memory_to_file(0, 8 * 1024 * 1024, open("bush_ibx.rom", "wb"))

if __name__ == "__main__":
    main()
