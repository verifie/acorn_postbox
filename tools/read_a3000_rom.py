import postbox

with postbox.Postbox() as pb:
    pb.setup_memc(high_rom_time=2)  # 2 x 8MHz clocks = 250 ns
    pb.read_memory_to_file(0x3800000, 2 * 1024 * 1024, open("a3000.rom", "wb"))
