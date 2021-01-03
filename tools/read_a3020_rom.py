import postbox

def main():
    with postbox.Postbox() as pb:
        pb.setup_memc(high_rom_time=1)  # 3 x 12MHz clocks = 250 ns
        return pb.read_memory_to_file(0x3800000, 2 * 1024 * 1024, open("a3020.rom", "wb"))

if __name__ == "__main__":
    main()
