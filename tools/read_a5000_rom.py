import postbox

def main():
    with postbox.Postbox() as pb:
        pb.setup_memc(high_rom_time=postbox.MEMC_ROM_SPEED_3N_3S)
        return pb.read_memory_to_file(0x3800000, 2 * 1024 * 1024, open("a5000.rom", "wb"))

if __name__ == "__main__":
    main()
