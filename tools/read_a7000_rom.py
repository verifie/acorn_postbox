import postbox


def main():
    with postbox.Postbox() as pb:
        return pb.read_memory_to_file(0, 4 * 1024 * 1024, open("a7000.rom", "wb"))


if __name__ == "__main__":
    main()
