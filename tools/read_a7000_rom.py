import postbox

with postbox.Postbox() as pb:
    pb.read_memory_to_file(0, 4 * 1024 * 1024, open("a7000.rom", "wb"))
