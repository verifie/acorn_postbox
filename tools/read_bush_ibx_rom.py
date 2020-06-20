import postbox

with postbox.Postbox() as pb:
    pb.read_memory_to_file(0, 8 * 1024 * 1024, open("bush_ibx.rom", "wb"))
