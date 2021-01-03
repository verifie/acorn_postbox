import hashlib
import importlib
import sys
import traceback

def main():
    target_script, = sys.argv[1:]
    if target_script.endswith(".py"): target_script = target_script[:-3]
    print("repeatedly running %s.py" % target_script)
    mod = importlib.import_module(target_script)
    passes = fails = crashes = 0
    n_times = 10
    try:
        for i in range(n_times):
            try:
                rom = mod.main()
                digest = hashlib.sha1(rom).hexdigest()
                if digest in (
                    "68533e0a93657a879c1697c04f32d517345b1a61",
                    "3487729e87bebc9cb51665838c27beff22f7b3bd",
                    "352097bf2d6509e1a008e3658410de9b9e6e8541",
                ):
                    passes += 1
                    print("rom hash OK")
                else:
                    fails += 1
                    print("rom hash BAD")
            except KeyboardInterrupt:
                raise
            except:
                traceback.print_exc()
                crashes += 1
            print("%d ok, %d bad, %d crashed" % (passes, fails, crashes))
    finally:
        print("Ran %s %d times; %d ok, %d bad, %d crashed" % (target_script, n_times, passes, fails, crashes))
        if passes == n_times:
            print("Test passed.")
        else:
            print("*** TEST FAILED ***")
            sys.exit(1)

if __name__ == "__main__":
    main()
