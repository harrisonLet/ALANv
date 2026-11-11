import serial, random, time, sys
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"   # the ST-LINK VCP; check `ls /dev/ttyACM*`
s = serial.Serial(port, 115200)

import threading
def reader():
    while True:
        data = s.read(s.in_waiting or 1)
        sys.stdout.buffer.write(data); sys.stdout.flush()
threading.Thread(target=reader, daemon=True).start()

n = 0
t0 = time.time()
while True:
    cmd = random.choice([
        f"heading {random.randint(0, 359)}",
        f"heading {random.choice([0, 359, 180])}",      # wrap-around and extreme targets
        f"rudder {random.randint(-90, 90)}",
        f"tail {random.randint(-90, 90)}",
        f"sail {random.randint(-180, 180)}",
        "status", "center", "help",
        "heading 99999", "heading -5", "heading", "xyzzy " + "A" * 80,   # bad input / overlong line
    ])
    s.write((cmd + "\r\n").encode())
    n += 1
    if n % 200 == 0:
        print(f"{n} cmds, {time.time()-t0:.0f}s", flush=True)
    time.sleep(random.choice([0, 0.001, 0.01, 0.05]))     # 0 = as fast as the port allows