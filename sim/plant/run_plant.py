#!/usr/bin/env python3
"""
Boat physics simulator for the ALAN virtual board.

  python3 sim/plant/run_plant.py                                  wait for Renode, default scenario
  python3 sim/plant/run_plant.py --scenario sim/plant/scenarios/gusty_beam_reach.json
  python3 sim/plant/run_plant.py --standalone 60                  physics only, no Renode

While it runs, type commands (wind 300 4, gps off, status, help...) and press Enter.
"""
import argparse
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from alan_plant.server import HELP, Simulation, load_scenario, run_standalone, serve  # noqa: E402


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", help="scenario JSON file (see sim/plant/scenarios)")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--standalone", type=float, metavar="SECONDS", help="run without Renode for this many seconds")
    ap.add_argument("--fast", action="store_true", help="standalone: run as fast as possible instead of real time")
    ap.add_argument("--log-dir", default=os.path.join(here, "logs"))
    ap.add_argument("--no-console", action="store_true", help="do not read commands from the terminal")
    args = ap.parse_args()

    sim = Simulation(load_scenario(args.scenario), args.log_dir)

    if not args.no_console and sys.stdin.isatty():
        def console():
            print(HELP)
            for line in sys.stdin:
                with sim.lock:
                    sim.command(line)
                if not sim.running:
                    break
        threading.Thread(target=console, daemon=True).start()

    try:
        if args.standalone:
            run_standalone(sim, args.standalone, realtime=not args.fast)
        else:
            serve(sim, port=args.port)
    except KeyboardInterrupt:
        pass
    finally:
        sim.close()
        print(f"Log saved: {sim.log_path}")


if __name__ == "__main__":
    main()
