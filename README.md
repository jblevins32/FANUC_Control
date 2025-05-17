## How to run
1. Start robodk from downloaded folder with `./RoboDK-Start.sh`
2. Load `Fanuc-LR-Mate-200iD-7L.robot` in robodk
3. In `server.py` uncomment `RDK.setRunMode(6)` to run on real robot, else comment out for simulation
4. Run `python3 attacker.py`
5. Run `python3 controller.py`
6. Press start on GUI

## To debug
1. Debug server.py with breakpoint
2. Run `python3 controller.py`
3. Press start on GUI

## How it works
- `server.py` is a server awaiting client request from `controller.py`
...