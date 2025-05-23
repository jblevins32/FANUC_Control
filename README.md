## How to run
1. Start robodk from downloaded folder with `./RoboDK-Start.sh`
2. Load `Fanuc-LR-Mate-200iD-7L.robot` in robodk
3. In `server.py` uncomment `RDK.setRunMode(6)` to run on real robot, else comment out for simulation
4. Run `python3 server.py`
5. Run `python3 controller.py`
6. Press start on GUI

## Applying attacks
7. In GUI, select from the dropdown the type of attack to apply
7. In GUI, select `detectable attack` to apply a detectable attack 
8. In GUI, also select `undetectable attack` to make the attack undetectable as visualized in the GUI (must also have `detectable attack` selected).

## To debug
1. Debug server.py with breakpoint
2. Run `python3 controller.py`
3. Press start on GUI
- Same process but vice versa for controller.py

## Current FANUC connection steps:
1. Set PC network settings IPv4 to manual, set netmask to 255.255.255.0, and input one of the addresses listed in the FANUC TCP:
    - 192.168.137.1
    - 192.168.137.2
    - 192.168.137.3
2. Find robot IP via the TCP Menu-Setup-Host Comm-TCP/IP and enter it to robodk connection GUI
    - 192.168.137.20

## How it works
- `server.py` is a server awaiting client request from `controller.py`
... TO DO
