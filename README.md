# NXP drone accompanied by hawk vision (FPV camera on gimbal) with gesture control
### Description 📝
Change your vision from ground to sky. See more, feel more, increase your senses.
### Requirements ✅
* **OS** Ubuntu 20.04.XX LTS or newer
* **Anaconda** enviroment with Python 3.7.x (To run Hawk script (hawk.py) a.k.a. controller)
* **Simulator** PX4 (jMAVSim)
##### INSTALL ALL
pip3 install -r requirements.txt
### Documentation 📘 
[Hackster.io (NXP drone accompanied by HAWK vision)](https://www.hackster.io/spartans/nxp-drone-accompanied-by-hawk-vision-fb33bf)
### Usage 🔨
##### Opening simulator
`cd PX4 && make px4_sitl_default jmavsim`
##### Running HAWK script
`cd build && python3 hawk.py`
### HELP ⚠️
**saso.pavlic@nxp.com**
**peter.vrbancic@nxp.com**