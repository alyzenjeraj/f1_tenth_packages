# F1TENTH Project
## Running the Docker Container
> [!NOTE]  
> These steps are to be used on Linux (Ubuntu), and possibly WSL2 on Windows
1. Install [Docker Engine](https://docs.docker.com/engine/install/)
2. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) - Install *and* configure it for Docker *rootless* too
2. Clone this repository
3. `cd` into the root of this repo
4. Run `source ws-aliases.sh`
5. Run `ws-build` to build the image
6. Run `ws-start` to start the image
7. Run `ws-exec` to open an interactive shell into the container


To stop using the shell, run `exit` while inside the container, and `ws-stop` to completely stop and remove the container

Example:
```bash
source ws-aliases.sh
ws-build
ws-start
ws-exec
```