# tbai_mujoco

MuJoCo simulator bridge for tbai_sdk.

## Setup

```bash
bash setup.bash
```

This clones `tbai_sdk` (if not found), builds Zenoh, and installs the pixi environment.

If you already have `tbai_sdk` elsewhere, point to it:

```bash
TBAI_SDK_DIR=/path/to/tbai_sdk bash setup.bash
```

## Build

```bash
pixi run build
```

## Run

```bash
pixi run -- ./build/simulate/tbai_mujoco simulate/configs/config_go2.yaml
```

## Examples

```bash
# C++
./build/example/stand_go2 --store_images --store_depth

# Python
pixi run python example/stand_go2.py --store_images
```
