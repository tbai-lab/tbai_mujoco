# tbai_mujoco

MuJoCo simulator bridge for tbai_sdk.

## Setup

```bash
bash setup.bash
```

This clones `tbai_sdk` and `tbai_mujoco_descriptions` into `thirdparty/`, builds Zenoh, and installs the pixi environment.

If you already have `tbai_sdk` elsewhere, point to it:

```bash
TBAI_SDK_DIR=/path/to/tbai_sdk bash setup.bash
```

## Build

```bash
pixi run build
```

## Examples

First, start the simulator in one terminal:

```bash
pixi run -- ./build/simulate/tbai_mujoco thirdparty/tbai_mujoco_descriptions/robots/go2/config.yaml
```

Then, in a second terminal, run an example:

```bash
# C++
pixi run -- ./build/example/stand_go2 --store_images --store_depth

# Python
pixi run python example/stand_go2.py --store_images --store_depth
```
