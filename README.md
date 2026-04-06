# tbai_mujoco

Based on the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) repository with tbai-specific functionalities implemeted.

## Setup and run

```bash
bash setup.bash
pixi run build
pixi run -- ./build/simulate/tbai_mujoco thirdparty/tbai_mujoco_descriptions/robots/go2/config.yaml
```