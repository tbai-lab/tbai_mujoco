# tbai_mujoco

Based on the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) repository with tbai-specific functionalities implemeted.

<img width="1438" height="997" alt="image" src="https://github.com/user-attachments/assets/6859e017-d215-4532-87e0-9a49a0ed1dfc" />


## Setup and run

```bash
bash setup.bash
pixi run build
pixi run -- ./build/simulate/tbai_mujoco thirdparty/tbai_mujoco_descriptions/robots/go2/config.yaml
```
