#!/usr/bin/env python3
"""交互式 MuJoCo 演示 —— 核手套箱灵巧手抓取放置。

运行后会弹出 MuJoCo 窗口，手会自动循环执行：
    下探 → 合拢(触觉确认) → 抬起 → 搬运 → 降入 → 松手 → 撤离
任务完成后自动复位、重新开始。

鼠标交互：
    · 左键拖动      旋转视角
    · 右键拖动      平移视角
    · 滚轮          缩放
    · 双击物体      选中
    · Ctrl+右键拖动 对选中物体施加力（可以把瓶子拽走捣乱，看手会不会受影响）

用法：
    source ../../venv/bin/activate
    python run_interactive.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco
import mujoco.viewer
import numpy as np

from src.grasp_controller import GraspController

PHASE_CN = {
    "approach":      "① 接近 —— 五指手盲触下探到样品瓶",
    "close":         "② 合拢 —— 五指+拇指包络抓取（触觉确认抓牢）",
    "lift":          "③ 抬起 —— 提起样品瓶",
    "reorient":      "④ 手内重定向 —— 握持中绕轴旋转样品",
    "reorient_back": "⑤ 复位 —— 旋回到搬运姿态",
    "carry":         "⑥ 搬运 —— 横移到屏蔽罐上方",
    "lower":         "⑦ 降入 —— 下降到罐内",
    "release":       "⑧ 松手 —— 五指张开，样品落入罐中",
    "retract":       "⑨ 撤离 —— 空手抬起",
    "done":          "✅ 完成 —— 即将复位重来",
}


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    model = mujoco.MjModel.from_xml_path(os.path.join(here, "mjcf", "glovebox_scene.xml"))
    data = mujoco.MjData(model)
    ctrl = GraspController(model, data)

    print("=" * 64)
    print("🤖 核手套箱灵巧手 —— 交互式抓取放置演示")
    print("=" * 64)
    print("Shadow 五指灵巧手在'无视觉'核手套箱中，靠指尖触觉盲抓样品瓶，")
    print("手内重定向后放入屏蔽罐。手会自动循环演示。")
    print("鼠标：左键转视角 / 滚轮缩放 / 双击选中 / Ctrl+右键拖动施力捣乱")
    print("关闭窗口或 Ctrl+C 退出。\n")

    dt = model.opt.timestep
    done_hold = 0.0
    last_phase = None

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 一个能看清手和瓶子的视角
        viewer.cam.lookat[:] = [0.16, 0.0, 0.18]
        viewer.cam.distance = 1.05
        viewer.cam.azimuth = 140
        viewer.cam.elevation = -18
        # 显示接触点与接触力，让"触觉"可见
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True

        while viewer.is_running():
            step_start = time.time()

            ctrl.step(dt)
            mujoco.mj_step(model, data)

            name, touch = ctrl.status()
            if name != last_phase:
                print(f"  {PHASE_CN.get(name, name):<34}  触觉力(N)={np.round(touch,1)}")
                last_phase = name

            # 完成后停留 1.5s 再复位重来，形成循环演示
            if ctrl.done:
                done_hold += dt
                if done_hold > 1.5:
                    mujoco.mj_resetData(model, data)
                    ctrl.reset()
                    done_hold = 0.0
                    last_phase = None
                    print("  —— 复位，重新开始 ——\n")

            viewer.sync()

            # 尽量贴近实时
            remain = dt - (time.time() - step_start)
            if remain > 0:
                time.sleep(remain)

    print("\n👋 演示结束")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 已退出")
