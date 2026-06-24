#!/usr/bin/env python3
"""无头自检：不弹窗、不渲染视频，跑完整抓取放置流程并打印关键指标。

判定标准：样品瓶被指尖触觉确认抓牢、抬起、放入屏蔽罐。
退出码 0 = 通过，1 = 失败，便于 CI / 复现核对。

用法：
    source ../../venv/bin/activate
    python validate.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mujoco
import numpy as np

from src.grasp_controller import GraspController


def run() -> bool:
    here = os.path.dirname(os.path.abspath(__file__))
    model = mujoco.MjModel.from_xml_path(os.path.join(here, "mjcf", "glovebox_scene.xml"))
    data = mujoco.MjData(model)
    ctrl = GraspController(model, data)

    print(f"nu(actuators)={model.nu}  nsensor={model.nsensor}")

    dt = model.opt.timestep
    peak = np.zeros(5)
    max_lift = 0.0
    confirmed = False
    while not ctrl.done:
        ctrl.step(dt)
        mujoco.mj_step(model, data)
        peak = np.maximum(peak, ctrl.touch_forces())
        if ctrl.status()[0] == "close" and ctrl.grasp_secure():
            confirmed = True
        max_lift = max(max_lift, data.body("sample_vial").xpos[2])

    v = data.body("sample_vial").xpos
    placed = abs(v[0] - 0.32) < 0.07 and abs(v[1]) < 0.09 and v[2] < 0.09

    print(f"peak touch (N) FF MF RF LF TH = {np.round(peak, 2)}")
    print(f"max lift height (m) = {max_lift:.3f}")
    print(f"final vial pos = {np.round(v, 3)}")
    print(f"grasp tactilely confirmed: {confirmed}")
    print(f"placed in container: {placed}")

    ok = confirmed and max_lift > 0.18 and placed
    print("\n" + ("✅ 自检通过" if ok else "❌ 自检失败"))
    return ok


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
