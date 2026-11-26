import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import * as THREE from "three";

import { ArmScene, JointMap } from "./scene";

const GROUP = "arm";
const BASE_LINK = "arm_base_link";
const TIP_LINK = "camera_link";
const JOINTS = ["arm_base_yaw_joint", "arm_shoulder_pitch_joint", "arm_wrist_pitch_joint"];
const LABELS = ["Yaw", "Shoulder", "Wrist"];
const IK_PERIOD_MS = 100;

type Preset = { name: string; joints: JointMap };

const CSS = `
.ap{display:flex;flex-direction:column;height:100%;width:100%;font:12px/1.3 system-ui,sans-serif;color:#d8dde3;background:#14181c}
.ap .view{flex:1;min-height:120px;position:relative}
.ap .ctl{padding:5px 8px 6px;gap:4px;background:#1b2026;border-top:1px solid #2a3037;display:flex;flex-direction:column;gap:5px}
.ap .row{display:flex;align-items:center;gap:6px;flex-wrap:wrap}
.ap .row.btns button{flex:1 1 auto}
.ap .row label{width:58px;color:#9aa4ae}.ap .row label.chk{width:auto;white-space:nowrap}
.ap .row .val{width:44px;text-align:right;font-variant-numeric:tabular-nums;color:#9aa4ae}
.ap input[type=range]{flex:1;min-width:0}
.ap select,.ap button{background:#262d35;color:#d8dde3;border:1px solid #38414b;border-radius:4px;padding:3px 7px;font:inherit}
.ap select{flex:1;min-width:0}
.ap button{cursor:pointer}.ap button:hover{background:#303943}
.ap button.go{background:#c96a0a;border-color:#e8861f;color:#fff}.ap button.go:hover{background:#e07a12}
.ap button.stop{background:#8f2323;border-color:#b53030;color:#fff}
.ap button:disabled{opacity:.45;cursor:default}
.ap .status{font-size:11px;color:#9aa4ae;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.ap .status b{color:#ffb15c}.ap .status.bad b{color:#ff6b6b}.ap .status.ok b{color:#6ee09a}
.ap .msg{position:absolute;left:8px;top:6px;color:#9aa4ae;font-size:11px;pointer-events:none}
`;

function sec(t: { sec: number; nanosec?: number; nsec?: number }): number {
  return Number(t.sec) + Number(t.nanosec ?? t.nsec ?? 0) * 1e-9;
}

export function initArmPlannerPanel(context: PanelExtensionContext): () => void {
  const root = context.panelElement;
  root.innerHTML = `<style>${CSS}</style>
  <div class="ap">
    <div class="view"><div class="msg">waiting for /robot_description ...</div></div>
    <div class="ctl">
      <div class="row"><label>Preset</label><select class="preset"></select>
        <button class="cur" title="Set the goal to where the arm is now">Goal = current</button></div>
      ${LABELS.map(
        (l, i) =>
          `<div class="row"><label>${l}</label><input class="sl" data-i="${i}" type="range" step="0.01" min="-1.6" max="1.6" value="0"><span class="val" data-v="${i}">0.00</span></div>`,
      ).join("")}
      <div class="row"><label>Speed</label><input class="vel" type="range" min="0.05" max="1" step="0.05" value="0.5"><span class="val velv">0.50</span></div>
      <div class="row btns">
        <button class="plan">Plan</button><button class="exec">Execute</button>
        <button class="go planexec">Plan &amp; Execute</button><button class="stop">Stop</button>
        <button class="replay" title="Replay the planned trajectory on the ghost">Replay</button>
      </div>
      <div class="row"><div class="status" style="flex:1;min-width:0"></div><label class="chk"><input type="checkbox" class="chassis"> chassis</label></div>
    </div>
  </div>`;
  const $ = <T extends HTMLElement>(sel: string) => root.querySelector(sel) as T;
  const view = $("div.view");
  const msgEl = $("div.msg");
  const presetSel = $<HTMLSelectElement>("select.preset");
  const sliders = Array.from(root.querySelectorAll<HTMLInputElement>("input.sl"));
  const valEls = Array.from(root.querySelectorAll<HTMLElement>("span[data-v]"));
  const velSl = $<HTMLInputElement>("input.vel");
  const statusEl = $("div.status");
  const buttons = {
    plan: $<HTMLButtonElement>("button.plan"),
    exec: $<HTMLButtonElement>("button.exec"),
    planexec: $<HTMLButtonElement>("button.planexec"),
    stop: $<HTMLButtonElement>("button.stop"),
    replay: $<HTMLButtonElement>("button.replay"),
    cur: $<HTMLButtonElement>("button.cur"),
  };

  const arm = new ArmScene(view);
  let presets: Preset[] = [];
  let currentJoints: JointMap = {};
  let haveUrdf = false;
  let rosStatus = "";
  let localNote = "";
  let plannedTrajectory: { t: number; q: number[] }[] | undefined;
  let plannedNames: string[] = [];
  let animation: number | undefined;
  let disposed = false;

  // ------------------------------------------------------------------ status line
  const showStatus = () => {
    const text = localNote || rosStatus || "idle";
    statusEl.innerHTML = `<b>${text.replace(/</g, "&lt;")}</b>`;
    statusEl.className = "status " + (/fail|error|unreach|not available/i.test(text) ? "bad" : /succeed|planned/i.test(text) ? "ok" : "");
    statusEl.setAttribute("title", text);
  };
  const note = (t: string) => {
    localNote = t;
    showStatus();
  };

  // ------------------------------------------------------------------ goal helpers
  const refreshSliders = () => {
    const g = arm.getGoal();
    JOINTS.forEach((j, i) => {
      const lim = arm.limits[j];
      if (lim) {
        sliders[i]!.min = String(lim[0]);
        sliders[i]!.max = String(lim[1]);
      }
      sliders[i]!.value = String(g[j] ?? 0);
      valEls[i]!.textContent = (g[j] ?? 0).toFixed(2);
    });
  };
  const setGoalAndBall = (joints: JointMap) => {
    stopAnimation();
    arm.setGoal(joints);
    arm.resetBallToGoal();
    refreshSliders();
    plannedTrajectory = undefined;
  };

  // ------------------------------------------------------------------ IK while dragging
  let ikBusy = false;
  let ikPending: THREE.Vector3 | undefined;
  let lastIk = 0;
  const runIk = async (target: THREE.Vector3) => {
    if (!context.callService) {
      note("services not available on this data source");
      return;
    }
    if (ikBusy) {
      ikPending = target;
      return;
    }
    ikBusy = true;
    lastIk = performance.now();
    try {
      const g = arm.getGoal();
      const res: any = await context.callService("/compute_ik", {
        ik_request: {
          group_name: GROUP,
          robot_state: { joint_state: { name: JOINTS, position: JOINTS.map((j) => g[j] ?? 0) } },
          ik_link_name: TIP_LINK,
          pose_stamped: {
            header: { frame_id: arm.rootFrame },
            pose: {
              position: { x: target.x, y: target.y, z: target.z },
              orientation: { x: 0, y: 0, z: 0, w: 1 },
            },
          },
          avoid_collisions: false,
          timeout: { sec: 0, nanosec: 80_000_000 },
        },
      });
      if (res?.error_code?.val === 1) {
        const js = res.solution.joint_state;
        const sol: JointMap = {};
        (js.name as string[]).forEach((n, i) => {
          if (JOINTS.includes(n)) {
            sol[n] = (js.position as number[])[i]!;
          }
        });
        arm.setGoal(sol);
        arm.setBallReachable(true);
        refreshSliders();
        plannedTrajectory = undefined;
        if (localNote.startsWith("unreachable")) {
          note("");
        }
      } else {
        arm.setBallReachable(false);
        note("unreachable target (IK failed)");
      }
    } catch (e) {
      note(`IK call failed: ${String(e)}`);
    } finally {
      ikBusy = false;
      if (ikPending) {
        const p = ikPending;
        ikPending = undefined;
        setTimeout(() => void runIk(p), Math.max(0, IK_PERIOD_MS - (performance.now() - lastIk)));
      }
    }
  };
  arm.onBallDragged = (p) => {
    stopAnimation();
    void runIk(p);
  };
  arm.onBallReleased = () => {
    // let the last queued IK finish, then leave the ball where the user put it
  };

  // ------------------------------------------------------------------ plan / execute
  const callTrigger = async (name: string): Promise<{ success: boolean; message: string } | undefined> => {
    if (!context.callService) {
      note("services not available on this data source");
      return undefined;
    }
    try {
      return (await context.callService(name, {})) as any;
    } catch (e) {
      note(`${name} failed: ${String(e)}`);
      return undefined;
    }
  };

  const plan = async (): Promise<boolean> => {
    if (!context.callService) {
      note("services not available on this data source");
      return false;
    }
    const g = arm.getGoal();
    const vel = Number(velSl.value);
    note("planning ...");
    stopAnimation();
    try {
      const res: any = await context.callService("/arm_commander/plan", {
        motion_plan_request: {
          group_name: GROUP,
          goal_constraints: [
            {
              name: "",
              joint_constraints: JOINTS.map((j) => ({
                joint_name: j,
                position: g[j] ?? 0,
                tolerance_above: 0.001,
                tolerance_below: 0.001,
                weight: 1.0,
              })),
            },
          ],
          max_velocity_scaling_factor: vel,
          max_acceleration_scaling_factor: vel,
        },
      });
      const r = res?.motion_plan_response;
      if (r?.error_code?.val !== 1) {
        note(`planning failed (MoveIt error ${r?.error_code?.val})`);
        return false;
      }
      const jt = r.trajectory.joint_trajectory;
      plannedNames = jt.joint_names as string[];
      plannedTrajectory = (jt.points as any[]).map((p) => ({ t: sec(p.time_from_start), q: p.positions as number[] }));
      note("");
      const dur = plannedTrajectory[plannedTrajectory.length - 1]?.t ?? 0;
      localNote = `planned: ${plannedTrajectory.length} points, ${dur.toFixed(1)} s`;
      showStatus();
      replay();
      return true;
    } catch (e) {
      note(`planning failed: ${String(e)}`);
      return false;
    }
  };

  const execute = async () => {
    const r = await callTrigger("/arm_commander/execute");
    if (r && !r.success) {
      note(`cannot execute: ${r.message}`);
    } else if (r) {
      note("");
    }
  };

  function stopAnimation() {
    if (animation !== undefined) {
      cancelAnimationFrame(animation);
      animation = undefined;
    }
  }
  function replay() {
    if (!plannedTrajectory?.length) {
      return;
    }
    stopAnimation();
    const traj = plannedTrajectory;
    const names = plannedNames;
    const total = traj[traj.length - 1]!.t;
    const start = performance.now();
    const step = () => {
      const t = Math.min((performance.now() - start) / 1000, total);
      let i = 0;
      while (i < traj.length - 2 && traj[i + 1]!.t < t) {
        i++;
      }
      const a = traj[i]!;
      const b = traj[Math.min(i + 1, traj.length - 1)]!;
      const f = b.t > a.t ? Math.min(Math.max((t - a.t) / (b.t - a.t), 0), 1) : 1;
      const j: JointMap = {};
      names.forEach((n, k) => (j[n] = a.q[k]! + (b.q[k]! - a.q[k]!) * f));
      arm.setGoal(j);
      if (t < total) {
        animation = requestAnimationFrame(step);
      } else {
        animation = undefined;
        arm.resetBallToGoal();
        refreshSliders();
      }
    };
    animation = requestAnimationFrame(step);
  }

  // ------------------------------------------------------------------ UI wiring
  sliders.forEach((s, i) =>
    s.addEventListener("input", () => {
      const g = arm.getGoal();
      g[JOINTS[i]!] = Number(s.value);
      setGoalAndBall(g);
    }),
  );
  velSl.addEventListener("input", () => ($(".velv").textContent = Number(velSl.value).toFixed(2)));
  presetSel.addEventListener("change", () => {
    const p = presets.find((x) => x.name === presetSel.value);
    if (p) {
      setGoalAndBall({ ...arm.getGoal(), ...p.joints });
    }
  });
  buttons.cur.addEventListener("click", () => setGoalAndBall({ ...currentJoints }));
  buttons.plan.addEventListener("click", () => void plan());
  buttons.exec.addEventListener("click", () => void execute());
  buttons.planexec.addEventListener("click", async () => {
    if (await plan()) {
      await execute();
    }
  });
  buttons.stop.addEventListener("click", async () => {
    const r = await callTrigger("/arm_commander/stop");
    if (r) {
      note(r.success ? "" : r.message);
    }
  });
  buttons.replay.addEventListener("click", replay);
  $<HTMLInputElement>("input.chassis").addEventListener("change", (e) =>
    arm.setChassisVisible((e.target as HTMLInputElement).checked),
  );

  // ------------------------------------------------------------------ data
  const parsePresets = (srdf: string) => {
    const doc = new DOMParser().parseFromString(srdf, "text/xml");
    presets = Array.from(doc.querySelectorAll("group_state"))
      .filter((g) => g.getAttribute("group") === GROUP)
      .map((g) => ({
        name: g.getAttribute("name") ?? "?",
        joints: Object.fromEntries(
          Array.from(g.querySelectorAll("joint")).map((j) => [j.getAttribute("name")!, Number(j.getAttribute("value"))]),
        ),
      }));
    presetSel.innerHTML =
      `<option value="">- preset -</option>` + presets.map((p) => `<option value="${p.name}">${p.name}</option>`).join("");
  };

  context.watch("currentFrame");
  context.subscribe([
    { topic: "/robot_description" },
    { topic: "/robot_description_semantic" },
    { topic: "/joint_states" },
    { topic: "/arm_commander/status" },
  ]);
  context.onRender = (state: Immutable<RenderState>, done) => {
    for (const ev of state.currentFrame ?? []) {
      const m: any = ev.message;
      switch (ev.topic) {
        case "/robot_description":
          if (!haveUrdf && typeof m.data === "string" && m.data.length > 0) {
            try {
              arm.loadUrdf(m.data, BASE_LINK, TIP_LINK, JOINTS);
              haveUrdf = true;
              msgEl.remove();
              refreshSliders();
            } catch (e) {
              msgEl.textContent = `URDF error: ${String(e)}`;
            }
          }
          break;
        case "/robot_description_semantic":
          parsePresets(m.data as string);
          break;
        case "/joint_states": {
          const j: JointMap = {};
          (m.name as string[]).forEach((n, i) => {
            if (JOINTS.includes(n)) {
              j[n] = (m.position as number[])[i]!;
            }
          });
          if (Object.keys(j).length) {
            currentJoints = { ...currentJoints, ...j };
            if (haveUrdf) {
              arm.setCurrent(currentJoints);
            }
          }
          break;
        }
        case "/arm_commander/status":
          rosStatus = m.data as string;
          if (/^(succeeded|failed|executing|idle)/.test(rosStatus)) {
            localNote = "";
          }
          showStatus();
          break;
      }
    }
    done();
  };
  showStatus();

  // test hook for scripted checks (not used by the UI)
  (root as any).__armPlanner = {
    arm,
    setBall: (x: number, y: number, z: number) => {
      arm.ball.position.set(x, y, z);
      arm.onBallDragged?.(arm.ball.position.clone());
    },
    getGoal: () => arm.getGoal(),
    isReachable: () => (arm.ball.material as THREE.MeshStandardMaterial).color.getHex() !== 0xe53935,
  };

  return () => {
    disposed = true;
    stopAnimation();
    arm.dispose();
    void disposed;
  };
}
