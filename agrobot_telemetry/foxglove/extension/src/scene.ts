// three.js scene for the arm planner: the current arm (solid), the goal arm (translucent ghost)
// and a draggable ball at the goal end effector.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "three/examples/jsm/controls/TransformControls.js";
import URDFLoader, { URDFRobot } from "urdf-loader";

export type JointMap = Record<string, number>;

const ORANGE = 0xff8c1a;
const RED = 0xe53935;

function armLinkNames(robot: URDFRobot, armBase: string): Set<string> {
  const names = new Set<string>();
  const base = (robot as any).links[armBase];
  if (!base) {
    return names;
  }
  base.traverse((o: any) => {
    if (o.isURDFLink) {
      names.add(o.urdfName);
    }
  });
  return names;
}

function nearestLink(o: THREE.Object3D): string | undefined {
  for (let p: THREE.Object3D | null = o; p; p = p.parent) {
    if ((p as any).isURDFLink) {
      return (p as any).urdfName as string;
    }
  }
  return undefined;
}

export class ArmScene {
  readonly renderer: THREE.WebGLRenderer;
  readonly scene = new THREE.Scene();
  readonly camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
  readonly orbit: OrbitControls;
  readonly transform: TransformControls;
  readonly ball: THREE.Mesh;

  current?: URDFRobot;
  goal?: URDFRobot;
  rootFrame = "base_footprint";
  tipLink = "camera_link";
  baseLink = "arm_base_link";
  jointNames: string[] = [];
  limits: Record<string, [number, number]> = {};

  onBallDragged?: (position: THREE.Vector3) => void;
  onBallReleased?: () => void;

  private frame = 0;
  private resizeObserver: ResizeObserver;
  private chassisVisible = false;
  private chassisMeshes: { mesh: THREE.Mesh; opacity: number }[] = [];
  private dragging = false;

  constructor(private container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.container.appendChild(this.renderer.domElement);
    this.renderer.domElement.style.display = "block";

    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-0.42, -0.5, 0.4);
    this.scene.background = new THREE.Color(0x14181c);

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const sun = new THREE.DirectionalLight(0xffffff, 1.4);
    sun.position.set(1, -1, 2);
    this.scene.add(sun);

    const grid = new THREE.GridHelper(2, 20, 0x3a4048, 0x262b31);
    grid.rotation.x = Math.PI / 2;
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.15));

    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.enableDamping = true;

    this.ball = new THREE.Mesh(
      new THREE.SphereGeometry(0.022, 24, 16),
      new THREE.MeshStandardMaterial({
        color: ORANGE,
        emissive: ORANGE,
        emissiveIntensity: 0.35,
        transparent: true,
        opacity: 0.85,
        depthTest: false,
      }),
    );
    this.ball.renderOrder = 999;
    this.scene.add(this.ball);

    this.transform = new TransformControls(this.camera, this.renderer.domElement);
    this.transform.setMode("translate");
    this.transform.setSize(0.6);
    this.transform.attach(this.ball);
    this.scene.add(this.transform);
    this.transform.addEventListener("dragging-changed", (e: any) => {
      this.orbit.enabled = !e.value;
      this.dragging = !!e.value;
      if (!e.value) {
        this.onBallReleased?.();
      }
    });
    this.transform.addEventListener("objectChange", () => {
      this.onBallDragged?.(this.ball.position.clone());
    });

    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(this.container);
    this.resize();
    const loop = () => {
      this.frame = requestAnimationFrame(loop);
      this.orbit.update();
      this.renderer.render(this.scene, this.camera);
    };
    loop();
  }

  private resize(): void {
    const w = Math.max(this.container.clientWidth, 10);
    const h = Math.max(this.container.clientHeight, 10);
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }

  get isDragging(): boolean {
    return this.dragging;
  }

  /** Build current + goal robots from URDF text. Only the arm links are drawn. */
  loadUrdf(urdf: string, baseLink: string, tipLink: string, jointNames: string[]): void {
    this.baseLink = baseLink;
    this.tipLink = tipLink;
    this.jointNames = jointNames;
    for (const r of [this.current, this.goal]) {
      if (r) {
        this.scene.remove(r);
      }
    }
    this.chassisMeshes = [];
    const build = (ghost: boolean): URDFRobot => {
      const loader = new URDFLoader();
      const robot = loader.parse(urdf);
      const arm = armLinkNames(robot, baseLink);
      const root = robot.children.find((c: any) => c.isURDFLink) as any;
      if (root && !ghost) {
        this.rootFrame = root.urdfName;
      }
      robot.traverse((o: any) => {
        if (!o.isMesh) {
          return;
        }
        const link = nearestLink(o);
        const isArm = link !== undefined && arm.has(link);
        const mats = (Array.isArray(o.material) ? o.material : [o.material]) as THREE.Material[];
        if (ghost) {
          if (!isArm) {
            o.visible = false;
            return;
          }
          o.material = new THREE.MeshStandardMaterial({
            color: ORANGE,
            transparent: true,
            opacity: 0.45,
            depthWrite: false,
          });
        } else if (!isArm) {
          mats.forEach((m) => {
            m.transparent = true;
            m.opacity = 0.18;
          });
          o.visible = this.chassisVisible;
          this.chassisMeshes.push({ mesh: o, opacity: 0.18 });
        }
      });
      this.scene.add(robot);
      return robot;
    };
    this.current = build(false);
    this.goal = build(true);
    this.limits = {};
    for (const n of jointNames) {
      const j = (this.current as any).joints[n];
      if (j) {
        this.limits[n] = [j.limit.lower, j.limit.upper];
      }
    }
    this.setGoal(Object.fromEntries(jointNames.map((n) => [n, 0])));
    this.resetBallToGoal();
    const b = new THREE.Vector3();
    (this.current as any).links[baseLink].getWorldPosition(b);
    this.orbit.target.set(b.x, b.y, b.z + 0.17);
  }

  setChassisVisible(v: boolean): void {
    this.chassisVisible = v;
    this.chassisMeshes.forEach((m) => (m.mesh.visible = v));
  }

  setCurrent(joints: JointMap): void {
    if (this.current) {
      for (const [n, v] of Object.entries(joints)) {
        (this.current as any).joints[n]?.setJointValue(v);
      }
    }
  }

  setGoal(joints: JointMap): void {
    if (this.goal) {
      for (const [n, v] of Object.entries(joints)) {
        (this.goal as any).joints[n]?.setJointValue(v);
      }
      this.goal.updateMatrixWorld(true);
    }
  }

  getGoal(): JointMap {
    const out: JointMap = {};
    for (const n of this.jointNames) {
      const j = (this.goal as any)?.joints[n];
      if (j) {
        out[n] = Number(j.angle);
      }
    }
    return out;
  }

  /** Put the ball at the goal arm's end effector (forward kinematics). */
  resetBallToGoal(): void {
    if (!this.goal) {
      return;
    }
    this.goal.updateMatrixWorld(true);
    const p = new THREE.Vector3();
    (this.goal as any).links[this.tipLink].getWorldPosition(p);
    this.ball.position.copy(p);
    this.setBallReachable(true);
  }

  setBallReachable(ok: boolean): void {
    const m = this.ball.material as THREE.MeshStandardMaterial;
    m.color.setHex(ok ? ORANGE : RED);
    m.emissive.setHex(ok ? ORANGE : RED);
  }

  dispose(): void {
    cancelAnimationFrame(this.frame);
    this.resizeObserver.disconnect();
    this.transform.dispose();
    this.orbit.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}
