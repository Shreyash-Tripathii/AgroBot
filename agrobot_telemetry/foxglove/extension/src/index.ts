import { ExtensionContext } from "@foxglove/extension";

import { initArmPlannerPanel } from "./panel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Arm Planner", initPanel: initArmPlannerPanel });
}
