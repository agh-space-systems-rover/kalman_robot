import { ExtensionContext } from "@foxglove/extension";

import { initCamera2DofPanel } from "./Camera2DofPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Kalman Camera 2DOF",
    initPanel: initCamera2DofPanel,
  });
}
