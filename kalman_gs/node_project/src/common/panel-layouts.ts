import { PanelID, defaultPanel, panelInfos } from '../panels';

// Declare types.
export type LeafPanelLayout = {
  panel: PanelID;
  props?: any;
};

export type HorizontalPanelLayout = {
  division: number;
  left: PanelLayout;
  right: PanelLayout;
};

export type VerticalPanelLayout = {
  division: number;
  top: PanelLayout;
  bottom: PanelLayout;
};

export type PanelLayout =
  | LeafPanelLayout
  | HorizontalPanelLayout
  | VerticalPanelLayout;

export type PanelLayouts = {
  layouts: {
    [name: string]: PanelLayout;
  };
  currentLayout: string;
};

// Initialize the global state.
let panelLayouts: PanelLayouts = {
  layouts: {
    Default: {
      panel: defaultPanel
    }
  },
  currentLayout: 'Default'
};

// Load state from local storage if available.
const savedLayouts = localStorage.getItem('panel-layouts');
if (savedLayouts) {
  panelLayouts = JSON.parse(savedLayouts);
}

// Replace any invalid panel types with the default.
const removeInvalidPanels = (layout: PanelLayout) => {
  if ('panel' in layout) {
    if (!(layout.panel in panelInfos)) {
      layout.panel = defaultPanel;
    }
  } else if ('left' in layout) {
    removeInvalidPanels(layout.left);
    removeInvalidPanels(layout.right);
  } else if ('top' in layout) {
    removeInvalidPanels(layout.top);
    removeInvalidPanels(layout.bottom);
  }
};
for (const layoutName in panelLayouts.layouts) {
  const layout = panelLayouts.layouts[layoutName];
  removeInvalidPanels(layout);
}

// Start saving the layouts every now and then.
setInterval(() => {
  localStorage.setItem('panel-layouts', JSON.stringify(panelLayouts));
}, 100);

export { panelLayouts };
