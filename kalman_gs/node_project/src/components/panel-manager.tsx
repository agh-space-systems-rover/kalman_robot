import styles from './panel-manager.module.css';

import {
  HorizontalPanelLayout,
  LeafPanelLayout,
  PanelLayout,
  VerticalPanelLayout,
  panelLayouts
} from '../common/panel-layouts';
import Button from './button';
import Dropdown from './dropdown';
import {
  faGripLines,
  faGripLinesVertical,
  faXmark
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import {
  useCallback,
  useEffect,
  useState,
  useContext,
  Component,
  createRef
} from 'react';

import { defaultPanel, panelInfos, PanelID } from '../panels';

function clamp(value: number, min: number, max: number) {
  return Math.min(Math.max(value, min), max);
}

export default function PanelManager() {
  const [rerenderCounter, setRerenderCounter] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCounter(rerenderCounter + 1);
  }, [rerenderCounter]);

  // Send panel resize event after every re-render.
  useEffect(() => {
    // Emit panel resize event to force all panels to resize
    window.dispatchEvent(new Event('any-panel-resize'));
  }, [rerenderCounter]);

  // Expose a custom event that allows to re-render the panel manager.
  useEffect(() => {
    window.addEventListener('rerender-panel-manager', rerender);
    return () => {
      window.removeEventListener('rerender-panel-manager', rerender);
    };
  }, [rerender]);

  const renderLayout = useCallback(
    (layout: PanelLayout, path: string[] = []) => {
      // If layout does not have props, set it to an empty object so that panel components can add properties to it.
      if (!layout.hasOwnProperty('props')) {
        (layout as LeafPanelLayout).props = {};
      }

      if (layout.hasOwnProperty('left')) {
        return (
          <div className={styles['horizontal-layout']}>
            <div
              className={styles['layout-container']}
              style={{
                flex: (layout as HorizontalPanelLayout).division
              }}
            >
              {renderLayout((layout as HorizontalPanelLayout).left, [
                ...path,
                'left'
              ])}
            </div>
            <div className={styles['horizontal-divider']}>
              <div
                className={styles['horizontal-divider-handle']}
                onMouseDown={(event) => {
                  const layoutElement =
                    event.currentTarget.parentElement!.parentElement!;
                  const ogX = event.clientX;
                  const ogDiv = (layout as HorizontalPanelLayout).division;

                  document.documentElement.style.cursor = 'col-resize';

                  const mouseMove = (event: MouseEvent) => {
                    let layout =
                      panelLayouts.layouts[panelLayouts.currentLayout];
                    for (const key of path) {
                      layout = (layout as any)[key];
                    }
                    (layout as HorizontalPanelLayout).division = clamp(
                      ogDiv +
                        (event.clientX - ogX) / (layoutElement.clientWidth - 5),
                      0.1,
                      0.9
                    );

                    // -5 to account for the width of the divider
                    // setState(newState);
                    // Set CSS styles directly to avoid rerendering
                    layoutElement.children
                      .item(0)!
                      .setAttribute(
                        'style',
                        `flex: ${(layout as HorizontalPanelLayout).division};`
                      );
                    layoutElement.children
                      .item(2)!
                      .setAttribute(
                        'style',
                        `flex: ${1 - (layout as HorizontalPanelLayout).division};`
                      );

                    // Emit panel resize event to force all panels to resize
                    window.dispatchEvent(new Event('any-panel-resize'));
                  };
                  const mouseUp = () => {
                    document.documentElement.style.cursor = '';
                    document.removeEventListener('mousemove', mouseMove);
                    document.removeEventListener('mouseup', mouseUp);
                  };
                  document.addEventListener('mousemove', mouseMove);
                  document.addEventListener('mouseup', mouseUp);
                }}
              />
            </div>
            <div
              className={styles['layout-container']}
              style={{
                flex: 1 - (layout as HorizontalPanelLayout).division
              }}
            >
              {renderLayout((layout as HorizontalPanelLayout).right, [
                ...path,
                'right'
              ])}
            </div>
          </div>
        );
      } else if (layout.hasOwnProperty('top')) {
        return (
          <div className={styles['vertical-layout']}>
            <div
              className={styles['layout-container']}
              style={{
                flex: (layout as VerticalPanelLayout).division
              }}
            >
              {renderLayout((layout as VerticalPanelLayout).top, [
                ...path,
                'top'
              ])}
            </div>
            <div className={styles['vertical-divider']}>
              <div
                className={styles['vertical-divider-handle']}
                onMouseDown={(event) => {
                  const layoutElement =
                    event.currentTarget.parentElement!.parentElement!;
                  const ogY = event.clientY;
                  const ogDiv = (layout as VerticalPanelLayout).division;

                  document.documentElement.style.cursor = 'row-resize';

                  const mouseMove = (event: MouseEvent) => {
                    let layout =
                      panelLayouts.layouts[panelLayouts.currentLayout];
                    for (const key of path) {
                      layout = (layout as any)[key];
                    }
                    (layout as VerticalPanelLayout).division = clamp(
                      ogDiv +
                        (event.clientY - ogY) /
                          (layoutElement.clientHeight - 5),
                      0.1,
                      0.9
                    );

                    // -5 to account for the width of the divider
                    // setState(newState);
                    // Set CSS styles directly to avoid rerendering
                    layoutElement.children
                      .item(0)!
                      .setAttribute(
                        'style',
                        `flex: ${(layout as VerticalPanelLayout).division};`
                      );
                    layoutElement.children
                      .item(2)!
                      .setAttribute(
                        'style',
                        `flex: ${1 - (layout as VerticalPanelLayout).division};`
                      );

                    // Emit panel resize event to force all panels to resize
                    window.dispatchEvent(new Event('any-panel-resize'));
                  };
                  const mouseUp = () => {
                    document.documentElement.style.cursor = '';
                    document.removeEventListener('mousemove', mouseMove);
                    document.removeEventListener('mouseup', mouseUp);
                  };
                  document.addEventListener('mousemove', mouseMove);
                  document.addEventListener('mouseup', mouseUp);
                }}
              />
            </div>
            <div
              className={styles['layout-container']}
              style={{
                flex: 1 - (layout as VerticalPanelLayout).division
              }}
            >
              {renderLayout((layout as VerticalPanelLayout).bottom, [
                ...path,
                'bottom'
              ])}
            </div>
          </div>
        );
      } else {
        const panel = panelInfos[(layout as LeafPanelLayout).panel];
        const panelRef = createRef(); // TODO: Is this the right way? We can't call useRef in a loop.
        return (
          <>
            <div className={styles['panel-header']}>
              <div className={styles['panel-selector']}>
                <Dropdown
                  key={rerenderCounter}
                  tooltip='Select a panel to replace this one.'
                  items={Object.entries(panelInfos).map(([id, panel]) => ({
                    icon: panel.icon,
                    text: panel.name
                  }))}
                  defaultItemIndex={Object.keys(panelInfos).indexOf(
                    (layout as LeafPanelLayout).panel
                  )}
                  onChange={(i) => {
                    const id = Object.keys(panelInfos)[i];
                    if (path.length === 0) {
                      panelLayouts.layouts[panelLayouts.currentLayout] = {
                        panel: id as PanelID
                      } as LeafPanelLayout;
                    } else {
                      let layout =
                        panelLayouts.layouts[panelLayouts.currentLayout];
                      for (const key of path.slice(0, -1)) {
                        layout = (layout as any)[key];
                      }
                      layout[path[path.length - 1]] = {
                        panel: id as PanelID
                      } as LeafPanelLayout;
                    }
                    rerender();
                  }}
                />
              </div>
              <div className={styles['panel-header-items']}>
                {panel.HeaderComponent && (
                  <panel.HeaderComponent
                    panelRef={panelRef}
                    props={(layout as LeafPanelLayout).props}
                  />
                )}
              </div>
              <div className={styles['panel-controls']}>
                <Button
                  tooltip='Split this panel vertically in half.'
                  onClick={() => {
                    const newLayout: HorizontalPanelLayout = {
                      division: 0.5,
                      left: layout,
                      right: {
                        panel: defaultPanel
                      }
                    };
                    if (path.length === 0) {
                      panelLayouts.layouts[panelLayouts.currentLayout] =
                        newLayout;
                    } else {
                      let layout =
                        panelLayouts.layouts[panelLayouts.currentLayout];
                      for (const key of path.slice(0, -1)) {
                        layout = (layout as any)[key];
                      }
                      (layout as any)[path[path.length - 1]] = newLayout;
                    }
                    rerender();
                  }}
                >
                  <FontAwesomeIcon icon={faGripLinesVertical} />
                </Button>
                <Button
                  tooltip='Split this panel horizontally in half.'
                  onClick={() => {
                    const newLayout: VerticalPanelLayout = {
                      division: 0.5,
                      top: layout,
                      bottom: {
                        panel: defaultPanel
                      }
                    };
                    if (path.length === 0) {
                      panelLayouts.layouts[panelLayouts.currentLayout] =
                        newLayout;
                    } else {
                      let layout =
                        panelLayouts.layouts[panelLayouts.currentLayout];
                      for (const key of path.slice(0, -1)) {
                        layout = (layout as any)[key];
                      }
                      (layout as any)[path[path.length - 1]] = newLayout;
                    }
                    rerender();
                  }}
                >
                  <FontAwesomeIcon icon={faGripLines} />
                </Button>
                <Button
                  tooltip='Close this panel.'
                  onClick={() => {
                    if (path.length === 0) {
                      // Refuse to close the last panel and replace it with a default panel instead
                      panelLayouts.layouts[panelLayouts.currentLayout] = {
                        panel: defaultPanel
                      };
                    } else {
                      // Else set the type of the parent layout to the other panel in pair

                      let opposite = 'left';
                      if (path[path.length - 1] === 'left') {
                        opposite = 'right';
                      } else if (path[path.length - 1] === 'top') {
                        opposite = 'bottom';
                      } else if (path[path.length - 1] === 'bottom') {
                        opposite = 'top';
                      }

                      if (path.length > 1) {
                        let layout =
                          panelLayouts.layouts[panelLayouts.currentLayout];
                        for (const key of path.slice(0, -2)) {
                          layout = (layout as any)[key];
                        }

                        (layout as any)[path[path.length - 2]] = (
                          layout as any
                        )[path[path.length - 2]][opposite];
                      } else {
                        panelLayouts.layouts[panelLayouts.currentLayout] = (
                          panelLayouts.layouts[
                            panelLayouts.currentLayout
                          ] as any
                        )[opposite];
                      }
                    }
                    rerender();
                  }}
                >
                  <FontAwesomeIcon icon={faXmark} />
                </Button>
              </div>
            </div>
            <div className={styles['panel-container']}>
              {/* <panel.Component props={(layout as LeafPanelLayout).props} /> */}
              {/* If component is a class component, assign a ref */}
              {panel.Component.prototype instanceof Component ? (
                <panel.Component
                  ref={panelRef}
                  props={(layout as LeafPanelLayout).props}
                />
              ) : (
                <panel.Component props={(layout as LeafPanelLayout).props} />
              )}
            </div>
          </>
        );
      }
    },
    [rerenderCounter]
  );

  return (
    <div className={styles['panel-manager']}>
      <div
        className={styles['layout-container']}
        key={panelLayouts.currentLayout}
      >
        {renderLayout(panelLayouts.layouts[panelLayouts.currentLayout])}
      </div>
    </div>
  );
}
