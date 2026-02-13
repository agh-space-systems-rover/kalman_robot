import styles from './navbar.module.css';

import { panelLayouts } from '../common/panel-layouts';
import { modalRef, splashRef } from '../common/refs';
import { IndicatorComponents } from '../indicators';
import ContextMenu from './context-menu';
import Modal from './modal';
import Tooltip from './tooltip';
import { faEdit, faPlus, faTableColumns, faTrash } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useRef, useState } from 'react';

import { defaultPanel } from '../panels';

function sortObject(obj: any) {
  return Object.keys(obj)
    .sort()
    .reduce((result: any, key) => {
      result[key] = obj[key];
      return result;
    }, {});
}

export default function Navbar() {
  const [rerenderCounter, setRerenderCounter] = useState(0);

  const rerenderNavbarAndPanelManager = useCallback(() => {
    setRerenderCounter(rerenderCounter + 1);
    window.dispatchEvent(new Event('rerender-panel-manager'));
  }, [rerenderCounter]);

  return (
    <div className={styles['navbar']}>
      <div className={styles['logo']} onClick={() => splashRef.current?.show()} />
      <div className={styles['layout-selector']}>
        {Object.keys(panelLayouts.layouts).map((layoutName) => {
          return (
            <div
              key={layoutName}
              className={styles['layout-container']}
              onClick={() => {
                if (panelLayouts.currentLayout === layoutName) return;
                panelLayouts.currentLayout = layoutName;
                rerenderNavbarAndPanelManager();
              }}
            >
              <ContextMenu
                items={[
                  {
                    icon: faTrash,
                    text: 'Delete',
                    onClick: () => {
                      if (Object.keys(panelLayouts.layouts).length === 1) {
                        modalRef.current?.showConfirm({
                          title: 'Cannot Delete',
                          icon: faTableColumns,
                          message:
                            'You cannot delete all layouts. Please create a new layout before deleting this one.',
                          confirmText: 'OK',
                          cancelText: 'Close',
                          onConfirm: () => {}
                        });
                        return;
                      }

                      modalRef.current?.showConfirm({
                        title: 'Delete layout',
                        icon: faTableColumns,
                        message: `Are you sure you want to delete "${layoutName}"?`,
                        confirmText: 'Delete',
                        cancelText: 'Cancel',
                        onConfirm: () => {
                          delete panelLayouts.layouts[layoutName];
                          if (panelLayouts.currentLayout === layoutName) {
                            panelLayouts.currentLayout = Object.keys(panelLayouts.layouts)[0];
                          }
                          rerenderNavbarAndPanelManager();
                        }
                      });
                    }
                  },
                  {
                    icon: faEdit,
                    text: 'Rename',
                    onClick: () => {
                      modalRef.current?.showPrompt({
                        title: 'Rename layout',
                        icon: faTableColumns,
                        message: 'Choose a new name for the layout:',
                        placeholder: 'New layout name',
                        defaultValue: layoutName,
                        confirmText: 'Save',
                        onSubmit: (newLayoutName) => {
                          if (!newLayoutName) return;
                          panelLayouts.layouts[newLayoutName] = panelLayouts.layouts[layoutName];
                          delete panelLayouts.layouts[layoutName];
                          if (panelLayouts.currentLayout === layoutName) {
                            panelLayouts.currentLayout = newLayoutName;
                          }
                          panelLayouts.layouts = sortObject(panelLayouts.layouts);
                          rerenderNavbarAndPanelManager();
                        }
                      });
                    }
                  }
                ]}
              >
                {layoutName === panelLayouts.currentLayout ? (
                  <div className={styles['layout'] + ' ' + styles['active']}>{layoutName}</div>
                ) : (
                  <div className={styles['layout']}>{layoutName}</div>
                )}
              </ContextMenu>
            </div>
          );
        })}
        <Tooltip
          text='Create a new layout.'
          className={styles['layout']}
          onClick={() => {
            modalRef.current?.showPrompt({
              title: 'Create new layout',
              icon: faTableColumns,
              message: 'Choose a name for the new layout:',
              placeholder: 'Layout name',
              confirmText: 'Create',
              onSubmit: (layoutName) => {
                if (!layoutName) return;
                panelLayouts.layouts[layoutName] = {
                  panel: defaultPanel
                };
                panelLayouts.currentLayout = layoutName;
                panelLayouts.layouts = sortObject(panelLayouts.layouts);
                rerenderNavbarAndPanelManager();
              }
            });
          }}
        >
          <FontAwesomeIcon icon={faPlus} />
        </Tooltip>
      </div>
      <div className={styles['indicators']}>
        {IndicatorComponents.map((Component, i) => (
          <Component key={i} />
        ))}
      </div>
    </div>
  );
}
