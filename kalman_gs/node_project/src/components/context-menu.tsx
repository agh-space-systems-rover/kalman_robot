import styles from './context-menu.module.css';

import Popup from './popup';
import Tooltip from './tooltip';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { ReactNode, useCallback, useRef, useState } from 'react';

type Item = {
  icon?: IconDefinition;
  text: string;
  onClick?: () => void;
};

type Props = {
  items: Item[];
  children: ReactNode;
  openOnClick?: boolean;
  className?: string;
  tooltip?: string;
  [key: string]: any;
};

let currentlyOpenMenu: Popup | null = null;

export default function ContextMenu({
  items,
  children,
  openOnClick = false,
  className,
  tooltip,
  ...props
}: Props) {
  const popupRef = useRef<Popup>(null);
  const hideTimeout = useRef<any>(null);

  const anyItemHasIcon = items.some((item) => item.icon);

  const [open, setOpen] = useState(false);

  const openDropdown = useCallback(
    (e) => {
      if (currentlyOpenMenu) {
        currentlyOpenMenu.hide();
      }

      e.preventDefault();
      popupRef.current?.show();
      setOpen(true);
      clearTimeout(hideTimeout.current);
      currentlyOpenMenu = popupRef.current;
    },
    [setOpen, popupRef, hideTimeout]
  );

  const triggerAttribs = {
    className: styles['context-menu-trigger'],
    onContextMenu: openDropdown,
    onClick: (e) => {
      if (openOnClick) {
        openDropdown(e);
      }
    },
    onMouseLeave: () => {
      hideTimeout.current = setTimeout(() => {
        popupRef.current?.hide();
        setOpen(false);
      }, 100);
    },
    onMouseMove: () => {
      clearTimeout(hideTimeout.current);
    }
  };

  return (
    <Popup
      ref={popupRef}
      margin={0}
      className={styles['popup'] + (className ? ` ${className}` : '')}
      popup={
        <div
          className={styles['context-menu']}
          onMouseEnter={() => {
            clearTimeout(hideTimeout.current);
          }}
          onMouseLeave={() => {
            hideTimeout.current = setTimeout(() => {
              popupRef.current?.hide();
              setOpen(false);
            }, 100);
          }}
          onClick={(e) => {
            e.stopPropagation();
            // Stops the parent from receiving any clicks targeted at the context menu.
          }}
        >
          {items.map((item, i) => {
            return (
              <div
                key={i}
                className={styles['item']}
                onClick={() => {
                  item.onClick?.();
                  popupRef.current?.hide();
                  setOpen(false);
                }}
              >
                {anyItemHasIcon && (
                  <div className={styles['item-icon']}>
                    {item.icon && <FontAwesomeIcon icon={item.icon} />}
                  </div>
                )}
                <div className={styles['item-text']}>{item.text}</div>
              </div>
            );
          })}
        </div>
      }
      {...props}
    >
      {tooltip && !open ? (
        <Tooltip text={tooltip} {...triggerAttribs} />
      ) : (
        <div {...triggerAttribs} />
      )}
      {children}
    </Popup>
  );
}
