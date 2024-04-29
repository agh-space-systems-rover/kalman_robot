import styles from './context-menu.module.css';

import Popup from './popup';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { ReactNode, useRef } from 'react';

type Item = {
  icon?: IconDefinition;
  text: string;
  onClick?: () => void;
};

type Props = {
  items: Item[];
  children: ReactNode;
  [key: string]: any;
};

let currentlyOpenMenu: Popup | null = null;

export default function ContextMenu({ items, ...props }: Props) {
  const popupRef = useRef<Popup>(null);
  const hideTimeout = useRef<any>(null);

  const anyItemHasIcon = items.some((item) => item.icon);

  return (
    <Popup
      ref={popupRef}
      margin={0}
      popup={
        <div
          className={styles['context-menu']}
          onMouseEnter={() => {
            clearTimeout(hideTimeout.current);
          }}
          onMouseLeave={() => {
            hideTimeout.current = setTimeout(() => {
              popupRef.current?.hide();
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
      <div
        className={styles['context-menu-trigger']}
        onContextMenu={(e) => {
          if (currentlyOpenMenu) {
            currentlyOpenMenu.hide();
          }

          e.preventDefault();
          popupRef.current?.show();
          clearTimeout(hideTimeout.current);
          currentlyOpenMenu = popupRef.current;
        }}
        onMouseLeave={() => {
          hideTimeout.current = setTimeout(() => {
            popupRef.current?.hide();
          }, 100);
        }}
        onMouseMove={() => {
          clearTimeout(hideTimeout.current);
        }}
      />
      {props.children}
    </Popup>
  );
}
