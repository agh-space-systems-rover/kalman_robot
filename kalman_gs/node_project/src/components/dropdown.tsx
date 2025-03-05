import styles from './dropdown.module.css';

import Tooltip from './tooltip';
import { IconDefinition, faChevronDown } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { ReactNode, useCallback, useEffect, useState } from 'react';

type Item = {
  icon?: IconDefinition;
  text: string;
  value?: any;
};

type Props = {
  tooltip?: string;
  items: Item[];
  defaultItemIndex?: number;
  onChange?: (index: number, value?: any) => void;
  className?: string;
  popupClassName?: string;
  [key: string]: any;
};

export default function Dropdown({
  tooltip,
  items,
  defaultItemIndex = 0,
  onChange,
  className,
  popupClassName,
  ...props
}: Props) {
  const [open, setOpen] = useState(false);
  const [selectedItemIndex, setSelectedItemIndex] = useState(defaultItemIndex);

  const openDropdown = useCallback(() => {
    setOpen(true);
    if (window) {
      window.addEventListener('keyup', closeDropdown);
    }
  }, []);

  const closeDropdown = useCallback(() => {
    setOpen(false);
    if (window) {
      window.removeEventListener('keyup', closeDropdown);
    }
  }, []);

  useEffect(() => {
    return () => {
      if (window) {
        window.removeEventListener('keyup', closeDropdown);
      }
    };
  }, []);

  return (
    <Tooltip
      className={styles['dropdown'] + (className ? ` ${className}` : '')}
      text={tooltip}
      onClick={() => {
        if (open) {
          closeDropdown();
        } else {
          openDropdown();
        }
      }}
      {...props}
    >
      {items[selectedItemIndex] && (
        <>
          {items[selectedItemIndex].icon && (
            <FontAwesomeIcon className={styles['item-icon']} icon={items[selectedItemIndex].icon} />
          )}
          <div className={styles['item-text']}>{items[selectedItemIndex].text}</div>
        </>
      )}
      <FontAwesomeIcon icon={faChevronDown} className={styles['item-icon']} />
      {open && (
        <>
          <div className={styles['dropdown-popup'] + (popupClassName ? ` ${popupClassName}` : '')}>
            {items.map((item, index) => (
              <div
                key={index}
                className={styles['dropdown-item']}
                onClick={() => {
                  onChange?.(index, item.value);
                  closeDropdown();
                  setSelectedItemIndex(index);
                }}
              >
                {item.icon && <FontAwesomeIcon className={styles['item-icon']} icon={item.icon} />}
                <div className={styles['item-text']}>{item.text}</div>
              </div>
            ))}
          </div>
          <div className={styles['dropdown-popup-background']} onClick={closeDropdown} />
        </>
      )}
    </Tooltip>
  );
}
