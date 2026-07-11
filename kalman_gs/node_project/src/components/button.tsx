import styles from './button.module.css';

import Tooltip from './tooltip';
import { CSSProperties, ReactNode } from 'react';

type Props = {
  children: ReactNode;
  tooltip?: string;
  onClick?: () => void;
  disabled?: boolean;
  active?: boolean;
  className?: string;
  style?: CSSProperties;
};

export default function Button({ children, tooltip, onClick, disabled, active, className, style }: Props) {
  const buttonClassName =
    styles['button'] +
    (className ? ` ${className}` : '') +
    (active ? ` ${styles['active']}` : '') +
    (disabled ? ` ${styles['disabled']}` : '');

  if (tooltip === undefined || disabled) {
    return (
      <div className={buttonClassName} style={style} onClick={disabled ? undefined : onClick}>
        {children}
      </div>
    );
  } else {
    return (
      <Tooltip text={tooltip} className={buttonClassName} style={style} onClick={disabled ? undefined : onClick}>
        {children}
      </Tooltip>
    );
  }
}
