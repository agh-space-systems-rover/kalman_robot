import styles from './button.module.css';

import Tooltip from './tooltip';
import { ReactNode } from 'react';

type Props = {
  children: ReactNode;
  tooltip?: string;
  onClick?: () => void;
  disabled?: boolean;
  className?: string;
};

export default function Button({
  children,
  tooltip,
  onClick,
  disabled,
  className
}: Props) {
  if (tooltip === undefined || disabled) {
    return (
      <div
        className={
          styles['button'] +
          (className ? ` ${className}` : '') +
          (disabled ? ` ${styles['disabled']}` : '')
        }
        onClick={disabled ? undefined : onClick}
      >
        {children}
      </div>
    );
  } else {
    return (
      <Tooltip
        text={tooltip}
        className={
          styles['button'] +
          (className ? ` ${className}` : '') +
          (disabled ? ` ${styles['disabled']}` : '')
        }
        onClick={disabled ? undefined : onClick}
      >
        {children}
      </Tooltip>
    );
  }
}
