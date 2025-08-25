import styles from './button.module.css';

import Tooltip from './tooltip';
import {CSSProperties, ReactNode} from 'react';

type Props = {
  children: ReactNode;
  tooltip?: string;
  onClick?: () => void;
  disabled?: boolean;
  className?: string;
  style?: CSSProperties;
};

export default function Button({
  children,
  tooltip,
  onClick,
  disabled,
  className,
  style
}: Props) {
  if (tooltip === undefined || disabled) {
    return (
      <div
        className={
          styles['button'] +
          (className ? ` ${className}` : '') +
          (disabled ? ` ${styles['disabled']}` : '')
        }
        style={style}
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
        style={style}
        onClick={disabled ? undefined : onClick}
      >
        {children}
      </Tooltip>
    );
  }
}
