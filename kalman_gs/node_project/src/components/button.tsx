import styles from './button.module.css';

import Tooltip from './tooltip';
import { ReactNode } from 'react';

type Props = {
  children: ReactNode;
  tooltip?: string;
  onClick?: () => void;
  className?: string;
};

export default function Button({
  children,
  tooltip,
  onClick,
  className
}: Props) {
  if (tooltip === undefined) {
    return (
      <div
        className={styles['button'] + (className ? ` ${className}` : '')}
        onClick={onClick}
      >
        {children}
      </div>
    );
  } else {
    return (
      <Tooltip
        text={tooltip}
        className={styles['button'] + (className ? ` ${className}` : '')}
        onClick={onClick}
      >
        {children}
      </Tooltip>
    );
  }
}
