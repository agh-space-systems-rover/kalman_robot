import styles from './label.module.css';

import Tooltip from './tooltip';
import { ReactNode } from 'react';

type Props = {
  children: ReactNode;
  tooltip?: string;
  color?: string;
  className?: string;
  [key: string]: any;
};

export default function Label({
  children,
  tooltip,
  color = 'red',
  className,
  style,
  ...props
}: Props) {
  if (tooltip === undefined) {
    return (
      <div
        className={styles['label'] + (className ? ` ${className}` : '')}
        style={{ backgroundColor: color, ...style }}
        {...props}
      >
        {children}
      </div>
    );
  } else {
    return (
      <Tooltip
        text={tooltip}
        className={
          styles['label'] +
          ' ' +
          styles['help'] +
          (className ? ` ${className}` : '')
        }
        style={{ backgroundColor: color }}
        {...props}
      >
        {children}
      </Tooltip>
    );
  }
}
