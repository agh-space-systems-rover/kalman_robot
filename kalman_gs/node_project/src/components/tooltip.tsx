import styles from './tooltip.module.css';

import Popup from './popup';
import { ReactNode, useRef } from 'react';

type Props = {
  text: string;
  children?: ReactNode;
  className?: string;
  [key: string]: any;
};

export default function Tooltip({
  text,
  children,
  className,
  ...props
}: Props) {
  const showTimeout = useRef<any>(null);
  const popupRef = useRef<Popup>(null);

  // Replace \n with <br> for newlines
  let multilineText: ReactNode;
  if (text.includes('\\n')) {
    multilineText = (
      <>
        {text.split('\\n').map((line, i) => (
          <span key={i}>
            {line}
            <br />
          </span>
        ))}
      </>
    );
  } else {
    multilineText = text;
  }

  return (
    <Popup
      ref={popupRef}
      popup={<div className={styles['tooltip-popup']}>{multilineText}</div>}
      className={
        styles['tooltip-container'] + (className ? ` ${className}` : '')
      }
      {...props}
    >
      <div
        className={styles['tooltip-anchor']}
        onMouseEnter={() => {
          showTimeout.current = setTimeout(() => {
            popupRef.current?.show();
          }, 500);
        }}
        onMouseLeave={() => {
          clearTimeout(showTimeout.current);
          popupRef.current?.hide();
        }}
      />
      {children}
    </Popup>
  );
}
