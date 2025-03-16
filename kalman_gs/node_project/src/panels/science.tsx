import styles from './ueuos.module.css';

import React, { useRef, useState } from 'react';

type Props = {
  props: {};
};

window.addEventListener('ros-connect', () => {});

export default function Ueuos({ props }: Props) {
  return <div className={styles['science']}></div>;
}
