import React, { useMemo } from 'react';
import styles from './gizmo-panel.module.css';

export type Gizmo = { x: number; y: number; color: string; label?: string };

type Props = {
  width?: number;          // px
  height?: number;         // px
  gizmos: Gizmo[] | string; // array or JSON string
  selectedIndex: number | null;
  onSelect: (index: number) => void;
  className?: string;
};

export default function GizmoPanel({
  width = 320,
  height = 220,
  gizmos,
  selectedIndex,
  onSelect,
  className = '',
}: Props) {
  // Accept JSON string OR array.
  const items: Gizmo[] = useMemo(() => {
    if (typeof gizmos === 'string') {
      try { return JSON.parse(gizmos) as Gizmo[]; }
      catch { return []; }
    }
    return gizmos;
  }, [gizmos]);

  return (
    <div
      className={`${styles.container} ${className}`}
      style={{ width, height }}
      role="group"
      aria-label="Panel Gizmo Selector"
    >
      {/* Rounded background w/ subtle grid */}
      <svg className={styles.back} viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden>
        <defs>
          <linearGradient id="g_bg" x1="0" y1="0" x2="1" y2="1">
            <stop offset="0" stopColor="#0e0f12" />
            <stop offset="1" stopColor="#171a1f" />
          </linearGradient>
          <pattern id="g_grid" width="10" height="10" patternUnits="userSpaceOnUse">
            <path d="M 10 0 L 0 0 0 10" fill="none" stroke="#2a2f39" strokeWidth="0.5" />
          </pattern>
        </defs>
        <rect x="0.5" y="0.5" width="99" height="99" rx="12" ry="12" fill="url(#g_bg)" stroke="#2a2f39" />
        <rect x="0.5" y="0.5" width="99" height="99" rx="12" ry="12" fill="url(#g_grid)" opacity="0.35" />
      </svg>

      {/* Absolutely-positioned HTML buttons */}
      <div className={styles.overlay}>
        {items.map((g, i) => {
          const left = `${Math.max(0, Math.min(1, g.x)) * 100}%`;
          const top  = `${Math.max(0, Math.min(1, g.y)) * 100}%`;
          const isSel = i === selectedIndex;
          return (
            <button
              key={i}
              type="button"
              className={`${styles.gizmoBtn} ${isSel ? styles.selected : ''}`}
              style={{ left, top, background: g.color }}
              aria-pressed={isSel}
              title={g.label ?? `x=${g.x.toFixed(2)}, y=${g.y.toFixed(2)}`}
              onClick={() => onSelect(i)}
            >
              {g.label ? <span className={styles.label}>{g.label}</span> : null}
            </button>
          );
        })}
      </div>
    </div>
  );
}
