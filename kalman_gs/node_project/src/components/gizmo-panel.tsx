import React, { useMemo, useRef, useLayoutEffect, useState } from 'react';
import styles from './gizmo-panel.module.css';

export type Gizmo = { x: number; y: number; color: string; label?: string };
type Domain = { minX: number; maxX: number; minY: number; maxY: number };

type Props = {
    width?: number;            // desired css size (can be overridden by layout)
    height?: number;
    gizmos: Gizmo[] | string;
    selectedIndex: number | null;
    onSelect: (index: number) => void;

    domain?: Domain;           // fixed world domain (else auto)
    fitMode?: 'auto' | 'domain';
    yAxis?: 'down' | 'up';
    padding?: number;          // px inside panel
    buttonSize?: number;       // px
    className?: string;
};

const EPS = 1e-9;

export default function GizmoPanel({
    width = 320,
    height = 220,
    gizmos,
    selectedIndex,
    onSelect,
    domain,
    fitMode = 'auto',
    yAxis = 'down',
    padding = 10,
    buttonSize = 44,
    className = '',
}: Props) {
    const items: Gizmo[] = useMemo(() => {
        if (typeof gizmos === 'string') {
            try { return JSON.parse(gizmos) as Gizmo[]; } catch { return []; }
        }
        return gizmos;
    }, [gizmos]);

    // Measure the REAL rendered size so math always matches what CSS/layout did
    const rootRef = useRef<HTMLDivElement>(null);
    const [size, setSize] = useState<{ w: number; h: number } | null>(null);

    useLayoutEffect(() => {
        const el = rootRef.current;
        if (!el) return;
        const ro = new ResizeObserver(([entry]) => {
            const cr = entry.contentRect;
            setSize({ w: cr.width, h: cr.height });
        });
        ro.observe(el);
        return () => ro.disconnect();
    }, []);

    const W = size?.w ?? width;     // actual pixels we have
    const H = size?.h ?? height;

    // inner drawable area (after padding)
    const innerW = Math.max(0, W - 2 * padding);
    const innerH = Math.max(0, H - 2 * padding);

    // world-domain (auto-fit or fixed)
    const world = useMemo<Domain>(() => {
        if (fitMode === 'domain' && domain) return domain;

        if (items.length === 0) return { minX: 0, maxX: 1, minY: 0, maxY: 1 };

        let minX = +Infinity, maxX = -Infinity, minY = +Infinity, maxY = -Infinity;
        for (const g of items) {
            if (Number.isFinite(g.x)) { minX = Math.min(minX, g.x); maxX = Math.max(maxX, g.x); }
            if (Number.isFinite(g.y)) { minY = Math.min(minY, g.y); maxY = Math.max(maxY, g.y); }
        }
        if (!isFinite(minX) || !isFinite(minY)) return { minX: 0, maxX: 1, minY: 0, maxY: 1 };

        // 5% breathing room
        const spanX0 = Math.max(maxX - minX, EPS);
        const spanY0 = Math.max(maxY - minY, EPS);
        const padX0 = 0.05 * spanX0;
        const padY0 = 0.05 * spanY0;

        // first-pass scale (ignore button radius)
        const scale0 = Math.max(
            EPS,
            Math.min(innerW / (spanX0 + 2 * padX0), innerH / (spanY0 + 2 * padY0))
        );

        // expand by button radius in world units so edges never clip
        const rPx = buttonSize / 2;
        const mWorld = rPx / Math.max(scale0, EPS);

        return {
            minX: minX - padX0 - mWorld,
            maxX: maxX + padX0 + mWorld,
            minY: minY - padY0 - mWorld,
            maxY: maxY + padY0 + mWorld,
        };
    }, [items, fitMode, domain, innerW, innerH, buttonSize]);

    // world -> overlay pixels
    const spanX = Math.max(world.maxX - world.minX, EPS);
    const spanY = Math.max(world.maxY - world.minY, EPS);
    const scale = Math.min(innerW / spanX, innerH / spanY);
    const extraX = (innerW - scale * spanX) / 2;
    const extraY = (innerH - scale * spanY) / 2;
    const offsetX = padding + extraX - scale * world.minX;
    const offsetY = (yAxis === 'down')
        ? padding + extraY - scale * world.minY
        : padding + extraY + scale * world.maxY;

    const project = (x: number, y: number) => ({
        cx: scale * x + offsetX,
        cy: yAxis === 'down' ? scale * y + offsetY : -scale * y + offsetY,
    });

    // clamp inside overlay for belt-and-suspenders safety
    const radius = buttonSize / 2;
    const minXpx = padding + radius, maxXpx = W - padding - radius;
    const minYpx = padding + radius, maxYpx = H - padding - radius;

    return (
        <div
            ref={rootRef}
            className={`${styles.container} ${className}`}
            style={{
                width, height,
                flex: '0 0 auto',       // don’t let parent flex stretch this unless you want it to
            }}
            role="group"
            aria-label="Panel Gizmo Selector"
        >
            {/* background */}
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

            {/* overlay spans the whole container; padding is handled in math */}
            <div className={styles.overlay}>
                {items.map((g, i) => {
                    const p = project(g.x, g.y);
                    const left = Math.max(minXpx, Math.min(maxXpx, p.cx));
                    const top = Math.max(minYpx, Math.min(maxYpx, p.cy));
                    const isSel = i === selectedIndex;
                    return (
                        <button
                            key={i}
                            type="button"
                            className={`${styles.gizmoBtn} ${isSel ? styles.selected : ''}`}
                            style={{ left, top, width: buttonSize, height: buttonSize, background: g.color }}
                            aria-pressed={isSel}
                            title={g.label ?? `x=${g.x}, y=${g.y}`}
                            onClick={() => onSelect(i)}
                        />
                    );
                })}
            </div>
        </div>
    );
}
