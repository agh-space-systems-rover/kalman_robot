import styles from './popup.module.css';

import { Component, ReactNode, createRef } from 'react';

type Props = {
  margin?: number;
  popup?: ReactNode;
  children?: ReactNode;
  className?: string;
  [key: string]: any;
};

export default class Popup extends Component<Props> {
  private anchorRef = createRef<HTMLDivElement>();
  private popupRef = createRef<HTMLDivElement>();

  private updatePopupPosition = () => {
    const anchorMargin =
      this.props.margin === undefined ? 10 : this.props.margin;

    if (!this.anchorRef.current) return;
    if (!this.popupRef.current) return;

    const anchor = this.anchorRef.current;
    const popup = this.popupRef.current;

    const anchorBounds = anchor.getBoundingClientRect();
    const popupBounds = popup.getBoundingClientRect();
    const { innerWidth, innerHeight } = window;

    const desiredPopupPosition = {
      left: anchorBounds.left + anchorBounds.width / 2 - popupBounds.width / 2,
      top: anchorBounds.bottom + anchorMargin
    };

    const anchorOverlapsPopup = (popupPos: any) => {
      return (
        anchorBounds.left - anchorMargin < popupPos.left + popupBounds.width &&
        anchorBounds.right + anchorMargin > popupPos.left &&
        anchorBounds.top - anchorMargin < popupPos.top + popupBounds.height &&
        anchorBounds.bottom + anchorMargin > popupPos.top
      );
    };

    // Tune desired position so that the popup does not go outside the window.
    // Tune until the popup is inside the window and until it does not overlap with the anchor.
    if (desiredPopupPosition.left < 10) {
      desiredPopupPosition.left = 10;
      if (anchorOverlapsPopup(desiredPopupPosition)) {
        desiredPopupPosition.left = anchorBounds.right + anchorMargin;
      }
    } else if (
      desiredPopupPosition.left + popupBounds.width >
      innerWidth - 10
    ) {
      desiredPopupPosition.left = innerWidth - 10 - popupBounds.width;
      if (anchorOverlapsPopup(desiredPopupPosition)) {
        desiredPopupPosition.left =
          anchorBounds.left - anchorMargin - popupBounds.width;
      }
    }

    if (desiredPopupPosition.top < 10) {
      desiredPopupPosition.top = 10;
      if (anchorOverlapsPopup(desiredPopupPosition)) {
        desiredPopupPosition.top = anchorBounds.bottom + anchorMargin;
      }
    } else if (
      desiredPopupPosition.top + popupBounds.height >
      innerHeight - 10
    ) {
      desiredPopupPosition.top = innerHeight - 10 - popupBounds.height;
      if (anchorOverlapsPopup(desiredPopupPosition)) {
        desiredPopupPosition.top =
          anchorBounds.top - anchorMargin - popupBounds.height;
      }
    }

    // Apply the tuned position.
    popup.style.left = desiredPopupPosition.left + 'px';
    popup.style.top = desiredPopupPosition.top + 'px';
  };

  show() {
    if (!this.popupRef.current) return;
    this.popupRef.current.classList.add(styles['visible']);
    this.updatePopupPosition();
    window.addEventListener('resize', this.updatePopupPosition);
  }

  hide() {
    window.removeEventListener('resize', this.updatePopupPosition);
    if (!this.popupRef.current) return;
    this.popupRef.current.classList.remove(styles['visible']);
  }

  render() {
    const { margin, popup, children, ...props } = this.props;

    return (
      <div ref={this.anchorRef} {...props}>
        {children}
        <div className={styles['popup']} ref={this.popupRef}>
          {popup}
        </div>
      </div>
    );
  }
}
