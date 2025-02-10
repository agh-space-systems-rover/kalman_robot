import styles from './checkbox.module.css';

import { faCheck } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component, createRef, CSSProperties } from 'react';

type Props = {
  checked?: boolean;
  onChange?: (checked: boolean) => void;
  className?: string;
  label?: string;
  autoFocus?: boolean;
  style?: CSSProperties;
  checkboxStyle?: CSSProperties;
  labelStyle?: CSSProperties;
  [key: string]: any;
};

export default class Checkbox extends Component<Props> {
  private ref = createRef<HTMLInputElement>();

  constructor(props: Props) {
    super(props);
  }

  isChecked(): boolean {
    return this.ref.current?.checked || false;
  }

  setChecked(value: boolean) {
    if (this.ref.current) {
      this.ref.current.checked = value;
    }
  }

  toggle() {
    if (this.ref.current) {
      this.ref.current.checked = !this.ref.current.checked;
      this.props.onChange?.(this.ref.current.checked);
    }
  }

  render() {
    const { checked, onChange, className, label, style, checkboxStyle, labelStyle, ...props } = this.props;

    return (
      <div
        className={`${styles['checkbox-container']} ${className || ''}`}
        style={style}
        onClick={() => this.toggle()}
        {...props}
      >
        <div
          className={`${styles['checkbox']} ${this.isChecked() ? styles['checkbox-checked'] : ''}`}
          style={checkboxStyle}
        >
          <input
            ref={this.ref}
            type='checkbox'
            defaultChecked={checked}
            className={styles['checkbox-field']}
            onChange={(e) => onChange?.(e.target.checked)}
            autoFocus={this.props.autoFocus}
          />
          {this.isChecked() && <FontAwesomeIcon icon={faCheck} className={styles['checkbox-icon']} />}
        </div>
        {label && (
          <span className={styles['checkbox-label']} style={labelStyle}>
            {label}
          </span>
        )}
      </div>
    );
  }
}
