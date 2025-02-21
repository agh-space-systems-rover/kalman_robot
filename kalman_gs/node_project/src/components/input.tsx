import styles from './input.module.css';

import { Component, createRef } from 'react';

type Props = {
  type?: string;
  placeholder?: string;
  defaultValue?: string;
  minLength?: number;
  maxLength?: number;
  onChange?: (text: string) => void;
  onSubmit?: (text: string) => void;
  onFocus?: () => void;
  onBlur?: () => void;
  className?: string;
  autoFocus?: boolean;
  [key: string]: any;
};

export default class Input extends Component<Props> {
  private ref = createRef<HTMLInputElement>();

  constructor(props: Props) {
    super(props);
  }

  getValue(): any {
    if (this.props.type == 'float') {
      if (this.ref.current?.value === '') {
        return undefined;
      }
      return parseFloat(this.ref.current?.value!);
    }
    return this.ref.current?.value;
  }

  setValue(value: any) {
    this.ref.current!.value = value;
  }

  isEmpty(): boolean {
    return this.ref.current?.value === '';
  }

  selectAll() {
    this.ref.current?.select();
  }

  render() {
    const {
      type,
      placeholder,
      defaultValue,
      minLength,
      maxLength,
      onChange,
      onSubmit,
      onFocus,
      onBlur,
      className,
      ...props
    } = this.props;

    return (
      <div
        className={styles['input'] + (className ? ` ${className}` : '')}
        {...props}
      >
        <input
          ref={this.ref}
          placeholder={placeholder}
          defaultValue={defaultValue}
          minLength={minLength}
          maxLength={maxLength}
          className={styles['input-field']}
          onChange={(e) => {
            if (type === 'float') {
              console.log(e.target.value);
              this.ref.current.value = this.ref.current.value.replace(
                /[^\d.-]/g,
                ''
              );
              console.log(this.ref.current.value);
            }
            onChange?.(e.target.value);
          }}
          onKeyUp={(e) => {
            if (e.key === 'Enter') {
              onSubmit?.(e.currentTarget.value);
              this.ref.current?.blur();
            } else if (e.key === 'Escape') {
              this.ref.current?.blur();
            }
          }}
          onFocus={onFocus}
          onBlur={onBlur}
          autoFocus={this.props.autoFocus}
        />
      </div>
    );
  }
}
