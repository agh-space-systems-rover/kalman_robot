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
  disabled?: boolean;
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
    if (!this.props.disabled) this.ref.current!.value = value;
  }

  isEmpty(): boolean {
    return this.ref.current?.value === '';
  }

  selectAll() {
    if (!this.props.disabled) this.ref.current?.select();
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
      disabled,
      ...props
    } = this.props;

    return (
      <div
        className={styles['input'] + (className ? ` ${className}` : '')}
        aria-disabled={disabled || undefined}
        {...props}
      >
        <input
          ref={this.ref}
          placeholder={placeholder}
          defaultValue={defaultValue}
          minLength={minLength}
          maxLength={maxLength}
          className={styles['input-field']}
          disabled={disabled}
          onChange={(e) => {
            if (disabled) return;
            if (type === 'float') {
              this.ref.current!.value = this.ref.current!.value.replace(/[^\d.-]/g, '');
            }
            onChange?.(e.target.value);
          }}
          onKeyUp={(e) => {
            if (disabled) return;
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
