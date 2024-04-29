import styles from './input.module.css';

import { Component, createRef } from 'react';

type Props = {
  type?: string;
  placeholder?: string;
  defaultValue?: string;
  onChange?: (text: string) => void;
  onSubmit?: (text: string) => void;
  className?: string;
  [key: string]: any;
};

export default class Input extends Component<Props> {
  private ref = createRef<HTMLInputElement>();

  constructor(props: Props) {
    super(props);
  }

  getValue(): any {
    if (this.props.type == 'number') {
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

  render() {
    const {
      type,
      placeholder,
      defaultValue,
      onChange,
      onSubmit,
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
          type={type}
          placeholder={placeholder}
          defaultValue={defaultValue}
          className={styles['input-field']}
          onChange={(e) => onChange?.(e.target.value)}
          onKeyUp={(e) => {
            if (e.key === 'Enter') {
              onSubmit?.(e.currentTarget.value);
              this.ref.current?.blur();
            }
          }}
        />
      </div>
    );
  }
}
