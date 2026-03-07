import styles from './file-input.module.css';



import Button from './button';
import { faXmark, faUpload } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component, createRef } from 'react';


type Props = {
  accept?: string;
  multiple?: boolean;

  emptyLabel?: string;
  onChange?: (files: FileList | null) => void;
  onClear?: () => void;

  className?: string;

  canClearEmpty?: boolean;
  autoFocus?: boolean;
  disabled?: boolean;
  [key: string]: any;
};

type State = {
  label: string;
  hasFiles: boolean;
};

export default class FileInput extends Component<Props, State> {
  private ref = createRef<HTMLInputElement>();

  state: State = {
    label: this.props.emptyLabel ?? 'Wybierz plik…',
    hasFiles: false
  };

  private updateLabelFromInput() {
    const files = this.ref.current?.files;
    if (!files || files.length === 0) {
      this.setState({
        label: this.props.emptyLabel ?? 'Wybierz plik…',
        hasFiles: false
      });
      return;
    }

    const label = files.length === 1 ? files[0].name : `${files.length} plików`;
    this.setState({ label, hasFiles: true });
  }

  openPicker() {
    if (this.props.disabled) return;
    this.ref.current?.click();
  }

  clear() {
    if (!this.ref.current) return;
    this.ref.current.value = '';
    this.updateLabelFromInput();
    this.props.onChange?.(null);
    this.props.onClear?.();
  }

  render() {
    const { accept, multiple, onChange, className, autoFocus, disabled, emptyLabel, onClear, ...props } = this.props;

    const { label, hasFiles } = this.state;

    return (
      <div className={`${styles['file-input-container']}${className ? ` ${className}` : ''}`} {...props}>
        {/* hidden real input */}
        <input
          ref={this.ref}
          type='file'
          accept={accept}
          multiple={multiple}
          className={styles['file-hidden']}
          autoFocus={autoFocus}
          onChange={(e) => {
            this.updateLabelFromInput();
            onChange?.(e.currentTarget.files);
          }}
        />

        {/* main button */}
        <Button
          className={styles['file-main']}
          tooltip={hasFiles ? 'Zmień plik' : 'Wybierz plik'}
          onClick={() => this.openPicker()}
          disabled={disabled}
        >
          <span className={styles['file-main-inner']}>
            <FontAwesomeIcon icon={faUpload} />
            <span className={styles['file-main-text']}>{label}</span>
          </span>
        </Button>

        {/* clear button (X) */}
        <Button
          className={styles['file-clear']}
          tooltip='Usuń'
          onClick={() => this.clear()}
          disabled={(disabled || !hasFiles) && !this.props.canClearEmpty}
        >
          <FontAwesomeIcon icon={faXmark} />
        </Button>
      </div>
    );
  }
}
