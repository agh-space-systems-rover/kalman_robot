import styles from './modal.module.css';



import Button from './button';
import Input from './input';
import type { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import { faCheck, faXmark } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component, createRef } from 'react';


type PromptOptions = {
  title: string;
  icon?: IconDefinition | null;
  message?: string;
  placeholder?: string;
  defaultValue?: string;
  confirmText?: string;
  cancelText?: string;
  onSubmit: (value: string) => void;
};

type ConfirmOptions = {
  title: string;
  icon?: IconDefinition | null;
  message?: string;
  confirmText?: string;
  cancelText?: string;
  onConfirm: () => void;
};

type State = {
  shown: boolean;
  mode: 'prompt' | 'confirm' | null;
  title: string;
  icon?: IconDefinition | null;
  message?: string;
  confirmText: string;
  cancelText: string;
  defaultValue?: string;
  placeholder?: string;
  onSubmit?: (value: string) => void;
  onConfirm?: () => void;
};

export default class Modal extends Component<{}, State> {
  static defaultState: State = {
    shown: false,
    mode: null,
    title: '',
    icon: null,
    message: '',
    confirmText: 'OK',
    cancelText: 'Cancel',
    defaultValue: '',
    placeholder: '',
    onSubmit: undefined,
    onConfirm: undefined
  };

  state: State = Modal.defaultState;
  private inputRef = createRef<Input>();

  escHandler = (e: KeyboardEvent) => {
    if (e.key === 'Escape') this.hide();
  };

  showPrompt(opts: PromptOptions) {
    this.setState(
      {
        shown: true,
        mode: 'prompt',
        title: opts.title,
        icon: opts.icon ?? null,
        message: opts.message,
        confirmText: opts.confirmText ?? 'Confirm',
        cancelText: opts.cancelText ?? 'Cancel',
        defaultValue: opts.defaultValue,
        placeholder: opts.placeholder,
        onSubmit: opts.onSubmit,
        onConfirm: undefined
      },
      () => window.addEventListener('keydown', this.escHandler)
    );
  }

  showConfirm(opts: ConfirmOptions) {
    this.setState(
      {
        shown: true,
        mode: 'confirm',
        title: opts.title,
        icon: opts.icon ?? null,
        message: opts.message,
        confirmText: opts.confirmText ?? 'Confirm',
        cancelText: opts.cancelText ?? 'Cancel',
        defaultValue: '',
        placeholder: '',
        onSubmit: undefined,
        onConfirm: opts.onConfirm
      },
      () => window.addEventListener('keydown', this.escHandler)
    );
  }

  hide() {
    this.setState(Modal.defaultState, () => window.removeEventListener('keydown', this.escHandler));
  }

  render() {
    const { shown, mode, title, icon, message, confirmText, cancelText, defaultValue, placeholder } = this.state;

    return (
      <div className={styles['modal-bg'] + (shown ? ` ${styles['shown']}` : '')} onClick={() => this.hide()}>
        <div className={styles['modal']} onClick={(e) => e.stopPropagation()}>
          <div className={styles['content']}>
            <h1>
              {icon ? (
                <>
                  <FontAwesomeIcon icon={icon} />
                  &nbsp;&nbsp;
                </>
              ) : null}
              {title}
            </h1>

            {message && <div className={styles['message']}>{message}</div>}

            {mode === 'prompt' && (
              <div>
                <Input
                  ref={this.inputRef}
                  placeholder={placeholder}
                  defaultValue={defaultValue}
                  autoFocus
                  onSubmit={(v) => {
                    this.state.onSubmit?.(v);
                    this.hide();
                  }}
                />
              </div>
            )}

            <div className={styles['actions']}>
              <Button
                className={styles['action-btn']}
                onClick={() => {
                  if (mode === 'prompt') {
                    const v = this.inputRef.current?.getValue() ?? '';
                    this.state.onSubmit?.(String(v ?? ''));
                  } else if (mode === 'confirm') {
                    this.state.onConfirm?.();
                  }
                  this.hide();
                }}
              >
                <FontAwesomeIcon icon={faCheck} />
                &nbsp;&nbsp;
                <span>{confirmText}</span>
              </Button>
              <Button className={styles['action-btn']} onClick={() => this.hide()}>
                <FontAwesomeIcon icon={faXmark} />
                &nbsp;&nbsp;
                <span>{cancelText}</span>
              </Button>
            </div>
          </div>
        </div>
      </div>
    );
  }
}
