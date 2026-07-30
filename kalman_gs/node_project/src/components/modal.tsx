import styles from './modal.module.css';

import Button from './button';
import Input from './input';
import type { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import { faCheck, faCircleInfo, faFloppyDisk, faTrash, faXmark } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { Component, RefObject, createRef } from 'react';

type AlertOptions = {
  title: string;
  icon?: IconDefinition | null;
  message?: string;
  confirmText?: string;
};

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

type CoordinatePointOptions = {
  label: string;
  latitude?: number;
  longitude?: number;
  onDelete?: () => void;
};

type CoordinatePointValue = {
  latitude: number;
  longitude: number;
};

type CoordinatesOptions = {
  title: string;
  icon?: IconDefinition | null;
  message?: string;
  points: CoordinatePointOptions[];
  onSave: (points: (CoordinatePointValue | null)[]) => void;
  onDelete?: () => void;
};

type State = {
  shown: boolean;
  mode: 'alert' | 'prompt' | 'confirm' | 'coordinates' | null;
  title: string;
  icon?: IconDefinition | null;
  message?: string;
  confirmText: string;
  cancelText: string;
  defaultValue?: string;
  placeholder?: string;
  onSubmit?: (value: string) => void;
  onConfirm?: () => void;
  coordinates?: CoordinatesOptions;
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
  private coordinateInputRefs: {
    latitude: RefObject<Input>;
    longitude: RefObject<Input>;
  }[] = [];

  escHandler = (e: KeyboardEvent) => {
    if (e.key === 'Escape') this.hide();
  };

  showAlert(opts: AlertOptions) {
    this.setState(
      {
        shown: true,
        mode: 'alert',
        title: opts.title,
        icon: opts.icon === undefined ? faCircleInfo : opts.icon,
        message: opts.message,
        confirmText: opts.confirmText ?? 'OK',
        cancelText: '',
        defaultValue: '',
        placeholder: '',
        onSubmit: undefined,
        onConfirm: undefined
      },
      () => window.addEventListener('keydown', this.escHandler)
    );
  }

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

  showCoordinates(opts: CoordinatesOptions) {
    this.coordinateInputRefs = opts.points.map(() => ({
      latitude: createRef<Input>(),
      longitude: createRef<Input>()
    }));
    this.setState(
      {
        shown: true,
        mode: 'coordinates',
        title: opts.title,
        icon: opts.icon ?? null,
        message: opts.message,
        coordinates: opts
      },
      () => window.addEventListener('keydown', this.escHandler)
    );
  }

  submitCoordinates() {
    const points: (CoordinatePointValue | null)[] = [];

    for (const refs of this.coordinateInputRefs) {
      const latitude = refs.latitude.current?.getValue();
      const longitude = refs.longitude.current?.getValue();

      if (latitude === undefined && longitude === undefined) {
        points.push(null);
        continue;
      }

      if (
        !Number.isFinite(latitude) ||
        !Number.isFinite(longitude) ||
        latitude < -90 ||
        latitude > 90 ||
        longitude < -180 ||
        longitude > 180
      ) {
        return;
      }

      points.push({ latitude, longitude });
    }

    const onSave = this.state.coordinates?.onSave;
    this.hide();
    onSave?.(points);
  }

  hide() {
    this.setState(Modal.defaultState, () => window.removeEventListener('keydown', this.escHandler));
  }

  render() {
    const { shown, mode, title, icon, message, confirmText, cancelText, defaultValue, placeholder, coordinates } =
      this.state;

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
                    const onSubmit = this.state.onSubmit;
                    this.hide();
                    onSubmit?.(v);
                  }}
                />
              </div>
            )}

            {mode === 'coordinates' && (
              <div className={styles['coordinate-points']}>
                {coordinates?.points.map((point, index) => (
                  <div className={styles['coordinate-point']} key={point.label}>
                    <div className={styles['coordinate-point-label']}>{point.label}</div>
                    <div className={styles['coordinate-point-inputs']}>
                      <Input
                        ref={this.coordinateInputRefs[index]?.latitude}
                        type='float'
                        placeholder='Latitude'
                        defaultValue={point.latitude?.toString() ?? ''}
                        autoFocus={index === 0}
                      />
                      <Input
                        ref={this.coordinateInputRefs[index]?.longitude}
                        type='float'
                        placeholder='Longitude'
                        defaultValue={point.longitude?.toString() ?? ''}
                      />
                      <Button
                        className={styles['coordinate-point-delete']}
                        tooltip={`Delete ${point.label}.`}
                        onClick={() => {
                          this.coordinateInputRefs[index]?.latitude.current?.setValue('');
                          this.coordinateInputRefs[index]?.longitude.current?.setValue('');
                          point.onDelete?.();
                        }}
                      >
                        <FontAwesomeIcon icon={faXmark} />
                      </Button>
                    </div>
                  </div>
                ))}
              </div>
            )}

            {mode === 'coordinates' ? (
              <div className={styles['actions']}>
                <Button
                  className={styles['action-btn']}
                  disabled={!coordinates?.onDelete}
                  onClick={() => {
                    const onDelete = coordinates?.onDelete;
                    this.hide();
                    onDelete?.();
                  }}
                >
                  <FontAwesomeIcon icon={faTrash} />
                  &nbsp;&nbsp;<span>Delete</span>
                </Button>
                <Button
                  className={styles['action-btn']}
                  disabled={!coordinates}
                  onClick={() => this.submitCoordinates()}
                >
                  <FontAwesomeIcon icon={faFloppyDisk} />
                  &nbsp;&nbsp;<span>Save</span>
                </Button>
              </div>
            ) : (
              <div className={styles['actions']}>
                <Button
                  className={styles['action-btn']}
                  onClick={() => {
                    if (mode === 'prompt') {
                      const v = this.inputRef.current?.getValue() ?? '';
                      const onSubmit = this.state.onSubmit;
                      this.hide();
                      onSubmit?.(String(v ?? ''));
                    } else if (mode === 'confirm') {
                      const onConfirm = this.state.onConfirm;
                      this.hide();
                      onConfirm?.();
                    } else {
                      this.hide();
                    }
                  }}
                >
                  <FontAwesomeIcon icon={faCheck} />
                  &nbsp;&nbsp;
                  <span>{confirmText}</span>
                </Button>
                {mode !== 'alert' && (
                  <Button className={styles['action-btn']} onClick={() => this.hide()}>
                    <FontAwesomeIcon icon={faXmark} />
                    &nbsp;&nbsp;
                    <span>{cancelText}</span>
                  </Button>
                )}
              </div>
            )}
          </div>
        </div>
      </div>
    );
  }
}
