import styles from './feeds.module.css';

import CHANNELS from '../common/feed-channels';
import { feedCameras, feedChannels, feedPowers } from '../common/feeds';
import {
  IconDefinition,
  fa1,
  fa2,
  faCamera,
  faCheck,
  faDisplay,
  faTowerBroadcast,
  faWaveSquare
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useRef, useState } from 'react';

import Button from '../components/button';
import Input from '../components/input';
import Label from '../components/label';

type IntegerSelectorProps = {
  labelColor: string;
  labelIcon: IconDefinition;
  labelTooltip: string;
  min: number;
  max: number;
  defaultValue: number;
  onSet: (value: number) => void;
  customNames?: string[];
};

function IntegerSelector({
  labelColor,
  labelIcon,
  labelTooltip,
  min,
  max,
  defaultValue,
  onSet,
  customNames
}: IntegerSelectorProps) {
  const inputRef = useRef<Input>(null);

  const [editing, setEditing] = useState(false);

  const onSubmit = useCallback(() => {
    let integer = parseInt(inputRef.current?.getValue());
    integer = Math.max(min, Math.min(max, integer));
    if (customNames) {
      inputRef.current?.setValue(customNames[integer]);
    } else {
      inputRef.current?.setValue(integer.toString());
    }
    setEditing(false);
    onSet(integer);
  }, [inputRef, min, max]);

  return (
    <div className={styles['feed-controls-row']}>
      <Label color={labelColor} tooltip={labelTooltip}>
        <FontAwesomeIcon icon={labelIcon} />
      </Label>
      <Input
        className={styles['input'] + (editing ? ' editing' : '')}
        defaultValue={
          customNames ? customNames[defaultValue] : defaultValue.toString()
        }
        ref={inputRef}
        onChange={(value) => {
          // Check if there are non-digit characters
          if (value.match(/\D/)) {
            value = value.replace(/\D/g, '');
            inputRef.current?.setValue(value);
          }
        }}
        // onSubmit={onSubmit} # Handled by onBlur
        onFocus={() => {
          setEditing(true);
          if (customNames) {
            inputRef.current?.setValue(defaultValue.toString());
          }
          inputRef.current?.selectAll();
        }}
        onBlur={onSubmit}
      />
      <Button onClick={onSubmit}>
        <FontAwesomeIcon icon={faCheck} />
      </Button>
    </div>
  );
}

type FeedProps = {
  feedIndex: number;
};

function Feed({ feedIndex }: FeedProps) {
  const style = getComputedStyle(document.body);
  const redBg = style.getPropertyValue('--red-background');
  const greenBg = style.getPropertyValue('--green-background');
  const blueBg = style.getPropertyValue('--blue-background');

  const [rerenderCount, setRerenderCount] = useState(0);
  useEffect(() => {
    const updateFeeds = () => {
      setRerenderCount((count) => count + 1);
    };
    window.addEventListener('feeds-updated', updateFeeds);
    return () => {
      window.removeEventListener('feeds-updated', updateFeeds);
    };
  }, []);

  return (
    <div className={styles['feed']}>
      <div className={styles['feed-controls-row']}>
        <div className={styles['icon']}>
          <FontAwesomeIcon icon={faDisplay} />
          <FontAwesomeIcon
            className={styles['icon-digit']}
            icon={[fa1, fa2][feedIndex]}
          />
        </div>
      </div>
      <IntegerSelector
        labelColor={redBg}
        labelIcon={faCamera}
        labelTooltip='Camera (1 to 8)'
        min={1}
        max={8}
        defaultValue={feedCameras[feedIndex]}
        onSet={(value) => {
          feedCameras[feedIndex] = value;
          window.dispatchEvent(new Event('feeds-updated'));
        }}
        key={feedCameras[feedIndex]}
      />
      <IntegerSelector
        labelColor={greenBg}
        labelIcon={faWaveSquare}
        labelTooltip='Channel (1 to 40)'
        min={1}
        max={40}
        defaultValue={feedChannels[feedIndex]}
        onSet={(value) => {
          feedChannels[feedIndex] = value;
          window.dispatchEvent(new Event('feeds-updated'));
        }}
        customNames={CHANNELS}
        key={feedChannels[feedIndex] + 100}
      />
      <IntegerSelector
        labelColor={blueBg}
        labelIcon={faTowerBroadcast}
        labelTooltip='Power (1 to 4)'
        min={1}
        max={4}
        defaultValue={feedPowers[feedIndex]}
        onSet={(value) => {
          feedPowers[feedIndex] = value;
          window.dispatchEvent(new Event('feeds-updated'));
        }}
        key={feedPowers[feedIndex] + 200}
      />
    </div>
  );
}

export default function Feeds() {
  return (
    <div className={styles['feeds-panel']}>
      <div className={styles['feeds']}>
        <Feed feedIndex={0} />
        <Feed feedIndex={1} />
      </div>
    </div>
  );
}
