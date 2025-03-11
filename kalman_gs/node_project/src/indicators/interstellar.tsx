import styles from './interstellar.module.css';

import { faPlay, faShuttleSpace, faStop } from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useRef, useState } from 'react';

import Tooltip from '../components/tooltip';

class SeamlessAudioLoop {
  audioContext: AudioContext;
  isPlaying: boolean;
  bufferSource: AudioBufferSourceNode;
  buffer: AudioBuffer;

  constructor() {
    this.audioContext = new (window.AudioContext || (window as any).webkitAudioContext)();
    this.isPlaying = false;
    this.bufferSource = null;
    this.buffer = null;
  }

  async loadAudio(url) {
    try {
      const response = await fetch(url);
      const arrayBuffer = await response.arrayBuffer();
      this.buffer = await this.audioContext.decodeAudioData(arrayBuffer);
    } catch (error) {
      console.error('Error loading audio:', error);
    }
  }

  play() {
    console.log('play()');

    // Create a new buffer source
    this.bufferSource = this.audioContext.createBufferSource();
    this.bufferSource.buffer = this.buffer;

    // Enable looping
    this.bufferSource.loop = true;

    // Connect to audio output
    this.bufferSource.connect(this.audioContext.destination);

    // Start playback
    this.bufferSource.start(0);
    this.isPlaying = true;
  }

  stop() {
    console.log('stop()');

    if (this.bufferSource) {
      this.bufferSource.stop(0);
      this.bufferSource = null;
    }
    this.isPlaying = false;
  }

  togglePlayback() {
    if (this.audioContext.state === 'suspended') {
      this.audioContext.resume();
    }

    if (this.isPlaying) {
      this.stop();
    } else {
      this.play();
    }
  }
}

const audioLoop = new SeamlessAudioLoop();
audioLoop.loadAudio('/interstellar.ogg');
export { audioLoop };

export default function Interstellar() {
  const [playing, setPlaying] = useState(false);

  // Monitor playback and update the icon accordingly.
  useEffect(() => {
    const recheckPlayback = () => {
      setPlaying(audioLoop.isPlaying);
    };
    const interval = setInterval(recheckPlayback, 100);
    return () => clearInterval(interval);
  }, []);

  return (
    <Tooltip
      text='Play "No Time for Caution"\nfrom Interstellar (2014).'
      className={styles['interstellar']}
      onClick={() => {
        audioLoop.togglePlayback();
        setPlaying(audioLoop.isPlaying);
      }}
    >
      <FontAwesomeIcon icon={faShuttleSpace} />
      <FontAwesomeIcon
        icon={playing ? faStop : faPlay}
        className={styles['overlay-icon'] + ' ' + styles[playing ? 'stop' : 'play']}
      />
    </Tooltip>
  );
}
