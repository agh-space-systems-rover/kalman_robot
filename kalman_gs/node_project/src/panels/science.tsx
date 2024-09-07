import styles from './science.module.css'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';
import { fa1, fa2, faBox, faBoxOpen, faPause, faPlay, faStop } from '@fortawesome/free-solid-svg-icons';
import { onAutonomyClicked, onButtonClicked, onContainerClicked, uiData } from '../common/science';


export enum ButtonTypes {
    TARE,
    REQUEST,
    AUTONOMY
}

export enum WeightTypes {
    DRILL,
    ROCKS,
    SAMPLE
}

export enum ContainerState {
    CLOSE,
    OPEN
}

export enum AutonomyButton {
    RESET,
    PAUSE,
    PLAY
}


const weightButtonLabels = {
    [WeightTypes.DRILL]: "Drill",
    [WeightTypes.ROCKS]: "Rocks",
    [WeightTypes.SAMPLE]: "Sample"
};
type WeightInfo = {
    weightType: WeightTypes
}

function DataDisplay({labelText, displayedText})
{
    return (
        <div className={styles['inline-items']}>
            <div className={styles['info-label']}>{labelText}</div>
            <div className={styles['info-display']}>{displayedText}</div>
        </div>
    )
}

function AutonomyPanel()
{
    function handleAutonomyReset() {
        onAutonomyClicked(AutonomyButton.RESET);
    }

    function handleAutonomyPause() {
        onAutonomyClicked(AutonomyButton.PAUSE);
    }

    function handleAutonomyPlay() {
        onAutonomyClicked(AutonomyButton.PLAY);
    }

    return (
        <div className={styles['horizontal']}>
            <div onClick={handleAutonomyReset} className={styles['autonomy-button']}><FontAwesomeIcon icon={faStop}/></div>
            <div onClick={handleAutonomyPause} className={styles['autonomy-button']}><FontAwesomeIcon icon={faPause}/></div>
            <div onClick={handleAutonomyPlay} className={styles['autonomy-button']}><FontAwesomeIcon icon={faPlay}/></div>
        </div>
    )
}


function StandardWeight({weightType}: WeightInfo) {
    const [count, setCount] = useState(0); // to force re-render

    useEffect(() => {
        window.addEventListener("science-updated", ()=> {
            setCount(count+1);
        });
    });

    function handleTare() {
        onButtonClicked(weightType, ButtonTypes.TARE);
    }

    function handleRequest() {
        onButtonClicked(weightType, ButtonTypes.REQUEST);
    }

    return (
        <div className={styles['standardWeight']}>
            <DataDisplay labelText={weightButtonLabels[weightType]+": "} displayedText={uiData[weightType]}></DataDisplay>
            <div className={styles['inline-items']}>
                <div className={styles['tare-button']} onClick={handleTare}>Tare</div>
                <div className={styles['send-button']} onClick={handleRequest}>Request</div>
            </div>
            {weightType === WeightTypes.DRILL && (
                <>
                    <h2 className={styles.center}>Autonomy</h2>
                    <AutonomyPanel />
                </>
            )}
        </div>
    )
}



function Weights() {
    return (
        <div className={styles['weights']}>
            <StandardWeight weightType={WeightTypes.ROCKS}></StandardWeight>
            <StandardWeight weightType={WeightTypes.DRILL}></StandardWeight>
            <StandardWeight weightType={WeightTypes.SAMPLE}></StandardWeight>
      </div>
    )
}

function Container({containerNumber})
{
    const [count, setCount] = useState(0); // to force re-render
    useEffect(() => {
        window.addEventListener("science-updated", ()=> {
            setCount(count+1);
        });
    });

    const [containerState, setContainerState] = useState(ContainerState.CLOSE);

    function handleClick(state) {
        setContainerState(containerState == ContainerState.CLOSE? ContainerState.OPEN:ContainerState.CLOSE);
        onContainerClicked(state, containerNumber);
        // onContainerClicked(state, containerNumber);
        // onContainerClicked(state, containerNumber);
    }

    return (
        <div className={styles['column']}>
        <FontAwesomeIcon
        onClick={() => handleClick(ContainerState.OPEN)}
        className={styles['icon']}
        icon={faBoxOpen}/>
        <FontAwesomeIcon
            className={styles['icon-digit']}
            icon={[fa1, fa2][containerNumber]}
          />
        <FontAwesomeIcon
          onClick={() => handleClick(ContainerState.CLOSE)}
          className={styles['icon'] + " " + styles['icon_second']}
          icon={faBox}/>
          <FontAwesomeIcon
              className={styles['icon-digit']}
              icon={[fa1, fa2][containerNumber]}
            />
        </div>
        
    )
}


export default function Science() {
    return (
      <div className={styles['vertical']}>
        <div className={styles['horizontal']}><Weights></Weights></div>
        <div className={styles['horizontal']}>
            <Container containerNumber="0"></Container>
            <Container containerNumber="1"></Container>
        </div>
      </div>
    );
  }
  
  
  
  
  