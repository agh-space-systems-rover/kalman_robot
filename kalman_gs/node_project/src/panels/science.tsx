import styles from './science.module.css'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useEffect, useState } from 'react';
import { fa1, fa2, faBox, faBoxOpen } from '@fortawesome/free-solid-svg-icons';


enum WeightTypes {
    Drill,
    Rocks,
    Sample
}


const weightButtonLabels = {
    [WeightTypes.Drill]: "Drill",
    [WeightTypes.Rocks]: "Rocks",
    [WeightTypes.Sample]: "Sample"
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




function StandardWeight({weightType}: WeightInfo) {

    return (
        <div className={styles['standardWeight']}>
            <DataDisplay labelText={weightButtonLabels[weightType]+": "} displayedText="2137 g"></DataDisplay>
            <div className={styles['inline-items']}>
                <div className={styles['tare-button']}>Tare</div>
                <div className={styles['send-button']}>Request</div>
            </div>
            { weightType==WeightTypes.Drill? (
                <div className={styles['autonomy-button']}>Autonomy</div>
            ) : ""}
        </div>

    )
}



function Weights() {
    return (
        <div className={styles['weights']}>
            <StandardWeight weightType={WeightTypes.Rocks}></StandardWeight>
            <StandardWeight weightType={WeightTypes.Drill}></StandardWeight>
            <StandardWeight weightType={WeightTypes.Sample}></StandardWeight>
      </div>
    )
}

function Container({containerNumber})
{
    const [containerState, setContainerState] = useState(0);

    function handleClick() {
        setContainerState(containerState ^ 1);
      }

    return (
        <div>
        <FontAwesomeIcon
        onClick={handleClick}
        className={styles['icon']}
        icon={containerState? faBoxOpen : faBox}/>
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
  
  
  
  
  