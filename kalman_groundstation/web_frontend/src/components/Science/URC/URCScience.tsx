import './URCScience.css'

import { useAppSelector } from '../../../store/storeHooks'
import { URCModule } from './URCModule'
import { URCProbilab } from './URCProbilab'

export const URCScience: () => JSX.Element = () => {
  const modules = useAppSelector((state) => state.science.modules)

  return (
    <div className='science-wrapper'>
      <div className='modules-wrapper'>
        {modules.map((module, idx) => (
          <URCModule module={module} moduleId={idx} key={idx} />
        ))}
      </div>
      <URCProbilab />
    </div>
  )
}
