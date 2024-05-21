import './MainPage.css'

import React from 'react'

import { AccessPoint } from '../AccessPoint/AccessPoint'
import { Arm } from '../Arm/Arm'
import { AutonomyFront } from '../AutonomyFront/AutonomyFront'
import { FeedIndicator } from '../FeedControl/FeedIndicator'
import { IMUVis } from '../IMUVis/IMUVis'
import { Map } from '../Map/Map'
import { OldMap } from '../Map/oldMap'
import { Motors } from '../Motors/Motors'
import { Navbar } from '../Navbar/Navbar'
import { Science } from '../Science/Science'
import { Sidebar } from '../Sidebar/Sidebar'
import { Trajectories } from '../Trajectories/Trajectories'

export const MainPage: React.FC = () => {
  const [autonomyView, setAutonomyView] = React.useState(false)
  const [sidebarView, setSidebarView] = React.useState(false)

  const sidebar = sidebarView ? <Sidebar /> : null

  return (
    <div className='wrapper'>
      {sidebar}
      <Navbar
        autonomyView={autonomyView}
        toggleAutonomyView={(): void => setAutonomyView((oldView) => !oldView)}
        sidebarView={sidebarView}
        toggleSidebarView={(): void => setSidebarView((oldView) => !oldView)}
      />
      <>
        <div className='first column'>
          {autonomyView ? (
            <AutonomyFront />
          ) : (
            <>
              <Arm />
              <Trajectories />
            </>
          )}
        </div>
        <div className='second column'>
          <FeedIndicator />
          <Motors />
          <IMUVis />
        </div>
        <div className='third column'>
          {autonomyView ? null : (
            <div style={{ display: 'flex' }}>
              <Science />
              <AccessPoint />
            </div>
          )}
          {/* Old map is deprecated after ERC 2023 */}
          {Date.now() >= 1694960400000 ? <Map /> : <OldMap />}
        </div>
      </>
    </div>
  )
}
