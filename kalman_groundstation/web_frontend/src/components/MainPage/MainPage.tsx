import './MainPage.css'

import React from 'react'

// import { AccessPoint } from '../AccessPoint/AccessPoint'
import { Arm } from '../Arm/Arm'
import { AutonomyFront } from '../AutonomyFront/AutonomyFront'
import { FeedIndicator } from '../FeedControl/FeedIndicator'
import { IMUVis } from '../IMUVis/IMUVis'
import { Map } from '../Map/Map'
// import { OldMap } from '../Map/oldMap'
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
              {/* <AccessPoint /> */}

              <textarea
                id='gps-waypoints-data'
                style={{ backgroundColor: '#081f3e', margin: '10px', color: 'white', flex: 1, padding: '10px' }}
                placeholder={`waypoint1 51.45302204448234 -112.71581991471845
waypoint2 51.453090573262614 -112.71610959329223
waypoint3 N 51 27 11 W 112 42 59`}
              ></textarea>
            </div>
          )}
          <Map />
        </div>
      </>
    </div>
  )
}
