import { alertsRef, settingsRef } from '../common/refs';
import { ros } from '../common/ros';
import { SetFeedRequest } from '../common/ros-interfaces';
import { getKeybind } from './keybinds';
import { Service } from 'roslib';

let feedCameras = [1, 2];
let feedChannels = [4, 8];
let feedPowers = [1, 1];

// Load feed config from local storage.
const feedConfig = localStorage.getItem('feed-config');
if (feedConfig) {
  const feeds: any = JSON.parse(feedConfig);
  feedCameras = feeds.cameras;
  feedChannels = feeds.channels;
  feedPowers = feeds.powers;
}

// Save feed config to local storage on update.
window.addEventListener('feeds-updated', () => {
  localStorage.setItem(
    'feed-config',
    JSON.stringify({
      cameras: feedCameras,
      channels: feedChannels,
      powers: feedPowers
    } as any)
  );
});

// ROS
window.addEventListener('ros-connect', () => {
  const setFeed = new Service<SetFeedRequest, {}>({
    ros: ros,
    name: '/set_feed',
    serviceType: 'kalman_interfaces/SetFeeds'
  });

  // Keep track of previous values to avoid unnecessary calls.
  // 0 is an invalid state that will force update on first call.
  let prevCameras = [0, 0];
  let prevChannels = [0, 0];
  let prevPowers = [0, 0];
  let lastActionableReq = null;

  const sendCall = () => {
    let errorShownOnce = false; // Prevents from showing two errors, one for each feed.
    // For each of the two feeds, send a call if any of the values have changed.
    for (let feed = 0; feed < 2; feed++) {
      // Construct the request.
      // On first call, all previous values are 0, so all current values will be sent.
      const req: SetFeedRequest = {
        feed: feed + 1, // Feeds in SetFeed are 1-indexed.
        camera: feedCameras[feed] === prevCameras[feed] ? 0 : feedCameras[feed],
        channel:
          feedChannels[feed] === prevChannels[feed] ? 0 : feedChannels[feed],
        power: feedPowers[feed] === prevPowers[feed] ? 0 : feedPowers[feed]
      };

      // Remember the current values for the next call.
      prevCameras[feed] = feedCameras[feed];
      prevChannels[feed] = feedChannels[feed];
      prevPowers[feed] = feedPowers[feed];

      // If any of the values have changed, send the call.
      const errorCb = (error: string) => {
        if (!errorShownOnce) {
          alertsRef.current?.pushAlert('Failed to update feeds: ' + error);
          errorShownOnce = true;
        }
      };
      if (req.camera || req.channel || req.power) {
        setFeed.callService(req, undefined, errorCb);
        lastActionableReq = req;
      } else if (lastActionableReq !== null) {
        // If no values have changed, repeat the last call.
        setFeed.callService(lastActionableReq, undefined, errorCb);
      }
    }
  };

  window.addEventListener('feeds-updated', () => sendCall());
  sendCall();
});

// Keybinds
let changingFeedI = 0;
function cycleFeedCameras(direction: number) {
  feedCameras[changingFeedI] += direction;
  if (feedCameras[changingFeedI] < 1) {
    feedCameras[changingFeedI] = 8;
  } else if (feedCameras[changingFeedI] > 8) {
    feedCameras[changingFeedI] = 1;
  }
  window.dispatchEvent(new Event('feeds-updated'));
}
function showCameraOnFeed(camera: number, overrideFeedI?: number) {
  if (overrideFeedI !== undefined) {
    feedCameras[overrideFeedI] = camera;
  } else {
    feedCameras[changingFeedI] = camera;
  }
  window.dispatchEvent(new Event('feeds-updated'));
}
window.addEventListener('keydown', (event) => {
  // Check if any input box is focused.
  if (document.activeElement.tagName === 'INPUT') {
    return;
  }
  // Check if settings are open.
  if (settingsRef.current?.isShown()) {
    return;
  }

  switch (event.code) {
    case getKeybind('Cycle Feed 1 Cameras Backwards'):
      cycleFeedCameras(-1);
      break;
    case getKeybind('Cycle Feed 1 Cameras'):
      cycleFeedCameras(1);
      break;
    case getKeybind('Show Camera 1 on Feed 1'):
      showCameraOnFeed(1);
      break;
    case getKeybind('Show Camera 2 on Feed 1'):
      showCameraOnFeed(2);
      break;
    case getKeybind('Show Camera 3 on Feed 1'):
      showCameraOnFeed(3);
      break;
    case getKeybind('Show Camera 4 on Feed 1'):
      showCameraOnFeed(4);
      break;
    case getKeybind('Show Camera 5 on Feed 1'):
      showCameraOnFeed(5);
      break;
    case getKeybind('Show Camera 6 on Feed 1'):
      showCameraOnFeed(6);
      break;
    case getKeybind('Show Camera 7 on Feed 1'):
      showCameraOnFeed(7);
      break;
    case getKeybind('Show Camera 8 on Feed 1'):
      showCameraOnFeed(8);
      break;
    case getKeybind('Hold to Change Cameras on Feed 2 not 1'):
      changingFeedI = 1;
      break;
    case getKeybind('Show Camera 1 on Feed 2'):
      showCameraOnFeed(1, 1);
      break;
    case getKeybind('Show Camera 2 on Feed 2'):
      showCameraOnFeed(2, 1);
      break;
    case getKeybind('Show Camera 3 on Feed 2'):
      showCameraOnFeed(3, 1);
      break;
    case getKeybind('Show Camera 4 on Feed 2'):
      showCameraOnFeed(4, 1);
      break;
    case getKeybind('Show Camera 5 on Feed 2'):
      showCameraOnFeed(5, 1);
      break;
    case getKeybind('Show Camera 6 on Feed 2'):
      showCameraOnFeed(6, 1);
      break;
    case getKeybind('Show Camera 7 on Feed 2'):
      showCameraOnFeed(7, 1);
      break;
    case getKeybind('Show Camera 8 on Feed 2'):
      showCameraOnFeed(8, 1);
      break;
  }
});
window.addEventListener('keyup', (event) => {
  if (event.code === getKeybind('Hold to Change Cameras on Feed 2 not 1')) {
    changingFeedI = 0;
  }
});

export { feedCameras, feedChannels, feedPowers };
