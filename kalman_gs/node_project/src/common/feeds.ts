import { getKeybind } from './keybinds';
import { alertsRef, settingsRef } from './refs';
import { ros } from './ros';
import { SetFeedRequest } from './ros-interfaces';
import { Service } from 'roslib';


// Create arrays storing feed configuration
// Store initial values
let feedCameras = [1, 2];
let feedChannels = [15, 8];
let feedPowers = [1, 1];

// Type for storing details from CustomEvent
type FeedUpdateDetail = {
  feed: number;
  camera?: number;
  channel?: number;
  power?: number;
};

// Initial SetFeed request to send after connecting ROS
const prepareInitialSetFeedRequest = (feedI: number) => {
  return {
    feed: feedI + 1, // Feeds in SetFeed are 1-indexed
    camera: feedCameras[feedI],
    channel: feedChannels[feedI],
    power: feedPowers[feedI]
  };
};

// SetFeed request with operator data to send after change feeds in feeds.tsx or via keyboard
const prepareSetFeedRequestFromEventDetail = (detail: FeedUpdateDetail) => {
  return {
    feed: detail.feed + 1, // Feeds in SetFeed are 1-indexed
    camera: detail.camera ?? 0,
    channel: detail.channel ?? 0,
    power: detail.power ?? 0
  };
};

// Load feed config from local storage.
const feedConfig = localStorage.getItem('feed-config');
if (feedConfig) {
  const feeds: any = JSON.parse(feedConfig);

  // Verify that the values are valid.
  if (
    feeds.cameras.length === 2 &&
    feeds.channels.length === 2 &&
    feeds.powers.length === 2 &&
    feeds.cameras.every((camera: any) => typeof camera === 'number') &&
    feeds.channels.every((channel: any) => typeof channel === 'number') &&
    feeds.powers.every((power: any) => typeof power === 'number')
  ) {
    // If valid, use the values.
    feedCameras = feeds.cameras;
    feedChannels = feeds.channels;
    feedPowers = feeds.powers;
  }
}

window.addEventListener('feeds-updated', (e) => {
  // Save feed config to local storage on update.
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

  const sendCall = (feedRequest: SetFeedRequest | null = null, showAlerts: boolean = true) => {
    // If any of the values have changed, send the call.
    const errorCb = (error: string) => {
      if (showAlerts) {
        alertsRef.current?.pushAlert('Failed to update feeds: ' + error);
      }
    };

    // Send previously prepared request
    setFeed.callService(feedRequest, undefined, errorCb);
  };

  // Send initial request to set default values or stored in local storage config
  sendCall(prepareInitialSetFeedRequest(0), false);
  sendCall(prepareInitialSetFeedRequest(1), false);

  // Handle changes from feeds panel or keyboard
  window.addEventListener('feeds-updated', (e) => {
    const detail = (e as CustomEvent<FeedUpdateDetail>).detail;
    sendCall(prepareSetFeedRequestFromEventDetail(detail));
  });
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
  window.dispatchEvent(
    new CustomEvent<FeedUpdateDetail>('feeds-updated', {
      detail: {
        feed: changingFeedI,
        camera: feedCameras[changingFeedI]
      }
    })
  );
}

function showCameraOnFeed(camera: number, overrideFeedI?: number) {
  const feedI = overrideFeedI ?? changingFeedI;
  feedCameras[feedI] = camera;

  window.dispatchEvent(
    new CustomEvent<FeedUpdateDetail>('feeds-updated', {
      detail: {
        feed: feedI,
        camera: feedCameras[feedI]
      }
    })
  );
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

export { FeedUpdateDetail, feedCameras, feedChannels, feedPowers };
