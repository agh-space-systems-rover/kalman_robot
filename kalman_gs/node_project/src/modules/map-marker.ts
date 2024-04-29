export type MapMarker = {
  latitude: number;
  longitude: number;
};

// Initialize the global state.
let mapMarker: MapMarker = {
  latitude: 51.477928,
  longitude: -0.001545
};

// Load state from local storage if available.
const savedLayouts = localStorage.getItem('map-marker');
if (savedLayouts) {
  mapMarker = JSON.parse(savedLayouts);
}

// Start saving the layouts every now and then.
setInterval(() => {
  localStorage.setItem('map-marker', JSON.stringify(mapMarker));
}, 100);

export { mapMarker };
