const waypointColors = ['red', 'yellow', 'green', 'cyan', 'blue', 'magenta'] as const;
type WaypointColor = (typeof waypointColors)[number];

type Waypoint = {
  lat: number;
  lon: number;
  name: string;
  color: WaypointColor;
};

let waypoints: Waypoint[] = [];

// Load waypoints from local storage.
const savedWaypoints = localStorage.getItem('waypoints');
if (savedWaypoints) {
  waypoints = JSON.parse(savedWaypoints);
}

// Save feed config to local storage on update.
window.addEventListener('waypoints-update', () => {
  localStorage.setItem('waypoints', JSON.stringify(waypoints));
});

function addWaypoint(waypoint: Waypoint) {
  waypoints.push(waypoint);
  window.dispatchEvent(new Event('waypoints-update'));
}

function removeWaypoint(waypoint: Waypoint) {
  const index = waypoints.indexOf(waypoint);
  if (index !== -1) {
    waypoints.splice(index, 1);
    window.dispatchEvent(new Event('waypoints-update'));
  }
}

function removeAllWaypoints() {
  waypoints.splice(0, waypoints.length);
  window.dispatchEvent(new Event('waypoints-update'));
}

function exportWaypointsAsText(): string {
  return waypoints
    .map((waypoint) => {
      return `${waypoint.name} ${waypoint.lat.toFixed(8)} ${waypoint.lon.toFixed(8)} ${waypoint.color}`;
    })
    .join('\n');
}

function parseDMS(dms: string): number {
  const [degrees, minutes, seconds, direction] = dms.split(/\s+/);
  let decimal =
    parseFloat(degrees) + parseFloat(minutes) / 60 + parseFloat(seconds) / 3600;
  if (direction === 'S' || direction === 'W') {
    decimal = -decimal;
  }
  return decimal;
}

function parseLatLon(coordinate: string): { lat: number; lon: number } {
  const regexDMS =
    /([NS])\s*(\d+)\s*(\d+)\s*(\d+)\s*([EW])\s*(\d+)\s*(\d+)\s*(\d+)/i;
  const regexDecimal = /([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)/;

  if (regexDMS.test(coordinate)) {
    const [, latDir, latDeg, latMin, latSec, lonDir, lonDeg, lonMin, lonSec] =
      regexDMS.exec(coordinate)!;
    const lat = parseDMS(`${latDeg} ${latMin} ${latSec} ${latDir}`);
    const lon = parseDMS(`${lonDeg} ${lonMin} ${lonSec} ${lonDir}`);
    return { lat, lon };
  } else if (regexDecimal.test(coordinate)) {
    const [, lat, lon] = regexDecimal.exec(coordinate)!;
    return { lat: parseFloat(lat), lon: parseFloat(lon) };
  } else {
    throw new Error('Invalid coordinate format');
  }
}

function importWaypointsFromText(
  text: string,
  defaultColor: WaypointColor = waypointColors[0]
) {
  const newWaypoints: Waypoint[] = [];

  const lines = text
    .split('\n')
    .map((line) => line.trim())
    .filter((line) => line);

  try {
    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];

      // This regex will look for a sequence that matches either the DMS or decimal format
      const coordinateMatch = line.match(
        /\s([NS]\s*\d+\s*\d+\s*\d+\s*[EW]\s*\d+\s*\d+\s*\d+)|\s([-+]?\d*\.?\d+\s+[-+]?\d*\.?\d+)/i
      );

      if (!coordinateMatch) {
        throw new Error('No valid coordinates found in "' + line + '"');
      }

      // Extract the coordinate part from the line
      const coordPart = coordinateMatch[0];
      // Everything before the coordinates is considered the name
      const namePart = line.slice(0, coordinateMatch.index);
      // Rest of the line, either begins with a color, or is another waypoint. (When newlines were replaced with spaces.)
      const restOfLinePart = line.slice(
        coordinateMatch.index + coordPart.length
      );

      const coord = coordPart.trim();
      const name = namePart.trim();
      const restOfLine = restOfLinePart.trim();

      // Start matching the string char by char until we find any match or we exhaust the options.
      // The rest of the string is considered new line and should be pushed to the next iteration.
      let color: WaypointColor = defaultColor;
      let extraContent = restOfLine.trim();
      for (const colorOption of waypointColors) {
        if (restOfLine.startsWith(colorOption)) {
          color = colorOption;
          extraContent = restOfLine.slice(color.length).trim();
          break;
        }
      }
      if (extraContent.length > 0) {
        lines.splice(i + 1, 0, extraContent);
      }

      const { lat, lon } = parseLatLon(coord.trim());

      newWaypoints.push({ lat, lon, name, color });
    }
    waypoints.push(...newWaypoints);
  } catch (error) {
    console.error(error);
    alert('Failed to import waypoints:\n' + error);
  }
  window.dispatchEvent(new Event('waypoints-update'));
}

export {
  waypointColors,
  WaypointColor,
  Waypoint,
  waypoints,
  addWaypoint,
  removeWaypoint,
  removeAllWaypoints,
  exportWaypointsAsText,
  importWaypointsFromText
};
