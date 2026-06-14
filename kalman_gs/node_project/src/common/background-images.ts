export type BackgroundImage = {
  name: string;
  dataUrl: string;
};

const BACKGROUND_IMAGES_KEY = 'panel-manager-background-images';
const LEGACY_BACKGROUND_IMAGE_KEY = 'panel-manager-background-image';
const LEGACY_BACKGROUND_IMAGE_NAME_KEY = 'panel-manager-background-image-name';

function isBackgroundImage(value: unknown): value is BackgroundImage {
  return (
    typeof value === 'object' &&
    value !== null &&
    typeof (value as BackgroundImage).name === 'string' &&
    typeof (value as BackgroundImage).dataUrl === 'string'
  );
}

export function getBackgroundImages(): BackgroundImage[] {
  const storedImages = localStorage.getItem(BACKGROUND_IMAGES_KEY);

  if (storedImages) {
    try {
      const parsedImages = JSON.parse(storedImages);
      if (Array.isArray(parsedImages)) {
        return parsedImages.filter(isBackgroundImage);
      }
    } catch {
      return [];
    }
  }

  const legacyImage = localStorage.getItem(LEGACY_BACKGROUND_IMAGE_KEY);
  const legacyImageName = localStorage.getItem(LEGACY_BACKGROUND_IMAGE_NAME_KEY);
  if (!legacyImage || !legacyImageName) {
    return [];
  }

  return [
    {
      name: legacyImageName,
      dataUrl: legacyImage
    }
  ];
}

export function setBackgroundImages(images: BackgroundImage[]) {
  if (images.length > 0) {
    localStorage.setItem(BACKGROUND_IMAGES_KEY, JSON.stringify(images));
  } else {
    localStorage.removeItem(BACKGROUND_IMAGES_KEY);
  }

  localStorage.removeItem(LEGACY_BACKGROUND_IMAGE_KEY);
  localStorage.removeItem(LEGACY_BACKGROUND_IMAGE_NAME_KEY);
}

export function getBackgroundImagesLabel(images = getBackgroundImages()) {
  if (images.length === 0) {
    return null;
  }
  if (images.length === 1) {
    return images[0].name;
  }
  return `${images.length} images`;
}

export function getRandomBackgroundImage() {
  const images = getBackgroundImages();
  if (images.length === 0) {
    return null;
  }

  return images[Math.floor(Math.random() * images.length)];
}
