type Theme = 'dark' | 'light' | 'berry';

let currentTheme: Theme = 'dark';

const savedTheme = localStorage.getItem('theme');
if (savedTheme) {
  currentTheme = JSON.parse(savedTheme);
}

function setTheme(theme: Theme) {
  currentTheme = theme;

  // Set data-theme on root.
  document.documentElement.setAttribute('data-theme', theme);

  // Emit event.
  window.dispatchEvent(new Event('theme-change'));

  // Save to local storage.
  localStorage.setItem('theme', JSON.stringify(theme));
}

// document.documentElement.setAttribute('data-theme', currentTheme);
setTheme(currentTheme);

export { Theme, currentTheme, setTheme };
