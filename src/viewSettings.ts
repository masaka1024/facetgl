import type { Projection } from './cameraRig';
import { languageSetting, setLanguageSetting, t, type LanguageSetting } from './i18n';

export interface ViewSettings {
  edgeScale: number;      // outline width in pixels per 1.0 of PMX edge size (0 = off)
  projection: Projection;
  fov: number;            // perspective field of view in degrees
}

// MMD's default camera view angle is 30 degrees, and MMD-compatible renderers draw an
// edge of size 1.0 one pixel wide.
export const DEFAULT_VIEW_SETTINGS: ViewSettings = {
  edgeScale: 1,
  projection: 'perspective',
  fov: 30,
};

// v2: edgeScale changed from a multiplier of MMDLoader's thickness to pixels per edge size
const STORAGE_KEY = 'facetgl.viewSettings.v2';

export function loadViewSettings(): ViewSettings {
  try {
    const saved = JSON.parse(localStorage.getItem(STORAGE_KEY) ?? 'null');
    if (!saved || typeof saved !== 'object') return { ...DEFAULT_VIEW_SETTINGS };
    return {
      edgeScale: clampNumber(saved.edgeScale, 0, 4, DEFAULT_VIEW_SETTINGS.edgeScale),
      projection: saved.projection === 'orthographic' ? 'orthographic' : 'perspective',
      fov: clampNumber(saved.fov, 10, 90, DEFAULT_VIEW_SETTINGS.fov),
    };
  } catch {
    // Storage can be unavailable; the defaults still work
    return { ...DEFAULT_VIEW_SETTINGS };
  }
}

function saveViewSettings(settings: ViewSettings) {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(settings));
  } catch {
    // Not being able to remember the settings is not an error worth showing
  }
}

function clampNumber(value: unknown, min: number, max: number, fallback: number) {
  return typeof value === 'number' && Number.isFinite(value) ? Math.min(max, Math.max(min, value)) : fallback;
}

// Wires the "表示設定" popover. onChange receives the full settings after every edit.
export function setupViewSettingsPanel(initial: ViewSettings, onChange: (settings: ViewSettings) => void) {
  const settings = { ...initial };
  const button = document.getElementById('view-button')!;
  const panel = document.getElementById('view-panel')!;
  const edge = document.getElementById('edge-scale') as HTMLInputElement;
  const edgeValue = document.getElementById('edge-scale-value')!;
  const fov = document.getElementById('fov') as HTMLInputElement;
  const fovValue = document.getElementById('fov-value')!;
  const projectionButtons = [...panel.querySelectorAll<HTMLButtonElement>('[data-projection]')];
  const languageButtons = [...panel.querySelectorAll<HTMLButtonElement>('[data-language]')];

  const render = () => {
    edge.value = String(settings.edgeScale);
    edgeValue.textContent = settings.edgeScale === 0 ? t('view.edgeOff') : `${settings.edgeScale.toFixed(1)} px`;
    fov.value = String(settings.fov);
    fovValue.textContent = `${settings.fov}°`;
    fov.disabled = settings.projection === 'orthographic';
    for (const b of projectionButtons) b.classList.toggle('active', b.dataset.projection === settings.projection);
    for (const b of languageButtons) b.classList.toggle('active', b.dataset.language === languageSetting());
  };

  const update = (changes: Partial<ViewSettings>) => {
    Object.assign(settings, changes);
    render();
    saveViewSettings(settings);
    onChange({ ...settings });
  };

  button.addEventListener('click', () => {
    const open = panel.classList.toggle('open');
    button.classList.toggle('active', open);
  });
  edge.addEventListener('input', () => update({ edgeScale: Number(edge.value) }));
  fov.addEventListener('input', () => update({ fov: Number(fov.value) }));
  for (const b of projectionButtons) {
    b.addEventListener('click', () => update({ projection: b.dataset.projection as Projection }));
  }
  // The language is kept apart from the view settings (初期値に戻す does not change it)
  for (const b of languageButtons) {
    b.addEventListener('click', () => {
      setLanguageSetting(b.dataset.language as LanguageSetting);
      render();
    });
  }
  document.getElementById('view-reset')!.addEventListener('click', () => update({ ...DEFAULT_VIEW_SETTINGS }));

  render();
  return {
    refresh: render, // after the language changes
    toggleProjection: () =>
      update({ projection: settings.projection === 'perspective' ? 'orthographic' : 'perspective' }),
  };
}
