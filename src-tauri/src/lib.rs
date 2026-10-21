use std::sync::Mutex;

use tauri::ipc::Response;
#[cfg(any(target_os = "macos", target_os = "ios"))]
use tauri::{Emitter, Manager};

/// Files opened from the OS (macOS sends them as an "Opened" event instead of command-line
/// arguments). Until the frontend has asked for its startup file, they are kept here;
/// after that they are sent to it as "open-file" events.
#[derive(Default)]
struct OpenedFiles {
  frontend_ready: bool,
  pending: Option<String>,
}

/// Reads a local file and returns its raw bytes to the frontend.
#[tauri::command]
fn read_file(path: String) -> Result<Response, String> {
  std::fs::read(&path)
    .map(Response::new)
    .map_err(|e| format!("{path}: {e}"))
}

/// Returns the size of a local file in bytes.
#[tauri::command]
fn file_size(path: String) -> Result<u64, String> {
  std::fs::metadata(&path)
    .map(|m| m.len())
    .map_err(|e| format!("{path}: {e}"))
}

/// Lists files under a folder (a few levels deep) as relative paths with "/" separators.
/// Used to find textures that models reference from a different subfolder.
#[tauri::command]
fn list_files(dir: String) -> Vec<String> {
  fn walk(base: &std::path::Path, dir: &std::path::Path, depth: u32, out: &mut Vec<String>) {
    let Ok(entries) = std::fs::read_dir(dir) else { return };
    for entry in entries.flatten() {
      if out.len() >= 5000 {
        return;
      }
      let path = entry.path();
      if path.is_dir() {
        if depth < 3 {
          walk(base, &path, depth + 1, out);
        }
      } else if let Ok(rel) = path.strip_prefix(base) {
        out.push(rel.to_string_lossy().replace('\\', "/"));
      }
    }
  }
  let mut out = Vec::new();
  let base = std::path::Path::new(&dir);
  walk(base, base, 0, &mut out);
  out
}

/// Returns the file passed on launch: on the command line (Windows file association)
/// or through the OS "Opened" event (macOS). The frontend checks whether its format is supported.
#[tauri::command]
fn startup_file(opened: tauri::State<'_, Mutex<OpenedFiles>>) -> Option<String> {
  let mut opened = opened.lock().unwrap();
  opened.frontend_ready = true;
  opened.pending.take().or_else(|| {
    std::env::args()
      .skip(1)
      .find(|a| !a.starts_with('-') && std::path::Path::new(a).is_file())
  })
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
  tauri::Builder::default()
    .manage(Mutex::new(OpenedFiles::default()))
    .plugin(tauri_plugin_dialog::init())
    .setup(|app| {
      if cfg!(debug_assertions) {
        app.handle().plugin(
          tauri_plugin_log::Builder::default()
            .level(log::LevelFilter::Info)
            .build(),
        )?;
      }
      Ok(())
    })
    .invoke_handler(tauri::generate_handler![read_file, file_size, list_files, startup_file])
    .build(tauri::generate_context!())
    .expect("error while building tauri application")
    .run(|_app, _event| {
      // macOS: double-click in Finder, "Open With", or a drop on the Dock icon
      #[cfg(any(target_os = "macos", target_os = "ios"))]
      if let tauri::RunEvent::Opened { urls } = _event {
        let Some(path) = urls
          .iter()
          .filter_map(|u| u.to_file_path().ok())
          .find(|p| p.is_file())
          .map(|p| p.to_string_lossy().into_owned())
        else {
          return;
        };
        let state = _app.state::<Mutex<OpenedFiles>>();
        let mut opened = state.lock().unwrap();
        if opened.frontend_ready {
          let _ = _app.emit("open-file", path);
        } else {
          opened.pending = Some(path);
        }
      }
    });
}
