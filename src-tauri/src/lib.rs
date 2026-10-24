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

// File commands are `async` so that they run off the main thread: reading a big model or
// listing folders on a slow network drive (NAS) must not freeze the window.

/// Reads a local file and returns its raw bytes to the frontend.
#[tauri::command(async)]
fn read_file(path: String) -> Result<Response, String> {
  std::fs::read(&path)
    .map(Response::new)
    .map_err(|e| format!("{path}: {e}"))
}

/// Returns the size of a local file in bytes.
#[tauri::command(async)]
fn file_size(path: String) -> Result<u64, String> {
  std::fs::metadata(&path)
    .map(|m| m.len())
    .map_err(|e| format!("{path}: {e}"))
}

/// Limits for `list_files`, so that a huge folder or a slow network drive cannot keep it busy
const LIST_MAX_FILES: usize = 5000;
const LIST_MAX_DEPTH: u32 = 3;
const LIST_TIME_LIMIT: std::time::Duration = std::time::Duration::from_secs(5);

/// Lists files under a folder (a few levels deep) as relative paths with "/" separators.
/// Used to find textures that models reference from a different subfolder.
/// Stops early (returning what it found so far) at LIST_MAX_FILES files or LIST_TIME_LIMIT.
#[tauri::command(async)]
fn list_files(dir: String) -> Vec<String> {
  list_files_limited(std::path::Path::new(&dir), LIST_MAX_FILES, LIST_TIME_LIMIT)
}

fn list_files_limited(
  base: &std::path::Path,
  max_files: usize,
  time_limit: std::time::Duration,
) -> Vec<String> {
  struct Walk<'a> {
    base: &'a std::path::Path,
    max_files: usize,
    deadline: std::time::Instant,
    out: Vec<String>,
  }
  impl Walk<'_> {
    fn full(&self) -> bool {
      self.out.len() >= self.max_files || std::time::Instant::now() >= self.deadline
    }
    fn walk(&mut self, dir: &std::path::Path, depth: u32) {
      // Checked before reading the folder, so nothing more is read once the limit is hit
      if self.full() {
        return;
      }
      let Ok(entries) = std::fs::read_dir(dir) else { return };
      for entry in entries.flatten() {
        if self.full() {
          return;
        }
        // file_type() comes with the folder listing (no extra request per file on a NAS)
        // and does not follow links, so linked folders cannot loop
        let Ok(kind) = entry.file_type() else { continue };
        let path = entry.path();
        if kind.is_dir() {
          if depth < LIST_MAX_DEPTH {
            self.walk(&path, depth + 1);
          }
        } else if let Ok(rel) = path.strip_prefix(self.base) {
          self.out.push(rel.to_string_lossy().replace('\\', "/"));
        }
      }
    }
  }
  let mut walk = Walk {
    base,
    max_files,
    deadline: std::time::Instant::now() + time_limit,
    out: Vec::new(),
  };
  walk.walk(base, 0);
  walk.out
}

#[cfg(test)]
mod tests {
  use super::*;
  use std::time::Duration;

  fn temp_tree(name: &str) -> std::path::PathBuf {
    let root = std::env::temp_dir().join(format!("facetgl-test-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&root);
    std::fs::create_dir_all(root.join("tex/sub/deeper/toodeep")).unwrap();
    std::fs::write(root.join("model.pmx"), b"").unwrap();
    std::fs::write(root.join("tex/a.png"), b"").unwrap();
    std::fs::write(root.join("tex/sub/b.png"), b"").unwrap();
    std::fs::write(root.join("tex/sub/deeper/c.png"), b"").unwrap();
    std::fs::write(root.join("tex/sub/deeper/toodeep/d.png"), b"").unwrap();
    root
  }

  #[test]
  fn lists_files_three_levels_deep() {
    let root = temp_tree("depth");
    let mut files = list_files_limited(&root, 5000, Duration::from_secs(5));
    files.sort();
    assert_eq!(files, ["model.pmx", "tex/a.png", "tex/sub/b.png", "tex/sub/deeper/c.png"]);
    std::fs::remove_dir_all(&root).unwrap();
  }

  #[test]
  fn stops_at_file_limit() {
    let root = temp_tree("limit");
    assert_eq!(list_files_limited(&root, 2, Duration::from_secs(5)).len(), 2);
    std::fs::remove_dir_all(&root).unwrap();
  }

  #[test]
  fn stops_at_time_limit() {
    let root = temp_tree("time");
    assert!(list_files_limited(&root, 5000, Duration::ZERO).is_empty());
    std::fs::remove_dir_all(&root).unwrap();
  }
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
