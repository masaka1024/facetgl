use tauri::ipc::Response;

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

/// Returns the file passed on the command line (e.g. via file association).
/// The frontend checks whether its format is supported.
#[tauri::command]
fn startup_file() -> Option<String> {
  std::env::args()
    .skip(1)
    .find(|a| !a.starts_with('-') && std::path::Path::new(a).is_file())
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
  tauri::Builder::default()
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
    .run(tauri::generate_context!())
    .expect("error while building tauri application");
}
