use std::env;
use std::path::{Path, PathBuf};

use bytes::Buf;
use flate2::read::GzDecoder;
use tar::Archive;

const THREADX_RELEASE: &str = "6.4.1";
const THREADX_SOURCE_URL_BASE: &str =
    "https://github.com/eclipse-threadx/threadx/archive/refs/tags";

fn download_threadx(output_dir: &Path) -> Result<PathBuf, anyhow::Error> {
    let source_url = format!("{THREADX_SOURCE_URL_BASE}/v{THREADX_RELEASE}_rel.tar.gz");
    let response = reqwest::blocking::get(source_url)?.bytes()?;
    let decoder = GzDecoder::new(response.reader());
    let mut archive = Archive::new(decoder);

    archive.unpack(&output_dir)?;
    Ok(output_dir
        .to_owned()
        .join(format!("threadx-{THREADX_RELEASE}_rel")))
}

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let threadx_path = download_threadx(&out).unwrap();
    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}", threadx_path.join("common/inc").display()))
        .clang_arg(format!(
            "-I{}",
            threadx_path.join("ports/cortex_m4/gnu/inc").display()
        ))
        .clang_arg("-I./include")
        .clang_arg("-nostdinc")
        .clang_arg("--target=arm")
        .clang_arg("-mthumb")
        .clang_arg("-mcpu=cortex-m4")
        .clang_arg("-mfloat-abi=soft")
        .use_core()
        .allowlist_function("tx_.*")
        .allowlist_function("_tx_.*")
        .allowlist_type("TX_.*")
        .allowlist_var("TX_.*")
        .allowlist_var("TX_AUTO_START")
        .formatter(bindgen::Formatter::Rustfmt)
        .generate()
        .expect("Unable to generate bindings");

    let rust_source = bindings.to_string();
    let bindings_out_path = out.join("bindings.rs");
    std::fs::write(bindings_out_path, rust_source).expect("Couldn't write updated bindings");
}
