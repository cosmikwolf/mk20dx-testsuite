use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Copy the memory.x linker script for MK20DX256 (Teensy 3.1/3.2)
    fs::write(
        out_dir.join("memory.x"),
        include_bytes!("../mk20dx-hal/memory/memory_mk20d7.x"),
    )
    .unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());

    // Both bins and tests use cortex-m-rt's link.x + defmt.x
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rustc-link-arg=-Tdefmt.x");

    println!("cargo:rerun-if-changed=build.rs");
}
