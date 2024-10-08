use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

use bytes::Buf;
use flate2::read::GzDecoder;
use tar::Archive;

const THREADX_RELEASE: &str = "6.4.1";
const THREADX_SOURCE_URL_BASE: &str =
    "https://github.com/eclipse-threadx/threadx/archive/refs/tags";

static TX_PORT_FILES: &[&str] = &[
    "tx_thread_stack_build.S",
    "tx_thread_schedule.S",
    "tx_thread_system_return.S",
    "tx_thread_context_save.S",
    "tx_thread_context_restore.S",
    "tx_thread_interrupt_control.S",
    "tx_timer_interrupt.S",
];

static TX_COMMON_FILES: &[&str] = &[
    "tx_block_allocate.c",
    "tx_block_pool_cleanup.c",
    "tx_block_pool_create.c",
    "tx_block_pool_delete.c",
    "tx_block_pool_info_get.c",
    "tx_block_pool_initialize.c",
    "tx_block_pool_performance_info_get.c",
    "tx_block_pool_performance_system_info_get.c",
    "tx_block_pool_prioritize.c",
    "tx_block_release.c",
    "tx_byte_allocate.c",
    "tx_byte_pool_cleanup.c",
    "tx_byte_pool_create.c",
    "tx_byte_pool_delete.c",
    "tx_byte_pool_info_get.c",
    "tx_byte_pool_initialize.c",
    "tx_byte_pool_performance_info_get.c",
    "tx_byte_pool_performance_system_info_get.c",
    "tx_byte_pool_prioritize.c",
    "tx_byte_pool_search.c",
    "tx_byte_release.c",
    "tx_event_flags_cleanup.c",
    "tx_event_flags_create.c",
    "tx_event_flags_delete.c",
    "tx_event_flags_get.c",
    "tx_event_flags_info_get.c",
    "tx_event_flags_initialize.c",
    "tx_event_flags_performance_info_get.c",
    "tx_event_flags_performance_system_info_get.c",
    "tx_event_flags_set.c",
    "tx_event_flags_set_notify.c",
    "tx_initialize_high_level.c",
    "tx_initialize_kernel_enter.c",
    "tx_initialize_kernel_setup.c",
    "tx_mutex_cleanup.c",
    "tx_mutex_create.c",
    "tx_mutex_delete.c",
    "tx_mutex_get.c",
    "tx_mutex_info_get.c",
    "tx_mutex_initialize.c",
    "tx_mutex_performance_info_get.c",
    "tx_mutex_performance_system_info_get.c",
    "tx_mutex_prioritize.c",
    "tx_mutex_priority_change.c",
    "tx_mutex_put.c",
    "tx_queue_cleanup.c",
    "tx_queue_create.c",
    "tx_queue_delete.c",
    "tx_queue_flush.c",
    "tx_queue_front_send.c",
    "tx_queue_info_get.c",
    "tx_queue_initialize.c",
    "tx_queue_performance_info_get.c",
    "tx_queue_performance_system_info_get.c",
    "tx_queue_prioritize.c",
    "tx_queue_receive.c",
    "tx_queue_send.c",
    "tx_queue_send_notify.c",
    "tx_semaphore_ceiling_put.c",
    "tx_semaphore_cleanup.c",
    "tx_semaphore_create.c",
    "tx_semaphore_delete.c",
    "tx_semaphore_get.c",
    "tx_semaphore_info_get.c",
    "tx_semaphore_initialize.c",
    "tx_semaphore_performance_info_get.c",
    "tx_semaphore_performance_system_info_get.c",
    "tx_semaphore_prioritize.c",
    "tx_semaphore_put.c",
    "tx_semaphore_put_notify.c",
    "tx_thread_create.c",
    "tx_thread_delete.c",
    "tx_thread_entry_exit_notify.c",
    "tx_thread_identify.c",
    "tx_thread_info_get.c",
    "tx_thread_initialize.c",
    "tx_thread_performance_info_get.c",
    "tx_thread_performance_system_info_get.c",
    "tx_thread_preemption_change.c",
    "tx_thread_priority_change.c",
    "tx_thread_relinquish.c",
    "tx_thread_reset.c",
    "tx_thread_resume.c",
    "tx_thread_shell_entry.c",
    "tx_thread_sleep.c",
    "tx_thread_stack_analyze.c",
    "tx_thread_stack_error_handler.c",
    "tx_thread_stack_error_notify.c",
    "tx_thread_suspend.c",
    "tx_thread_system_preempt_check.c",
    "tx_thread_system_resume.c",
    "tx_thread_system_suspend.c",
    "tx_thread_terminate.c",
    "tx_thread_time_slice.c",
    "tx_thread_time_slice_change.c",
    "tx_thread_timeout.c",
    "tx_thread_wait_abort.c",
    "tx_time_get.c",
    "tx_time_set.c",
    "tx_timer_activate.c",
    "tx_timer_change.c",
    "tx_timer_create.c",
    "tx_timer_deactivate.c",
    "tx_timer_delete.c",
    "tx_timer_expiration_process.c",
    "tx_timer_info_get.c",
    "tx_timer_initialize.c",
    "tx_timer_performance_info_get.c",
    "tx_timer_performance_system_info_get.c",
    "tx_timer_system_activate.c",
    "tx_timer_system_deactivate.c",
    "tx_timer_thread_entry.c",
    "tx_trace_buffer_full_notify.c",
    "tx_trace_enable.c",
    "tx_trace_event_filter.c",
    "tx_trace_event_unfilter.c",
    "tx_trace_disable.c",
    "tx_trace_initialize.c",
    "tx_trace_interrupt_control.c",
    "tx_trace_isr_enter_insert.c",
    "tx_trace_isr_exit_insert.c",
    "tx_trace_object_register.c",
    "tx_trace_object_unregister.c",
    "tx_trace_user_event_insert.c",
    "txe_block_allocate.c",
    "txe_block_pool_create.c",
    "txe_block_pool_delete.c",
    "txe_block_pool_info_get.c",
    "txe_block_pool_prioritize.c",
    "txe_block_release.c",
    "txe_byte_allocate.c",
    "txe_byte_pool_create.c",
    "txe_byte_pool_delete.c",
    "txe_byte_pool_info_get.c",
    "txe_byte_pool_prioritize.c",
    "txe_byte_release.c",
    "txe_event_flags_create.c",
    "txe_event_flags_delete.c",
    "txe_event_flags_get.c",
    "txe_event_flags_info_get.c",
    "txe_event_flags_set.c",
    "txe_event_flags_set_notify.c",
    "txe_mutex_create.c",
    "txe_mutex_delete.c",
    "txe_mutex_get.c",
    "txe_mutex_info_get.c",
    "txe_mutex_prioritize.c",
    "txe_mutex_put.c",
    "txe_queue_create.c",
    "txe_queue_delete.c",
    "txe_queue_flush.c",
    "txe_queue_front_send.c",
    "txe_queue_info_get.c",
    "txe_queue_prioritize.c",
    "txe_queue_receive.c",
    "txe_queue_send.c",
    "txe_queue_send_notify.c",
    "txe_semaphore_ceiling_put.c",
    "txe_semaphore_create.c",
    "txe_semaphore_delete.c",
    "txe_semaphore_get.c",
    "txe_semaphore_info_get.c",
    "txe_semaphore_prioritize.c",
    "txe_semaphore_put.c",
    "txe_semaphore_put_notify.c",
    "txe_thread_create.c",
    "txe_thread_delete.c",
    "txe_thread_entry_exit_notify.c",
    "txe_thread_info_get.c",
    "txe_thread_preemption_change.c",
    "txe_thread_priority_change.c",
    "txe_thread_relinquish.c",
    "txe_thread_reset.c",
    "txe_thread_resume.c",
    "txe_thread_suspend.c",
    "txe_thread_terminate.c",
    "txe_thread_time_slice_change.c",
    "txe_thread_wait_abort.c",
    "txe_timer_activate.c",
    "txe_timer_change.c",
    "txe_timer_create.c",
    "txe_timer_deactivate.c",
    "txe_timer_delete.c",
    "txe_timer_info_get.c",
];

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
    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("stm32mp15xx_m4.ld"))
        .unwrap()
        .write_all(include_bytes!("stm32mp15xx_m4.ld"))
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=stm32mp15xx_m4.ld");

    // Specify linker arguments.

    // `--nmagic` is required if memory section addresses are not aligned to 0x10000,
    // for example the FLASH and RAM sections in your `memory.x`.
    // See https://github.com/rust-embedded/cortex-m-quickstart/pull/95
    println!("cargo:rustc-link-arg=--nmagic");

    // Set the linker script to the one provided by cortex-m-rt.
    println!("cargo:rustc-link-arg=-Tstm32mp15xx_m4.ld");

    let threadx_dir = download_threadx(&out).unwrap();
    let tx_common_dir = threadx_dir.join("common/src");
    let tx_common_inc = threadx_dir.join("common/inc");
    let tx_port_dir = threadx_dir.join("ports/cortex_m4/gnu/src");
    let tx_port_inc = threadx_dir.join("ports/cortex_m4/gnu/inc");
    cc::Build::new()
        .include(&tx_common_inc)
        .include(&tx_port_inc)
        .files(TX_PORT_FILES.iter().map(|&s| tx_port_dir.join(s)))
        .files(TX_COMMON_FILES.iter().map(|&s| tx_common_dir.join(s)))
        .compile("threadx");

    cc::Build::new()
        .file("src/tx_low_level.S")
        .compile("tx_low_level");
    println!("cargo:rerun-if-changed=src/tx_low_level.S");
}
