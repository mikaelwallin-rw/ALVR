mod average;
mod c_api;
mod connection_result;
pub mod inputs;
mod logging;
mod primitives;
mod version;

use parking_lot::{Condvar, Mutex, RwLockWriteGuard};
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

pub use anyhow;
pub use glam;
pub use log;
pub use parking_lot;
pub use semver;
pub use settings_schema;

pub use average::*;
pub use c_api::*;
pub use connection_result::*;
pub use inputs::*;
pub use log::{debug, error, info, warn};
pub use logging::*;
pub use primitives::*;
pub use version::*;

pub const ALVR_NAME: &str = "ALVR";

// Simple wrapper for AtomicBool when using Ordering::Relaxed. Deref cannot be implemented (cannot
// return local reference)
#[derive(Default)]
pub struct RelaxedAtomic(AtomicBool);

impl RelaxedAtomic {
    pub const fn new(initial_value: bool) -> Self {
        Self(AtomicBool::new(initial_value))
    }

    pub fn value(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }

    pub fn set(&self, value: bool) {
        self.0.store(value, Ordering::Relaxed);
    }
}

#[derive(PartialEq, Eq, Debug)]
pub enum LifecycleState {
    StartingUp,
    Idle,
    Resumed,
    ShuttingDown,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Eq, Default)]
pub enum ConnectionState {
    #[default]
    Disconnected,
    Connecting,
    Connected,
    Streaming,
    Disconnecting,
}

pub fn wait_rwlock<T>(condvar: &Condvar, guard: &mut RwLockWriteGuard<'_, T>) {
    let staging_mutex = Mutex::<()>::new(());
    let mut inner_guard = staging_mutex.lock();
    RwLockWriteGuard::unlocked(guard, move || {
        condvar.wait(&mut inner_guard);
    });
}

/// Timeout-bounded variant of [`wait_rwlock`].
///
/// Same semantics as [`wait_rwlock`] -- the outer write guard is released
/// while the thread parks on the Condvar -- but the wait returns after at
/// most `timeout` even if no notification arrives. Intended to be used in a
/// predicate loop:
///
/// ```ignore
/// while !shutdown_complete(&*guard) {
///     wait_rwlock_timeout(&cond, &mut guard, Duration::from_millis(500));
/// }
/// ```
///
/// This pattern is immune to the lost-wakeup race that [`wait_rwlock`] has:
/// between releasing the outer RwLock and entering `condvar.wait()`, a
/// `notify_one` call can fire and be dropped (Condvar wakeups are
/// edge-triggered and not buffered). With a timeout-bounded wait, even if
/// every wakeup is lost, the predicate is re-evaluated within `timeout` and
/// the loop exits as soon as the authoritative state says so.
pub fn wait_rwlock_timeout<T>(
    condvar: &Condvar,
    guard: &mut RwLockWriteGuard<'_, T>,
    timeout: Duration,
) {
    let staging_mutex = Mutex::<()>::new(());
    let mut inner_guard = staging_mutex.lock();
    RwLockWriteGuard::unlocked(guard, move || {
        // Discard the WaitTimeoutResult -- the caller's predicate loop is the
        // real authority on whether to keep waiting.
        let _ = condvar.wait_for(&mut inner_guard, timeout);
    });
}
