//    SAPF - Sound As Pure Form
//    Copyright (C) 2019 James McCartney
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

// Platform-agnostic spinlock abstraction
// Uses os_unfair_lock on macOS, std::atomic_flag elsewhere

#if defined(__APPLE__)
    #include <os/lock.h>

    using SpinLockType = os_unfair_lock;
    #define SPINLOCK_INIT OS_UNFAIR_LOCK_INIT

    inline void spinlock_lock(SpinLockType* lock) {
        os_unfair_lock_lock(lock);
    }

    inline void spinlock_unlock(SpinLockType* lock) {
        os_unfair_lock_unlock(lock);
    }

#else
    #include <atomic>

    struct SpinLockType {
        std::atomic_flag flag = ATOMIC_FLAG_INIT;
    };
    #define SPINLOCK_INIT {}

    inline void spinlock_lock(SpinLockType* lock) {
        while (lock->flag.test_and_set(std::memory_order_acquire)) {
            // Spin
        }
    }

    inline void spinlock_unlock(SpinLockType* lock) {
        lock->flag.clear(std::memory_order_release);
    }

#endif

// RAII spinlock guard
class SpinLocker
{
    SpinLockType& lock;
public:
    SpinLocker(SpinLockType& inLock) : lock(inLock) {
        spinlock_lock(&lock);
    }
    ~SpinLocker() {
        spinlock_unlock(&lock);
    }
};
