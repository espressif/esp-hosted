/*
 * SPDX-FileCopyrightText: 2015-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * nvs_flash.c — filesystem-backed NVS for non-IDF hosts.
 *
 * Each (partition, namespace) pair is stored as a fixed-record binary
 * file under `.nvs/` (default "./.nvs/" in the cwd; overridable via
 * CONFIG_ESP_RMAKER_FACTORY_DATA_DIR).  Records are
 * sized by the static `nvs_entry_t` layout in this file (key + value +
 * type, fixed lengths).  Updates are atomic via temp-file + rename;
 * thread safety is provided by per-handle and global pthread mutexes.
 */

/* Adapted from _upstream_posix_port/posix/src/os_posix_nvs.c for the
 * ESP-Hosted port/esp_idf_port/nvs_flash/ slot.  Same filesystem-
 * backed approach (NVS data persists under $HOME/.nvs/<namespace>/
 * or similar — see upstream impl for path handling).
 *
 * Mechanical changes vs upstream:
 *   - `os_nvs_*` API and types renamed to `nvs_*` (drops upstream's
 *     `os_` prefix per ESP-Hosted's IDF-shaped public surface).
 *   - `OS_NVS_*` enum values / error macros renamed to `NVS_*`.
 *   - Added `nvs_flash_init` / `nvs_flash_erase` thin wrappers around
 *     `nvs_init` / `nvs_erase_all` to match the IDF nvs_flash.h shape
 *     that linux_user examples actually call.
 *   - Body uses raw POSIX (pthread, open/read/write/lseek/close, rename,
 *     fsync) directly — no `os_*` primitive helpers were used upstream
 *     and no `eh_host_port_*` substitutions are needed for non-IDF hosts
 *     (port/host/posix/ runs on POSIX natively).
 *   - Internal types (`nvs_handle_t`, `nvs_iterator_t`, etc.) are kept
 *     local to this TU; the only header-visible surface is
 *     `nvs_flash_init` / `nvs_flash_erase` declared in
 *     host/compat/include/nvs_flash.h.
 */

/* Feature test macros must precede any system header include.
 * `_GNU_SOURCE` enables `pthread_mutex_timedlock`, `usleep`, `strnlen`
 * on glibc; equivalent to upstream's implicit assumption of a Linux
 * userspace target. */
#ifndef _GNU_SOURCE
#  define _GNU_SOURCE 1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

#include "esp_err.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>
#include "nvs_flash.h"
#include "sdkconfig.h"

/* ─────────────────────────── NVS error codes ─────────────────────────── */

#define NVS_SUCCESS 0
#define NVS_FAIL    -1

#ifndef ESP_ERR_NVS_BASE
#  define ESP_ERR_NVS_BASE 0x1100
#endif
#ifndef ESP_ERR_NVS_NOT_INITIALIZED
#  define ESP_ERR_NVS_NOT_INITIALIZED 0x1101
#endif
#ifndef ESP_ERR_NVS_NOT_FOUND
#  define ESP_ERR_NVS_NOT_FOUND 0x1102
#endif
#ifndef ESP_ERR_NVS_TYPE_MISMATCH
#  define ESP_ERR_NVS_TYPE_MISMATCH 0x1103
#endif
#ifndef ESP_ERR_NVS_READ_ONLY
#  define ESP_ERR_NVS_READ_ONLY 0x1104
#endif
#ifndef ESP_ERR_NVS_NOT_ENOUGH_SPACE
#  define ESP_ERR_NVS_NOT_ENOUGH_SPACE 0x1105
#endif
#ifndef ESP_ERR_NVS_INVALID_NAME
#  define ESP_ERR_NVS_INVALID_NAME 0x1106
#endif
#ifndef ESP_ERR_NVS_INVALID_HANDLE
#  define ESP_ERR_NVS_INVALID_HANDLE 0x1107
#endif
#ifndef ESP_ERR_NVS_WRITE_FAILED
#  define ESP_ERR_NVS_WRITE_FAILED 0x1108
#endif
#ifndef ESP_ERR_NVS_KEY_TOO_LONG
#  define ESP_ERR_NVS_KEY_TOO_LONG 0x1109
#endif
#ifndef ESP_ERR_NVS_INVALID_STATE
#  define ESP_ERR_NVS_INVALID_STATE 0x110b
#endif
#ifndef ESP_ERR_NVS_INVALID_LENGTH
#  define ESP_ERR_NVS_INVALID_LENGTH 0x110c
#endif

/* ──────────────────────── Layout / type definitions ───────────────────── */

#define MAX_KEY_LENGTH          256
#define MAX_VALUE_LENGTH        1024
#define NVS_FILE_PREFIX         "nvs_data-"
#define MAX_NVS_FILENAME_LENGTH 512

typedef enum {
    NVS_OPEN_MODE_READ_ONLY = 0,
    NVS_OPEN_MODE_READ_WRITE = 1,
} nvs_open_mode_t;

typedef enum {
    NVS_TYPE_ANY = 0,
    NVS_TYPE_STRING,
    NVS_TYPE_BLOB,
    NVS_TYPE_U8,
    NVS_TYPE_U16,
    NVS_TYPE_U32,
} nvs_type_t;

typedef struct nvs_handle_s {
    int fd;
    bool read_only;
    int ref_count;
    char filename[MAX_NVS_FILENAME_LENGTH];
    pthread_mutex_t mutex;
    struct nvs_handle_s* next;
} nvs_handle_t;

typedef struct {
    char key[MAX_KEY_LENGTH];
    size_t key_length;
    char value[MAX_VALUE_LENGTH];
    size_t value_length;
    nvs_type_t type;
} nvs_entry_t;

typedef struct {
    char key[MAX_KEY_LENGTH];
    size_t key_length;
    size_t value_length;
    nvs_type_t type;
} nvs_entry_info_t;

typedef struct nvs_iterator_s {
    nvs_handle_t* handle;
    nvs_entry_t current_entry;
    ssize_t current_position;
    ssize_t total_entries;
} nvs_iterator_t;

/* ───────────────────────────── State ──────────────────────────────────── */

static const char *TAG = "NVS"; /* Log tag */
static struct nvs_handle_s* opened_files = NULL;

/* Mutex for thread safety */
static pthread_mutex_t nvs_global_mutex;
static pthread_mutex_t handle_list_mutex;
static bool nvs_initialized = false;

static pthread_mutex_t nvs_key_mutex;

/* Add new file state mutex */
static pthread_mutex_t file_state_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Add this one constant near other static variables at top */
static const int MAX_FD_RETRIES = 3;

/* Add global file state lock timeout value in milliseconds */
static const int FILE_STATE_LOCK_TIMEOUT_MS = 100;

/* Sibling-of-build/ persistent dir.  Hidden + short to match the
 * pattern noted at the top of this file.  Overridable at compile
 * time via CONFIG_ESP_RMAKER_FACTORY_DATA_DIR (RMaker convention). */
#ifndef CONFIG_ESP_RMAKER_FACTORY_DATA_DIR
static const char* nvs_persistent_dir = ".nvs";
#else
static const char* nvs_persistent_dir = CONFIG_ESP_RMAKER_FACTORY_DATA_DIR;
#endif

/* Forward decls for the internal API used across functions below. */
static esp_err_t nvs_init(void);
static esp_err_t nvs_erase_all(void);
static esp_err_t nvs_open_from_partition(const char *part_name,
                                         const char *namespace_name,
                                         nvs_open_mode_t open_mode,
                                         nvs_handle_t **out_handle);
static esp_err_t nvs_get(nvs_handle_t* handle, const char* key, void* value,
                         size_t* length, nvs_type_t type);
static esp_err_t nvs_set(nvs_handle_t* handle, const char* key,
                         const void* value, size_t length, nvs_type_t type);

/* ─────────────────────────── Helpers ──────────────────────────────────── */

/* Helper for timed mutex locking */
static esp_err_t timed_mutex_lock(pthread_mutex_t *mutex, int timeout_ms) {
    if (!mutex) {
        return ESP_ERR_INVALID_ARG;
    }

    if (timeout_ms <= 0) {
        /* Regular lock without timeout */
        if (pthread_mutex_lock(mutex) != 0) {
            return ESP_FAIL;
        }
        return ESP_OK;
    }

    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (timeout_ms % 1000) * 1000000;

    /* Handle nanosecond overflow */
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
    }

    int ret = pthread_mutex_timedlock(mutex, &ts);
    if (ret != 0) {
        if (ret == ETIMEDOUT) {
            return ESP_ERR_TIMEOUT;
        }
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* Helper to validate a file descriptor */
static bool is_valid_fd(int fd) {
    if (fd < 0) {
        return false;
    }

    /* First check if fd is still valid using fcntl */
    int ret = fcntl(fd, F_GETFD);
    if (ret == -1) {
        ESP_LOGD(TAG, "File descriptor %d is invalid: %s", fd, strerror(errno));
        return false;
    }

    /* Next, verify we can seek on the descriptor */
    off_t original_pos = lseek(fd, 0, SEEK_CUR);
    if (original_pos < 0) {
        ESP_LOGD(TAG, "Cannot get position for fd %d: %s", fd, strerror(errno));
        return false;
    }

    /* Try to seek to start - this is a non-destructive test */
    if (lseek(fd, 0, SEEK_SET) < 0) {
        ESP_LOGD(TAG, "Cannot seek to start for fd %d: %s", fd, strerror(errno));
        return false;
    }

    /* Restore original position */
    if (lseek(fd, original_pos, SEEK_SET) < 0) {
        ESP_LOGW(TAG, "Failed to restore original position for fd %d", fd);
        return false;
    }

    return true;
}

static void create_nvs_persistent_dir(void) {
    pthread_mutex_lock(&nvs_global_mutex);
    struct stat st = {0};
    if (stat(nvs_persistent_dir, &st) == -1) {
        mkdir(nvs_persistent_dir, 0700);
    }
    pthread_mutex_unlock(&nvs_global_mutex);
}

static nvs_handle_t* find_existing_handle(const char* part_name, const char* namespace_name) {
    nvs_handle_t* current = opened_files;
    char filename[MAX_NVS_FILENAME_LENGTH];

    if (snprintf(filename, sizeof(filename), "%s/%s%s-%s.bin",
                 nvs_persistent_dir, NVS_FILE_PREFIX, part_name, namespace_name) >= (int)sizeof(filename)) {
        return NULL;
    }

    while (current) {
        if (strcmp(current->filename, filename) == 0) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

/* Reference counting functions */
static esp_err_t nvs_handle_ref_inc(nvs_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    pthread_mutex_lock(&handle->mutex);
    handle->ref_count++;
    pthread_mutex_unlock(&handle->mutex);

    return ESP_OK;
}

static esp_err_t nvs_handle_ref_dec(nvs_handle_t *handle, bool *should_free) {
    if (!handle || !should_free) {
        return ESP_ERR_INVALID_ARG;
    }

    pthread_mutex_lock(&handle->mutex);
    handle->ref_count--;
    *should_free = (handle->ref_count == 0);
    pthread_mutex_unlock(&handle->mutex);

    return ESP_OK;
}

static esp_err_t nvs_init(void) {
    if (nvs_initialized) {
        return ESP_OK;
    }

    if (pthread_mutex_init(&nvs_global_mutex, NULL) != 0) {
        return ESP_FAIL;
    }

    if (pthread_mutex_init(&handle_list_mutex, NULL) != 0) {
        pthread_mutex_destroy(&nvs_global_mutex);
        return ESP_FAIL;
    }

    if (pthread_mutex_init(&nvs_key_mutex, NULL) != 0) {
        pthread_mutex_destroy(&handle_list_mutex);
        pthread_mutex_destroy(&nvs_global_mutex);
        return ESP_FAIL;
    }

    if (pthread_mutex_init(&file_state_mutex, NULL) != 0) {
        pthread_mutex_destroy(&nvs_key_mutex);
        pthread_mutex_destroy(&handle_list_mutex);
        pthread_mutex_destroy(&nvs_global_mutex);
        return ESP_FAIL;
    }

    nvs_initialized = true;
    create_nvs_persistent_dir();
    return ESP_OK;
}

static esp_err_t nvs_deinit(void) {
    if (!nvs_initialized) {
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Deinitializing NVS...");

    /* Lock all mutexes to ensure no transactions are in progress */
    pthread_mutex_lock(&nvs_global_mutex);
    pthread_mutex_lock(&handle_list_mutex);
    pthread_mutex_lock(&nvs_key_mutex);
    pthread_mutex_lock(&file_state_mutex);

    nvs_handle_t* current = opened_files;
    nvs_handle_t* next = NULL;

    while (current) {
        next = current->next;
        if (current->fd >= 0) {
            ESP_LOGI(TAG, "Closing file descriptor %d for %s", current->fd, current->filename);
            fsync(current->fd);  /* Flush any pending writes */
            close(current->fd);
            current->fd = -1;
        }
        pthread_mutex_destroy(&current->mutex);
        free(current);
        current = next;
    }
    opened_files = NULL;

    /* Release mutexes in reverse order of acquisition */
    pthread_mutex_unlock(&file_state_mutex);
    pthread_mutex_unlock(&nvs_key_mutex);
    pthread_mutex_unlock(&handle_list_mutex);
    pthread_mutex_unlock(&nvs_global_mutex);

    pthread_mutex_destroy(&file_state_mutex);
    pthread_mutex_destroy(&nvs_key_mutex);
    pthread_mutex_destroy(&handle_list_mutex);
    pthread_mutex_destroy(&nvs_global_mutex);

    nvs_initialized = false;
    ESP_LOGD(TAG, "NVS deinitialized successfully");
    return ESP_OK;
}

static esp_err_t nvs_flash_init_partition(const char* partition_name) {
    if (!partition_name) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t nvs_open(const char* namespace_, nvs_open_mode_t mode, nvs_handle_t** handle) {
    const char* partition = "nvs";

    if (!namespace_ || !handle) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_open_from_partition(partition, namespace_, mode, handle);
}

static esp_err_t nvs_close(nvs_handle_t *handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    pthread_mutex_lock(&handle_list_mutex);
    /* Find handle in opened_files list */
    nvs_handle_t* current = opened_files;
    nvs_handle_t* prev = NULL;
    bool found = false;
    bool should_free = false;

    while (current) {
        if (current == handle) {
            found = true;
            esp_err_t err = nvs_handle_ref_dec(current, &should_free);
            if (err != ESP_OK) {
                pthread_mutex_unlock(&handle_list_mutex);
                return err;
            }

            if (should_free) {  /* Only remove if no more references */
                if (prev) {
                    prev->next = current->next;
                } else {
                    opened_files = current->next;
                }
                if (current->fd >= 0) {
                    close(current->fd);
                }
                pthread_mutex_destroy(&current->mutex);
                free(current);
            }
            break;
        }
        prev = current;
        current = current->next;
    }

    pthread_mutex_unlock(&handle_list_mutex);
    if (!found) {
        return ESP_ERR_NVS_INVALID_HANDLE;
    }

    return ESP_OK;
}

static esp_err_t search_key(nvs_handle_t* handle, const char* key, nvs_type_t type, nvs_entry_t* entry) {
    if (!handle || !key || !entry) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize the entry structure to all zeros for safety */
    memset(entry, 0, sizeof(nvs_entry_t));

    /* Validate file descriptor */
    if (handle->fd < 0) {
        ESP_LOGE(TAG, "Invalid file descriptor in search_key");
        return ESP_ERR_NVS_INVALID_HANDLE;
    }

    /* Verify file descriptor is still valid */
    if (!is_valid_fd(handle->fd)) {
        ESP_LOGE(TAG, "File descriptor %d is no longer valid in search_key", handle->fd);
        return ESP_ERR_NVS_INVALID_HANDLE;
    }

    /* Reset file position with retry if needed */
    for (int retry = 0; retry < 3; retry++) {
        if (lseek(handle->fd, 0, SEEK_SET) >= 0) {
            break;
        }

        /* Failed to seek, check if we should retry */
        if (retry == 2) {
            ESP_LOGE(TAG, "Failed to seek to start of file after retries: %s", strerror(errno));
            return ESP_ERR_NVS_INVALID_STATE;
        }

        /* Brief pause before retry */
        usleep(5000 * (retry + 1));
    }

    ssize_t bytes_read;
    off_t current_pos = 0;
    char key_buffer[MAX_KEY_LENGTH + 1]; /* +1 for null terminator */
    size_t key_len = strlen(key);

    /* Validate input key length */
    if (key_len > MAX_KEY_LENGTH) {
        ESP_LOGE(TAG, "Key '%s' exceeds maximum length", key);
        return ESP_ERR_NVS_KEY_TOO_LONG;
    }

    while (true) {
        /* Read the next entry */
        bytes_read = read(handle->fd, entry, sizeof(nvs_entry_t));

        /* Check for read errors */
        if (bytes_read < 0) {
            ESP_LOGE(TAG, "Error reading from NVS file: %s", strerror(errno));
            return ESP_ERR_NVS_INVALID_STATE;
        }

        /* Check if we've reached the end of the file */
        if (bytes_read == 0) {
            break;
        }

        /* Check if we got a partial read */
        if (bytes_read < (ssize_t)sizeof(nvs_entry_t)) {
            ESP_LOGE(TAG, "Partial read from NVS file: got %zd bytes, expected %zu",
                     bytes_read, sizeof(nvs_entry_t));
            return ESP_ERR_NVS_INVALID_STATE;
        }

        current_pos += bytes_read;

        /* Validate entry before using it */
        if (entry->key_length == 0 || entry->key_length > MAX_KEY_LENGTH || entry->value_length > MAX_VALUE_LENGTH) {
            ESP_LOGW(TAG, "Invalid entry at position %ld: key_len=%zu, value_len=%zu",
                    (long)current_pos, entry->key_length, entry->value_length);
            continue;
        }

        /* Make a null-terminated copy of the key for safe comparison */
        if (entry->key_length <= MAX_KEY_LENGTH) {
            memcpy(key_buffer, entry->key, entry->key_length);
            key_buffer[entry->key_length] = '\0';

            /* Exact key match required */
            if (entry->key_length == key_len && strcmp(key_buffer, key) == 0) {
                /* Type checking */
                if (type != NVS_TYPE_ANY && entry->type != type) {
                    ESP_LOGW(TAG, "Type mismatch for key '%s': expected %d, got %d",
                             key, type, entry->type);
                    return ESP_ERR_NVS_TYPE_MISMATCH;
                }

                /* Verify value validity */
                if (entry->value_length == 0) {
                    ESP_LOGW(TAG, "Found key '%s' but value length is 0", key);
                }

                ESP_LOGD(TAG, "Key '%s' found with type %d, value length %zu",
                         key, entry->type, entry->value_length);
                return ESP_OK;
            }
        }
    }

    ESP_LOGD(TAG, "Key '%s' not found", key);
    return ESP_ERR_NVS_NOT_FOUND;
}

static esp_err_t reopen_file_descriptor(nvs_handle_t* handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* No additional locks - the caller should have already locked handle->mutex */

    /* Check if the file descriptor needs reopening */
    if (!is_valid_fd(handle->fd)) {
        ESP_LOGW(TAG, "File descriptor %d is invalid, reopening %s", handle->fd, handle->filename);

        /* Always close the old descriptor if it exists */
        if (handle->fd >= 0) {
            close(handle->fd);
            handle->fd = -1;
        }

        /* Improved open with retry and verification */
        for (int retry = 0; retry < MAX_FD_RETRIES; retry++) {
            int flags = handle->read_only ? O_RDONLY : (O_RDWR | O_CREAT);
            handle->fd = open(handle->filename, flags, 0644);

            if (handle->fd >= 0) {
                /* First reset position to start of file */
                if (lseek(handle->fd, 0, SEEK_SET) < 0) {
                    ESP_LOGE(TAG, "Failed to seek to start after open: %s", strerror(errno));
                    close(handle->fd);
                    handle->fd = -1;
                    continue;  /* Try again */
                }

                /* Verify file content with a simple read test */
                char verify_buf[16]; /* Larger buffer for better verification */
                ssize_t read_bytes = read(handle->fd, verify_buf, sizeof(verify_buf));

                /* Even if read returns 0 (empty file), that's valid */
                if (read_bytes < 0) {
                    ESP_LOGE(TAG, "Read verification failed: %s", strerror(errno));
                    close(handle->fd);
                    handle->fd = -1;
                    continue;  /* Try again */
                }

                /* Always reset position after verification read */
                if (lseek(handle->fd, 0, SEEK_SET) < 0) {
                    ESP_LOGE(TAG, "Failed to reset position after verification: %s", strerror(errno));
                    close(handle->fd);
                    handle->fd = -1;
                    continue;  /* Try again */
                }

                /* Use fsync to ensure the file state is consistent */
                if (!handle->read_only) {
                    if (fsync(handle->fd) < 0) {
                        ESP_LOGW(TAG, "fsync after reopen failed: %s", strerror(errno));
                        /* Not critical, we can continue */
                    }
                }

                ESP_LOGI(TAG, "Successfully reopened and verified %s with fd %d",
                         handle->filename, handle->fd);
                return ESP_OK;
            }

            /* Exponential backoff before retry */
            usleep(10000 * (1 << retry));
        }

        ESP_LOGE(TAG, "All reopen attempts failed for %s", handle->filename);
        return ESP_ERR_NVS_INVALID_HANDLE;
    }

    /* File descriptor exists, but verify it's fully functional */
    if (lseek(handle->fd, 0, SEEK_SET) < 0) {
        ESP_LOGE(TAG, "Existing file descriptor not seekable: %s", strerror(errno));
        close(handle->fd);
        handle->fd = -1;
        /* Try to reopen */
        return reopen_file_descriptor(handle);
    }

    return ESP_OK;
}

/* Transaction support for thread safety */
static esp_err_t nvs_begin_transaction(nvs_handle_t* handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Establish consistent locking order to prevent deadlocks:
     * 1. First acquire nvs_key_mutex (global transaction lock)
     * 2. Then acquire handle-specific mutex
     */
    ESP_LOGD(TAG, "Attempting to begin transaction for %s", handle->filename);

    /* Use shorter timeout and retry approach for global mutex */
    for (int retry = 0; retry < 3; retry++) {
        /* Progressive timeout to prevent all threads waiting same time */
        int timeout_ms = 200 * (retry + 1);
        esp_err_t lock_result = timed_mutex_lock(&nvs_key_mutex, timeout_ms);

        if (lock_result == ESP_OK) {
            /* Global lock acquired, now try handle lock */
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;  /* 1 second timeout for handle lock */

            if (pthread_mutex_timedlock(&handle->mutex, &ts) == 0) {
                /* Both locks acquired, proceed with file validation */
                break;
            } else {
                /* Could not get handle lock, release global lock and retry */
                pthread_mutex_unlock(&nvs_key_mutex);
                ESP_LOGW(TAG, "Could not acquire handle mutex for %s, retrying", handle->filename);

                /* Backoff before retry */
                usleep(10000 * (retry + 1));

                if (retry == 2) {  /* Last retry */
                    ESP_LOGE(TAG, "Timeout acquiring handle mutex after multiple retries");
                    return ESP_ERR_TIMEOUT;
                }
                continue;
            }
        }

        if (retry == 2) {  /* Last retry */
            ESP_LOGE(TAG, "Timeout acquiring global transaction mutex after multiple retries");
            return ESP_ERR_TIMEOUT;
        }

        /* Backoff before retry */
        usleep(10000 * (retry + 1));
    }

    /* At this point we have both locks, check file descriptor */
    if (handle->fd < 0 || !is_valid_fd(handle->fd)) {
        ESP_LOGW(TAG, "File descriptor %d invalid during transaction begin, reopening", handle->fd);

        /* Use reopen_file_descriptor for consistent handling */
        esp_err_t err = reopen_file_descriptor(handle);
        if (err != ESP_OK) {
            /* Release locks in correct order */
            pthread_mutex_unlock(&handle->mutex);
            pthread_mutex_unlock(&nvs_key_mutex);
            ESP_LOGE(TAG, "Failed to reopen file %s during transaction begin", handle->filename);
            return err;
        }
    }

    ESP_LOGD(TAG, "Transaction began successfully for %s with fd %d",
             handle->filename, handle->fd);
    return ESP_OK;
}

static esp_err_t nvs_end_transaction(nvs_handle_t* handle, bool commit) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t result = ESP_OK;
    bool fd_was_reopened = false;

    /* First verify handle and file descriptor are valid */
    if (handle->fd < 0) {
        /* Invalid descriptor, can't commit */
        ESP_LOGW(TAG, "Invalid file descriptor in end_transaction");
        result = ESP_ERR_NVS_INVALID_HANDLE;
    } else if (commit && !handle->read_only) {
        /* Only perform fsync for write operations if we're committing */
        if (!is_valid_fd(handle->fd)) {
            /* File descriptor became invalid, try to reopen before commit */
            ESP_LOGW(TAG, "File descriptor %d became invalid before commit, attempting reopen",
                    handle->fd);

            /* We already have the locks, so we can use this simplified reopen call */
            if (handle->fd >= 0) {
                close(handle->fd);
                handle->fd = -1;
            }

            /* Open for writing */
            handle->fd = open(handle->filename, O_RDWR, 0644);
            if (handle->fd < 0) {
                ESP_LOGE(TAG, "Failed to reopen file for commit: %s", strerror(errno));
                result = ESP_ERR_NVS_INVALID_HANDLE;
            } else {
                fd_was_reopened = true;
                ESP_LOGI(TAG, "Successfully reopened file %s with fd %d for commit",
                         handle->filename, handle->fd);
            }
        }

        /* Perform fsync if file descriptor is valid */
        if (handle->fd >= 0) {
            /* If we reopened, we don't need to fsync as the file would have just been written */
            if (!fd_was_reopened && fsync(handle->fd) < 0) {
                ESP_LOGE(TAG, "Failed to fsync transaction for %s: %s",
                         handle->filename, strerror(errno));
                result = ESP_ERR_NVS_WRITE_FAILED;
            } else {
                ESP_LOGD(TAG, "Transaction committed successfully for %s", handle->filename);
            }
        }
    }

    /* Always release locks in reverse order of acquisition */
    pthread_mutex_unlock(&handle->mutex);
    pthread_mutex_unlock(&nvs_key_mutex);

    ESP_LOGD(TAG, "Transaction ended for %s with result %d", handle->filename, result);
    return result;
}

static esp_err_t nvs_debug_verify_ref_counts(void) {
    bool all_valid = true;
    ESP_LOGI(TAG, "Starting ref count verification...");

    /* Use trylock to avoid deadlocks */
    if (pthread_mutex_trylock(&handle_list_mutex) != 0) {
        ESP_LOGW(TAG, "Cannot verify ref counts - handle_list_mutex is locked");
        return ESP_ERR_TIMEOUT;
    }

    nvs_handle_t* current = opened_files;
    while (current) {
        /* Use trylock to avoid deadlocks */
        if (pthread_mutex_trylock(&current->mutex) != 0) {
            ESP_LOGW(TAG, "Cannot verify handle %s - mutex is locked",
                     current->filename);
            pthread_mutex_unlock(&handle_list_mutex);
            return ESP_ERR_TIMEOUT;
        }

        if (current->ref_count <= 0) {
            ESP_LOGE(TAG, "Invalid reference count %d for handle %s",
                     current->ref_count, current->filename);
            all_valid = false;
        } else {
            ESP_LOGI(TAG, "Valid reference count %d for handle %s",
                     current->ref_count, current->filename);
        }
        pthread_mutex_unlock(&current->mutex);
        current = current->next;
    }

    pthread_mutex_unlock(&handle_list_mutex);
    ESP_LOGI(TAG, "Ref count verification complete: %s",
             all_valid ? "all valid" : "some invalid");
    return all_valid ? ESP_OK : ESP_FAIL;
}

static esp_err_t nvs_get(nvs_handle_t* handle, const char* key, void* value, size_t* length, nvs_type_t type) {
    if (!handle || !key || !length) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Use transaction approach for thread safety */
    esp_err_t result = nvs_begin_transaction(handle);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Failed to begin transaction: %d", result);
        return result;
    }

    int retry_count = 0;
    const int max_retries = 3;
    bool should_release_mutex = true;

    do {
        /* Validate file descriptor before each operation */
        if (handle->fd < 0 || !is_valid_fd(handle->fd)) {
            ESP_LOGW(TAG, "Invalid file descriptor %d in nvs_get for key %s, attempting recovery",
                    handle->fd, key);

            /* Reopen the file within the existing transaction */
            esp_err_t reopen_err = reopen_file_descriptor(handle);
            if (reopen_err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reopen file in nvs_get: %d", reopen_err);
                result = reopen_err;
                break;
            }

            /* Successfully reopened, continue with operation */
            ESP_LOGI(TAG, "Successfully reopened file with fd %d in nvs_get", handle->fd);
        }

        /* Create a local copy of the entry for isolation */
        nvs_entry_t entry;
        memset(&entry, 0, sizeof(entry));

        /* Search for the key with the valid file descriptor */
        result = search_key(handle, key, type, &entry);

        if (result == ESP_OK) {
            /* Key found, verify entry is valid before using it */
            if (entry.key_length == 0 || entry.key_length > MAX_KEY_LENGTH ||
                entry.value_length > MAX_VALUE_LENGTH) {
                ESP_LOGW(TAG, "Found corrupt entry for key %s: key_len=%zu, value_len=%zu",
                        key, entry.key_length, entry.value_length);
                result = ESP_ERR_NVS_INVALID_STATE;

                /* Don't retry on corruption */
                break;
            }

            /* Copy value data if requested */
            if (value) {
                if (entry.value_length > *length) {
                    result = ESP_ERR_NVS_INVALID_LENGTH;
                    break;
                }

                /* Double-check value integrity */
                if (entry.value_length > 0) {
                    memcpy(value, entry.value, entry.value_length);
                }
            }

            /* Successfully found and copied the value */
            *length = entry.value_length;
            ESP_LOGD(TAG, "Successfully read key %s with type %d, value length %zu",
                     key, entry.type, entry.value_length);
            break;
        } else if (result == ESP_ERR_NVS_NOT_FOUND) {
            /* Key legitimately not found, no need to retry */
            ESP_LOGD(TAG, "Key %s not found in NVS", key);
            break;
        } else {
            /* Some error occurred during search, may warrant retry */
            ESP_LOGW(TAG, "Error %d searching for key %s, retry %d",
                     result, key, retry_count);

            /* For IO errors, try with a fresh transaction */
            if (result == ESP_ERR_NVS_INVALID_HANDLE || result == ESP_ERR_NVS_INVALID_STATE) {
                /* End current transaction and start a new one */
                nvs_end_transaction(handle, false);
                should_release_mutex = false;

                /* Backoff with exponential delay */
                usleep(10000 * (1 << retry_count));

                /* Begin a fresh transaction */
                result = nvs_begin_transaction(handle);
                if (result != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to restart transaction in nvs_get: %d", result);
                    return result;
                }

                should_release_mutex = true;
            } else {
                /* For other errors, just backoff and retry with same transaction */
                usleep(5000 * (1 << retry_count));
            }
        }

        retry_count++;
    } while (retry_count < max_retries);

    /* End transaction, read-only so no commit needed */
    if (should_release_mutex) {
        esp_err_t tx_result = nvs_end_transaction(handle, false);
        /* Return the more significant error if both failed */
        return (result != ESP_OK) ? result : tx_result;
    }

    return result;
}

static esp_err_t nvs_set(nvs_handle_t* handle, const char* key, const void* value, size_t length, nvs_type_t type) {
    if (!handle || !key || !value || length > MAX_VALUE_LENGTH) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Begin transaction - will lock mutexes in proper order */
    esp_err_t result = nvs_begin_transaction(handle);
    if (result != ESP_OK) {
        ESP_LOGW(TAG, "Failed to begin transaction in set: %d", result);
        return result;
    }

    /* Additional file state lock with timeout - using shorter timeout to prevent deadlocks */
    esp_err_t lock_result = timed_mutex_lock(&file_state_mutex, FILE_STATE_LOCK_TIMEOUT_MS);
    if (lock_result != ESP_OK) {
        ESP_LOGW(TAG, "Failed to acquire file state mutex for write operation: %d", lock_result);
        /* Release transaction locks and fail if we can't get file state lock */
        nvs_end_transaction(handle, false);
        return ESP_ERR_NVS_WRITE_FAILED;
    }

    /* Check if handle is read-only */
    if (handle->read_only) {
        result = ESP_ERR_NVS_READ_ONLY;
        goto cleanup;
    }

    /* Verify file descriptor is still valid */
    if (handle->fd < 0 || !is_valid_fd(handle->fd)) {
        ESP_LOGW(TAG, "File descriptor %d for %s is invalid before write, reopening",
                 handle->fd, handle->filename);

        /* Try reopening without releasing main transaction locks */
        if (handle->fd >= 0) {
            close(handle->fd);
            handle->fd = -1;
        }

        int flags = O_RDWR | O_CREAT;
        handle->fd = open(handle->filename, flags, 0644);
        if (handle->fd < 0) {
            ESP_LOGE(TAG, "Failed to reopen file for write: %s", strerror(errno));
            result = ESP_ERR_NVS_INVALID_HANDLE;
            goto cleanup;
        }
    }

    /* Check key length */
    size_t key_len = strlen(key);
    if (key_len > MAX_KEY_LENGTH) {
        result = ESP_ERR_NVS_KEY_TOO_LONG;
        goto cleanup;
    }

    /* Create a temporary file for atomic update */
    char temp_filename[MAX_NVS_FILENAME_LENGTH];
    if (snprintf(temp_filename, sizeof(temp_filename), "%s.tmp", handle->filename) >= (int)sizeof(temp_filename)) {
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    /* Open temporary file */
    int temp_fd = open(temp_filename, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (temp_fd < 0) {
        ESP_LOGE(TAG, "Failed to open temp file %s: %s", temp_filename, strerror(errno));
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    /* Verify file descriptor again - could have changed */
    if (handle->fd < 0) {
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_INVALID_HANDLE;
        goto cleanup;
    }

    /* Copy all entries except the one we're updating */
    nvs_entry_t entry;

    /* Reset file position - check if it succeeds */
    if (lseek(handle->fd, 0, SEEK_SET) < 0) {
        ESP_LOGE(TAG, "Failed to reset file position for %s: %s",
                 handle->filename, strerror(errno));
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    ssize_t bytes_read;
    while ((bytes_read = read(handle->fd, &entry, sizeof(nvs_entry_t))) == sizeof(nvs_entry_t)) {
        /* Skip invalid entries */
        if (entry.key_length > MAX_KEY_LENGTH || entry.value_length > MAX_VALUE_LENGTH) {
            ESP_LOGW(TAG, "Skipping invalid entry during migration: key_len=%zu, value_len=%zu",
                   entry.key_length, entry.value_length);
            continue;
        }

        /* Skip the entry we're about to update */
        if (entry.key_length == key_len && strncmp(entry.key, key, key_len) == 0) {
            continue;
        }

        if (write(temp_fd, &entry, sizeof(nvs_entry_t)) != sizeof(nvs_entry_t)) {
            ESP_LOGE(TAG, "Failed to write entry to temp file: %s", strerror(errno));
            close(temp_fd);
            unlink(temp_filename);
            result = ESP_ERR_NVS_WRITE_FAILED;
            goto cleanup;
        }
    }

    /* Write the new entry */
    nvs_entry_t new_entry;
    memset(&new_entry, 0, sizeof(new_entry));
    strncpy(new_entry.key, key, key_len);
    new_entry.key[key_len] = '\0';
    new_entry.key_length = key_len;
    memcpy(new_entry.value, value, length);
    new_entry.value_length = length;
    new_entry.type = type;

    if (write(temp_fd, &new_entry, sizeof(nvs_entry_t)) != sizeof(nvs_entry_t)) {
        ESP_LOGE(TAG, "Failed to write new entry to temp file: %s", strerror(errno));
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    /* Ensure all data is written */
    if (fsync(temp_fd) < 0) {
        ESP_LOGE(TAG, "Failed to fsync temp file: %s", strerror(errno));
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    /* Close temporary file */
    close(temp_fd);

    /* Close original file */
    if (handle->fd >= 0) {
        close(handle->fd);
        handle->fd = -1;
    }

    /* Atomic rename - the critical operation */
    if (rename(temp_filename, handle->filename) != 0) {
        ESP_LOGE(TAG, "Failed to rename temp file to original: %s", strerror(errno));
        unlink(temp_filename);  /* Try to clean up */
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    /* Reopen the file with a retry mechanism */
    int retry_count = 0;
    while (retry_count < MAX_FD_RETRIES) {
        handle->fd = open(handle->filename, O_RDWR, 0644);
        if (handle->fd >= 0 && is_valid_fd(handle->fd)) {
            break;
        }

        if (handle->fd >= 0) {
            close(handle->fd);
            handle->fd = -1;
        }

        retry_count++;
        usleep(10000 * retry_count);  /* Increasing backoff */
    }

    if (handle->fd < 0) {
        ESP_LOGE(TAG, "Failed to reopen file after rename: %s", strerror(errno));
        result = ESP_ERR_NVS_WRITE_FAILED;
    }

cleanup:
    /* Always release file state mutex if we acquired it */
    if (lock_result == ESP_OK) {
        pthread_mutex_unlock(&file_state_mutex);
    }

    /* End transaction - will release transaction mutexes */
    esp_err_t tx_result = nvs_end_transaction(handle, (result == ESP_OK));

    /* Return the more serious error if both operations failed */
    return (result != ESP_OK) ? result : tx_result;
}

static esp_err_t nvs_commit(nvs_handle_t* handle) {
    (void)handle;
    return ESP_OK;
}

static esp_err_t nvs_set_value(nvs_handle_t* handle, const char* key, const void* value, nvs_type_t type) {
    if (!handle || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_entry_t entry;
    strncpy(entry.key, key, MAX_KEY_LENGTH);
    entry.key_length = strnlen(entry.key, MAX_KEY_LENGTH);

    entry.value_length = sizeof(value);
    memcpy(entry.value, value, entry.value_length);
    entry.type = type;

    lseek(handle->fd, 0, SEEK_END);

    if (write(handle->fd, &entry, sizeof(nvs_entry_t)) < 0) {
        ESP_LOGE(TAG, "Failed to write to NVS: %s", strerror(errno));
        return ESP_ERR_NVS_WRITE_FAILED;
    }

    return ESP_OK;
}

static esp_err_t nvs_erase(nvs_handle_t* handle, const char* key) {
    if (!handle || !key) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Begin transaction */
    esp_err_t result = nvs_begin_transaction(handle);
    if (result != ESP_OK) {
        return result;
    }

    /* Lock additional mutex for file operations */
    pthread_mutex_lock(&file_state_mutex);

    if (handle->read_only) {
        result = ESP_ERR_NVS_READ_ONLY;
        goto cleanup;
    }

    char temp_filename[MAX_NVS_FILENAME_LENGTH];
    if (snprintf(temp_filename, sizeof(temp_filename), "%s.tmp", handle->filename) >= (int)sizeof(temp_filename)) {
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    int temp_fd = open(temp_filename, O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (temp_fd < 0) {
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    nvs_entry_t entry;
    bool found = false;
    size_t key_len = strlen(key);

    // Reset file position to start
    if (lseek(handle->fd, 0, SEEK_SET) < 0) {
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    // Copy all entries except the one to be erased
    ssize_t bytes_read;
    while ((bytes_read = read(handle->fd, &entry, sizeof(nvs_entry_t))) == sizeof(nvs_entry_t)) {
        if (entry.key_length == key_len &&
            strncmp(entry.key, key, entry.key_length) == 0) {
            found = true;
            continue;
        }
        if (write(temp_fd, &entry, sizeof(nvs_entry_t)) != sizeof(nvs_entry_t)) {
            close(temp_fd);
            unlink(temp_filename);
            result = ESP_ERR_NVS_WRITE_FAILED;
            goto cleanup;
        }
    }

    if (bytes_read < 0 && !found) {
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_NOT_FOUND;
        goto cleanup;
    }

    // Ensure all data is written
    if (fsync(temp_fd) < 0) {
        close(temp_fd);
        unlink(temp_filename);
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    // Close both files
    close(temp_fd);
    close(handle->fd);
    handle->fd = -1;

    // Replace the original file with the temporary one
    if (rename(temp_filename, handle->filename) != 0) {
        result = ESP_ERR_NVS_WRITE_FAILED;
        goto cleanup;
    }

    // Reopen the file
    handle->fd = open(handle->filename, O_RDWR, 0644);
    if (handle->fd < 0) {
        result = ESP_ERR_NVS_WRITE_FAILED;
    }

cleanup:
    pthread_mutex_unlock(&file_state_mutex);
    /* End transaction */
    esp_err_t tx_result = nvs_end_transaction(handle, (result == ESP_OK));
    return (result != ESP_OK) ? result : tx_result;
}

static esp_err_t nvs_erase_all(void) {
    nvs_handle_t* current = opened_files;
    while (current) {
        close(current->fd);
        remove(current->filename);
        current = current->next;
    }

    return ESP_OK;
}

static esp_err_t nvs_get_str(nvs_handle_t* handle, const char* key, char* value, size_t* length) {
    if (!handle || !key || !length) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_entry_t entry;
    esp_err_t err = search_key(handle, key, NVS_TYPE_STRING, &entry);
    if (err != ESP_OK) {
        return err;  // Return ESP_ERR_NVS_TYPE_MISMATCH or ESP_ERR_NVS_NOT_FOUND
    }

    if (value) {
        if (entry.value_length > *length) {
            return ESP_ERR_NVS_INVALID_LENGTH;
        }
        memcpy(value, entry.value, entry.value_length);
        value[entry.value_length] = '\0'; // Null-terminate
    }
    *length = entry.value_length;
    return ESP_OK;
}

static esp_err_t nvs_set_str(nvs_handle_t* handle, const char* key, const char* value) {
    if (!handle || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    return nvs_set(handle, key, value, strlen(value) + 1, NVS_TYPE_STRING);
}

static esp_err_t nvs_get_blob(nvs_handle_t* handle, const char* key, void* value, size_t* length) {
    if (!handle || !key || !length) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if this is a chunked blob
    char meta_key[MAX_KEY_LENGTH];
    snprintf(meta_key, sizeof(meta_key), "%s.meta", key);

    struct {
        size_t total_size;
        int num_chunks;
    } meta;
    size_t meta_size = sizeof(meta);

    if (nvs_get(handle, meta_key, &meta, &meta_size, NVS_TYPE_BLOB) == ESP_OK) {
        if (!value) {
            *length = meta.total_size;
            return ESP_OK;
        }
        // This is a chunked blob
        if (*length < meta.total_size) {
            *length = meta.total_size;
            return ESP_ERR_NVS_INVALID_LENGTH;
        }

        uint8_t* data = (uint8_t*)value;
        size_t total_read = 0;

        for (int i = 0; i < meta.num_chunks; i++) {
            char chunk_key[MAX_KEY_LENGTH];
            snprintf(chunk_key, sizeof(chunk_key), "%s.%d", key, i);

            size_t chunk_size = (meta.total_size - total_read > MAX_VALUE_LENGTH) ?
                               MAX_VALUE_LENGTH : meta.total_size - total_read;

            esp_err_t err = nvs_get(handle, chunk_key, data + total_read, &chunk_size, NVS_TYPE_BLOB);
            if (err != ESP_OK) {
                return err;
            }
            total_read += chunk_size;
        }

        *length = meta.total_size;
        return ESP_OK;
    }

    // Not a chunked blob, handle normally
    return nvs_get(handle, key, value, length, NVS_TYPE_BLOB);
}

static esp_err_t nvs_set_blob(nvs_handle_t* handle, const char* key, const void* value, size_t length) {
    if (!handle || !key || !value) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Check maximum blob size */
    if (length > MAX_VALUE_LENGTH) {
        ESP_LOGE(TAG, "Blob size %zu exceeds maximum allowed size %d", length, MAX_VALUE_LENGTH);
        return ESP_ERR_NVS_INVALID_LENGTH;
    }

    /* Use transaction approach for thread safety */
    esp_err_t result = nvs_begin_transaction(handle);
    if (result != ESP_OK) {
        return result;
    }

    /* Write blob using standard set function, which will be handled in its own transaction */
    pthread_mutex_unlock(&handle->mutex);
    pthread_mutex_unlock(&nvs_key_mutex);

    /* Call the regular set function which will handle its own transaction */
    result = nvs_set(handle, key, value, length, NVS_TYPE_BLOB);

    return result;
}

static esp_err_t nvs_get_u8(nvs_handle_t* handle, const char* key, uint8_t* value) {
    size_t length = sizeof(uint8_t);
    return nvs_get(handle, key, value, &length, NVS_TYPE_U8);
}

static esp_err_t nvs_set_u8(nvs_handle_t* handle, const char* key, uint8_t value) {
    return nvs_set(handle, key, &value, sizeof(uint8_t), NVS_TYPE_U8);
}

static esp_err_t nvs_get_u16(nvs_handle_t* handle, const char* key, uint16_t* value) {
    size_t length = sizeof(uint16_t);
    return nvs_get(handle, key, value, &length, NVS_TYPE_U16);
}

static esp_err_t nvs_set_u16(nvs_handle_t* handle, const char* key, uint16_t value) {
    return nvs_set(handle, key, &value, sizeof(uint16_t), NVS_TYPE_U16);
}

static esp_err_t nvs_get_u32(nvs_handle_t* handle, const char* key, uint32_t* value) {
    size_t length = sizeof(uint32_t);
    return nvs_get(handle, key, value, &length, NVS_TYPE_U32);
}

static esp_err_t nvs_set_u32(nvs_handle_t* handle, const char* key, uint32_t value) {
    return nvs_set(handle, key, &value, sizeof(uint32_t), NVS_TYPE_U32);
}

static esp_err_t nvs_open_from_partition(const char *part_name, const char *namespace_name,
                                         nvs_open_mode_t open_mode, nvs_handle_t **out_handle) {
    if (!part_name || !namespace_name || !out_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!nvs_initialized) {
        return ESP_ERR_NVS_NOT_INITIALIZED;
    }

    pthread_mutex_lock(&handle_list_mutex);

    /* Check for existing handle first */
    nvs_handle_t* existing = find_existing_handle(part_name, namespace_name);
    if (existing) {
        nvs_handle_ref_inc(existing);
        *out_handle = existing;
        pthread_mutex_unlock(&handle_list_mutex);
        return ESP_OK;
    }

    nvs_handle_t* h = calloc(1, sizeof(nvs_handle_t));
    if (!h) {
        pthread_mutex_unlock(&handle_list_mutex);
        return ESP_ERR_NO_MEM;
    }

    if (pthread_mutex_init(&h->mutex, NULL) != 0) {
        free(h);
        pthread_mutex_unlock(&handle_list_mutex);
        return ESP_FAIL;
    }

    h->fd = -1;
    h->next = NULL;
    h->read_only = (open_mode == NVS_OPEN_MODE_READ_ONLY);
    h->ref_count = 1;  // Initialize reference count

    create_nvs_persistent_dir();

    if (snprintf(h->filename, sizeof(h->filename), "%s/%s%s-%s.bin",
                 nvs_persistent_dir, NVS_FILE_PREFIX, part_name, namespace_name) >= (int)sizeof(h->filename)) {
        ESP_LOGE(TAG, "Filename too long for handle");
        free(h);
        pthread_mutex_unlock(&handle_list_mutex);
        return ESP_ERR_NVS_WRITE_FAILED;
    }

    int flags = (open_mode == NVS_OPEN_MODE_READ_WRITE) ?
                (O_RDWR | O_CREAT) : O_RDONLY;

    h->fd = open(h->filename, flags, 0644);
    if (h->fd < 0) {
        ESP_LOGW(TAG, "Failed to open '%s': %s", h->filename, strerror(errno));
        free(h);
        pthread_mutex_unlock(&handle_list_mutex);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    /* Add new handle to list */
    h->next = opened_files;
    opened_files = h;
    *out_handle = h;

    pthread_mutex_unlock(&handle_list_mutex);
    return ESP_OK;
}

static esp_err_t nvs_entry_find(const char *part_name, const char *namespace_name, nvs_type_t type, nvs_iterator_t **output_iterator) {
    if (!part_name || !output_iterator) {
        return ESP_ERR_INVALID_ARG;
    }

    pthread_mutex_lock(&nvs_global_mutex);

    struct stat st = {0};
    if (stat(nvs_persistent_dir, &st) == -1) {
        ESP_LOGE(TAG, "Persistent directory does not exist: %s", nvs_persistent_dir);
        pthread_mutex_unlock(&nvs_global_mutex);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    nvs_handle_t* handle = malloc(sizeof(nvs_handle_t));
    if (!handle) {
        pthread_mutex_unlock(&nvs_global_mutex);
        return ESP_ERR_NO_MEM;
    }

    /* Initialize mutex for handle */
    if (pthread_mutex_init(&handle->mutex, NULL) != 0) {
        free(handle);
        pthread_mutex_unlock(&nvs_global_mutex);
        return ESP_FAIL;
    }

    handle->fd = -1;
    handle->read_only = true;
    handle->ref_count = 1;

    if (snprintf(handle->filename, sizeof(handle->filename), "%s/%s%s-%s.bin",
                nvs_persistent_dir, NVS_FILE_PREFIX, part_name,
                namespace_name ? namespace_name : "") >= (int)sizeof(handle->filename)) {
        ESP_LOGE(TAG, "Filename too long for handle");
        pthread_mutex_destroy(&handle->mutex);
        free(handle);
        pthread_mutex_unlock(&nvs_global_mutex);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    /* Open file with retries */
    int retry_count = 0;
    while (retry_count < MAX_FD_RETRIES) {
        handle->fd = open(handle->filename, O_RDONLY);
        if (handle->fd >= 0) {
            break;
        }
        retry_count++;
        usleep(10000 * retry_count);
    }

    if (handle->fd < 0) {
        ESP_LOGW(TAG, "Failed to open '%s': err:%s. May be not created yet",
                 handle->filename, strerror(errno));
        pthread_mutex_destroy(&handle->mutex);
        free(handle);
        pthread_mutex_unlock(&nvs_global_mutex);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    nvs_iterator_t* iterator = malloc(sizeof(nvs_iterator_t));
    if (!iterator) {
        close(handle->fd);
        pthread_mutex_destroy(&handle->mutex);
        free(handle);
        pthread_mutex_unlock(&nvs_global_mutex);
        return ESP_ERR_NO_MEM;
    }

    iterator->handle = handle;
    iterator->current_position = 0;
    iterator->total_entries = 0;

    pthread_mutex_lock(&handle->mutex);
    nvs_entry_t entry;
    while (read(handle->fd, &entry, sizeof(nvs_entry_t)) > 0) {
        if (type == NVS_TYPE_ANY || entry.type == type) {
            iterator->total_entries++;
        }
    }
    lseek(handle->fd, 0, SEEK_SET);
    pthread_mutex_unlock(&handle->mutex);

    *output_iterator = iterator;
    pthread_mutex_unlock(&nvs_global_mutex);
    return ESP_OK;
}

static esp_err_t nvs_entry_find_in_handle(nvs_handle_t *handle, nvs_type_t type, nvs_iterator_t **output_iterator) {
    if (!handle || !output_iterator) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_iterator_t* iterator = malloc(sizeof(nvs_iterator_t));
    if (!iterator) {
        return ESP_ERR_NO_MEM;
    }
    iterator->handle = handle;
    iterator->current_position = 0;
    iterator->total_entries = 0;

    nvs_entry_t entry;
    while (read(handle->fd, &entry, sizeof(nvs_entry_t)) > 0) {
        if (type == NVS_TYPE_ANY || entry.type == type) {
            iterator->total_entries++;
        }
    }
    lseek(handle->fd, 0, SEEK_SET);

    *output_iterator = iterator;
    return ESP_OK;
}

static esp_err_t nvs_entry_next(nvs_iterator_t *iterator) {
    if (!iterator || !iterator->handle) {
        return ESP_ERR_INVALID_ARG;
    }

    pthread_mutex_lock(&iterator->handle->mutex);

    /* Verify file descriptor */
    if (iterator->handle->fd < 0) {
        /* Try to reopen */
        iterator->handle->fd = open(iterator->handle->filename, O_RDONLY);
        if (iterator->handle->fd < 0) {
            pthread_mutex_unlock(&iterator->handle->mutex);
            return ESP_ERR_NVS_INVALID_HANDLE;
        }
        /* Seek to current position */
        lseek(iterator->handle->fd, iterator->current_position * sizeof(nvs_entry_t), SEEK_SET);
    }

    nvs_entry_t entry;
    ssize_t bytesRead = read(iterator->handle->fd, &entry, sizeof(nvs_entry_t));
    if (bytesRead <= 0) {
        pthread_mutex_unlock(&iterator->handle->mutex);
        return ESP_ERR_NVS_NOT_FOUND;
    }

    iterator->current_entry = entry;
    iterator->current_position++;
    pthread_mutex_unlock(&iterator->handle->mutex);
    return ESP_OK;
}

static esp_err_t nvs_entry_info(const nvs_iterator_t iterator, nvs_entry_info_t *out_info) {
    if (!out_info) {
        return ESP_ERR_INVALID_ARG;
    }

    out_info->key_length = iterator.current_entry.key_length;
    out_info->value_length = iterator.current_entry.value_length;
    out_info->type = iterator.current_entry.type;
    strncpy(out_info->key, iterator.current_entry.key, MAX_KEY_LENGTH);
    return ESP_OK;
}

static void nvs_release_iterator(nvs_iterator_t *iterator) {
    if (iterator) {
        if (iterator->handle) {
            close(iterator->handle->fd);
            free(iterator->handle);
        }
        free(iterator);
    }
}

/* ───────────────────── IDF nvs_flash.h public surface ─────────────────── */

esp_err_t nvs_flash_init(void) {
    return nvs_init();
}

esp_err_t nvs_flash_erase(void) {
    return nvs_erase_all();
}

/* Suppress unused-static warnings for the broader handle-based API kept
 * here for future use (callers route through wifi_os_adapter / esp_wifi
 * shim today; direct linux_user examples only call the two _flash_
 * entrypoints above). */
__attribute__((unused)) static void *_nvs_unused_refs[] = {
    (void *)nvs_deinit,
    (void *)nvs_flash_init_partition,
    (void *)nvs_open,
    (void *)nvs_close,
    (void *)nvs_get,
    (void *)nvs_set,
    (void *)nvs_commit,
    (void *)nvs_set_value,
    (void *)nvs_erase,
    (void *)nvs_get_str,
    (void *)nvs_set_str,
    (void *)nvs_get_blob,
    (void *)nvs_set_blob,
    (void *)nvs_get_u8,
    (void *)nvs_set_u8,
    (void *)nvs_get_u16,
    (void *)nvs_set_u16,
    (void *)nvs_get_u32,
    (void *)nvs_set_u32,
    (void *)nvs_open_from_partition,
    (void *)nvs_entry_find,
    (void *)nvs_entry_find_in_handle,
    (void *)nvs_entry_next,
    (void *)nvs_entry_info,
    (void *)nvs_release_iterator,
    (void *)nvs_debug_verify_ref_counts,
};
