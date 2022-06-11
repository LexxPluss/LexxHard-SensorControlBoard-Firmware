/*
 * Copyright (c) 2022, LexxPluss Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <zephyr.h>
#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <logging/log.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "sdlog_controller.hpp"

namespace lexxhard::sdlog_controller {

LOG_MODULE_REGISTER(log);

char __aligned(4) msgq_buffer[8 * sizeof (msg)];

class directory_list {
public:
    enum class ORDER {ASCENT, DESCENT};
    directory_list() {
        reset();
    }
    uint32_t list(const char *path, ORDER order = ORDER::ASCENT) {
        uint32_t count{0};
        reset();
        fs_dir_t dir;
        fs_dir_t_init(&dir);
        if (fs_opendir(&dir, path) == 0) {
            while (fs_readdir(&dir, &dirent) == 0 && dirent.name[0] != 0) {
                if (dirent.type == FS_DIR_ENTRY_FILE) {
                    push(order, dirent.name);
                    ++count;
                }
            }
            fs_closedir(&dir);
        }
        return count;
    }
    const char *operator[](int index) const {
        return entries[index];
    }
    static constexpr uint32_t MAX_ENTRIES{30};
private:
    void reset() {
        for (auto &i : entries)
            i[0] = '\0';
    }
    void push(ORDER order, const char* str) {
        for (uint32_t i{0}; i < MAX_ENTRIES; ++i) {
            if (entries[i][0] == 0) {
                stor(entries[i], str);
                break;
            } else {
                int comp{strcmp(str, entries[i])};
                if ((order == ORDER::ASCENT && comp < 0) ||
                    (order == ORDER::DESCENT && comp > 0)) {
                    for (uint32_t j{MAX_ENTRIES - 1}; j > i; --j) {
                        if (entries[j - 1][0] != 0)
                            stor(entries[j], entries[j - 1]);
                    }
                    stor(entries[i], str);
                    break;
                }
            }
        }
    }
    void stor(char *to, const char *from) const {
        snprintf(to, MAX_FNAME, "%s", from);
    }
    fs_dirent dirent;
    static constexpr uint32_t MAX_FNAME{MAX_FILE_NAME + 1};
    char entries[MAX_ENTRIES][MAX_FNAME];
};

class log_util {
public:
    void init(const char *root) {
        snprintf(workpath, sizeof workpath, "%s/log", root);
        struct fs_dirent ent;
        if (fs_stat(workpath, &ent) == -ENOENT)
            fs_mkdir(workpath);
    }
    void maintain_log_area(const char *root) {
        reduce_file_count(root);
        reduce_disk_volume(root);
    }
    void setup_new_log(const char *root) {
        snprintf(workpath, sizeof workpath, "%s/log", root);
        list.list(workpath, directory_list::ORDER::DESCENT);
        int log_number{0};
        if (list[0][0] != '\0') {
            int n{atoi(&list[0][2])};
            if (n >= 999999) {
                rotate_log(root, n);
                list.list(workpath, directory_list::ORDER::DESCENT);
                if (list[0][0] != '\0')
                    n = atoi(&list[0][2]);
            }
            log_number = n + 1;
        }
        snprintf(workpath, sizeof workpath, "%s/log/mb%06u.log", root, log_number);
        fs_file_t_init(&logfp);
        fs_open(&logfp, workpath, FS_O_WRITE | FS_O_CREATE | FS_O_APPEND);
    }
    void write(const char *message) {
        fs_write(&logfp, message, strlen(message));
        fs_sync(&logfp);
    }
private:
    void reduce_file_count(const char *root) {
        snprintf(workpath, sizeof workpath, "%s/log", root);
        uint32_t count{list.list(workpath)};
        if (count > MAX_FILE_COUNT) {
            uint32_t n{count - MAX_FILE_COUNT};
            if (n > list.MAX_ENTRIES)
                n = list.MAX_ENTRIES;
            for (uint32_t i{0}; i < n; ++i) {
                const char *name{list[i]};
                if (name[0] != '\0') {
                    snprintf(workpath, sizeof workpath, "%s/log/%s", root, name);
                    fs_unlink(workpath);
                }
            }
        }
    }
    void reduce_disk_volume(const char *root) {
        int32_t freeMB{get_freeMB(root)};
        if (freeMB > 0 && freeMB < MIN_FREE_MB) {
            list.list(workpath);
            for (uint32_t i{0}; i < list.MAX_ENTRIES; ++i) {
                const char *name{list[i]};
                if (name[0] != '\0') {
                    snprintf(workpath, sizeof workpath, "%s/log/%s", root, name);
                    fs_unlink(workpath);
                    if (get_freeMB(root) >= MIN_FREE_MB)
                        break;
                }
            }
        }
    }
    int32_t get_freeMB(const char *root) const {
        int32_t freeMB{-1};
        struct fs_statvfs statvfs;
        if (fs_statvfs(root, &statvfs) == 0)
            freeMB = static_cast<int64_t>(statvfs.f_frsize) * static_cast<int64_t>(statvfs.f_bfree) / 1000000LL;
        return freeMB;
    }
    void rotate_log(const char *root, int last_log_number) {
        int first_log_number{0};
        snprintf(workpath, sizeof workpath, "%s/log", root);
        list.list(workpath);
        if (list[0][0] != '\0')
            first_log_number = atoi(&list[0][2]);
        for (int from{first_log_number}, to{0}; from < last_log_number; ++from, ++to) {
            snprintf(workpath, sizeof workpath, "%s/log/mb%06u.log", root, from);
            snprintf(workpath2, sizeof workpath2, "%s/log/mb%06u.log", root, to);
            fs_rename(workpath, workpath2);
        }
    }
    directory_list list;
    fs_file_t logfp;
    char workpath[PATH_MAX], workpath2[PATH_MAX];
    static constexpr int32_t MIN_FREE_MB{1024};
    static constexpr uint32_t MAX_FILE_COUNT{500};
};

class sdlog_controller_impl {
public:
    int init() {
        k_msgq_init(&msgq, msgq_buffer, sizeof (msg), 8);
        return 0;
    }
    void run() {
        if (disk_access_init("SD") == 0) {
            LOG_INF("SD OK\n");
            mount.type = FS_FATFS;
            mount.fs_data = &fatfs;
            mount.mnt_point = sdroot;
            if (fs_mount(&mount) == 0) {
                util.init(sdroot);
                util.maintain_log_area(sdroot);
                util.setup_new_log(sdroot);
                fs_ok = true;
            }
        }
        if (!fs_ok)
            return;
        while (true) {
            msg message;
            if (k_msgq_get(&msgq, &message, K_MSEC(1000)) == 0)
                util.write(message.message);
        }
    }
private:
    FATFS fatfs;
    fs_mount_t mount;
    log_util util;
    bool fs_ok{false};
    static const char *sdroot;
} impl;
const char *sdlog_controller_impl::sdroot{"/SD:"};

void init()
{
    impl.init();
}

void run(void *p1, void *p2, void *p3)
{
    impl.run();
}

void output(const char *fmt, ...)
{
    msg message;
    va_list arg;
    va_start(arg, fmt);
    vsnprintk(message.message, sizeof message.message, fmt, arg);
    va_end(arg);
    while (k_msgq_put(&msgq, &message, K_NO_WAIT) != 0)
        k_msgq_purge(&msgq);
}

k_thread thread;
k_msgq msgq;

}

// vim: set expandtab shiftwidth=4:
