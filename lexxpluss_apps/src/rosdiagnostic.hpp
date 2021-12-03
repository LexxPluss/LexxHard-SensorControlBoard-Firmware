#pragma once

#include <cstring>

struct msg_rosdiag {
    msg_rosdiag() {
        level = OK;
        name[0] = message[0] = '\0';
        strcpy(id, "main");
    }
    msg_rosdiag(int8_t level, const char *name, const char *message) {
        this->level = level;
        strncpy(this->name, name, sizeof this->name);
        strncpy(this->message, message, sizeof this->message);
        this->name[sizeof this->name - 1] = '\0';
        this->message[sizeof this->message - 1] = '\0';
        strcpy(id, "main");
    }
    char name[24];
    char message[64];
    char id[8];
    int8_t level;
    static constexpr int8_t OK{0};
    static constexpr int8_t WARN{1};
    static constexpr int8_t ERROR{2};
    static constexpr int8_t STALE{3};
} __attribute__((aligned(4)));

struct rosdiagnostic {
    static void init();
};

extern k_msgq msgq_rosdiag;

// vim: set expandtab shiftwidth=4:
