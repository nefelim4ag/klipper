// TTY based IO
//
// Copyright (C) 2017-2021  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#define _GNU_SOURCE
#include <errno.h> // errno
#include <fcntl.h> // fcntl
#include <poll.h> // ppoll
#include <pty.h> // openpty
#include <stdio.h> // fprintf
#include <string.h> // memmove
#include <sys/stat.h> // chmod
#include <time.h> // struct timespec
#include <unistd.h> // ttyname
#include <pthread.h> // pthread_create
#include <stdatomic.h> // atomic_store
#include "board/irq.h" // irq_wait
#include "board/misc.h" // console_sendf
#include "board/pgm.h"  // READP
#include "command.h" // command_find_block
#include "internal.h" // console_setup
#include "sched.h" // sched_wake_task
#include "ringbuf.h"

static struct pollfd main_pfd[1];
static struct ring_buf outputq;
static pthread_t main;
static _Atomic int main_is_sleeping;
static _Atomic int force_shutdown;
static pthread_t reader;
static pthread_t writer;

pthread_mutex_t outputq_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t outputq_cond = PTHREAD_COND_INITIALIZER;

#define MP_TTY_IDX   0

// Report 'errno' in a message written to stderr
void
report_errno(char *where, int rc)
{
    int e = errno;
    fprintf(stderr, "Got error %d in %s: (%d)%s\n", rc, where, e, strerror(e));
}

/****************************************************************
 * Threaded IO
 ****************************************************************/
static uint8_t receive_buf[4096];
static int receive_pos;

#define QSIZE 32
struct fcall {
    uint32_t args[16];
    uint_fast16_t cmdid;
    int ack;
};
struct read_queue {
    struct fcall cq[QSIZE];
    _Atomic int tail, head;
};

struct read_queue read_queue;

static void
reader_signal(int signal) {}

void *
console_receive_buffer(void)
{
    return receive_buf;
}

// Find the command handler associated with a command
static const struct command_parser *
command_lookup_parser(uint_fast16_t cmdid)
{
    if (!cmdid || cmdid >= READP(command_index_size))
        shutdown("Invalid command");
    return &command_index[cmdid];
}

void command_enque(uint8_t *buf, uint_fast8_t msglen) {
    uint8_t *p = &buf[MESSAGE_HEADER_SIZE];
    uint8_t *msgend = &buf[msglen - MESSAGE_TRAILER_SIZE];
    int head = atomic_load(&read_queue.head);
    while (p < msgend) {
        uint_fast16_t cmdid = command_parse_msgid(&p);
        const struct command_parser *cp = command_lookup_parser(cmdid);
        p = command_parsef(p, msgend, cp, read_queue.cq[head].args);
        if (sched_is_shutdown() && !(READP(cp->flags) & HF_IN_SHUTDOWN)) {
            sched_report_shutdown();
            continue;
        }
        // identity response
        // if (cmdid == 1) {
        //     cp->func(read_queue.cq[head].args);
        //     continue;
        // }
        read_queue.cq[head].cmdid = cmdid;
        if (p < msgend)
            read_queue.cq[head].ack = 0;
        else
            read_queue.cq[head].ack = 1;
        head = (head + 1) % QSIZE;
        while (head == atomic_load(&read_queue.tail)) {
            // avoid wrap
            nanosleep(&(struct timespec){.tv_nsec = 10000}, NULL);
        }
        atomic_store(&read_queue.head, head);
        if (atomic_load(&main_is_sleeping))
            pthread_kill(main, SIGUSR1);
    }
}

static void *
tty_reader(void *_unused)
{
    memset(&read_queue, 0, sizeof(read_queue));
    while (1) {
        int ret = read(main_pfd[MP_TTY_IDX].fd, &receive_buf[receive_pos],
                       sizeof(receive_buf) - receive_pos);
        if (ret < 0) {
            if (errno != EWOULDBLOCK) {
                report_errno("read", ret);
            }
            continue;
        }

        if (ret == 15 && receive_buf[receive_pos + 14] == '\n'
            && memcmp(&receive_buf[receive_pos], "FORCE_SHUTDOWN\n", 15) == 0)
            atomic_store(&force_shutdown, 1);

        // Find and dispatch message blocks in the input
        int len = receive_pos + ret;
        uint_fast8_t pop_count, msglen = len > MESSAGE_MAX ? MESSAGE_MAX : len;
        ret = command_find_block(receive_buf, msglen, &pop_count);
        if (ret) {
            command_enque(receive_buf, pop_count);
            command_send_ack();
            len -= pop_count;
            if (len)
                memmove(receive_buf, &receive_buf[pop_count], len);
        }
        receive_pos = len;
    }

    return NULL;
}

static void *
tty_writer(void *_unused)
{
    static uint8_t buf[64];
    while (1) {
        pthread_mutex_lock(&outputq_mutex);
        while (ring_buffer_available_to_read(&outputq) == 0) {
            pthread_cond_wait(&outputq_cond, &outputq_mutex);
        }
        int len = ring_buffer_read(&outputq, buf, sizeof(buf));
        pthread_mutex_unlock(&outputq_mutex);

        int ret = write(main_pfd[MP_TTY_IDX].fd, buf, len);
        if (ret < 0) {
            report_errno("write", ret);
        }
    }

    return NULL;
}

/****************************************************************
 * Setup
 ****************************************************************/

int
set_non_blocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0) {
        report_errno("fcntl getfl", flags);
        return -1;
    }
    int ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    if (ret < 0) {
        report_errno("fcntl setfl", flags);
        return -1;
    }
    return 0;
}

int
set_close_on_exec(int fd)
{
    int ret = fcntl(fd, F_SETFD, FD_CLOEXEC);
    if (ret < 0) {
        report_errno("fcntl set cloexec", ret);
        return -1;
    }
    return 0;
}

static void
reader_signal(int signal);

int
console_setup(char *name)
{
    // Open pseudo-tty
    struct termios ti;
    memset(&ti, 0, sizeof(ti));
    int mfd, sfd, ret = openpty(&mfd, &sfd, NULL, &ti, NULL);
    if (ret) {
        report_errno("openpty", ret);
        return -1;
    }
    ret = set_close_on_exec(mfd);
    if (ret)
        return -1;
    ret = set_close_on_exec(sfd);
    if (ret)
        return -1;
    main_pfd[MP_TTY_IDX].fd = mfd;
    main_pfd[MP_TTY_IDX].events = POLLIN;

    // Create symlink to tty
    unlink(name);
    char *tname = ttyname(sfd);
    if (!tname) {
        report_errno("ttyname", 0);
        return -1;
    }
    ret = symlink(tname, name);
    if (ret) {
        report_errno("symlink", ret);
        return -1;
    }
    ret = chmod(tname, 0660);
    if (ret) {
        report_errno("chmod", ret);
        return -1;
    }

    // Make sure stderr is non-blocking
    ret = set_non_blocking(STDERR_FILENO);
    if (ret)
        return -1;

    ring_buffer_init(&outputq);
    main = pthread_self();
    pthread_create(&reader, NULL, tty_reader, NULL);
    // pthread_setschedparam(reader, SCHED_OTHER,
    //                       &(struct sched_param){.sched_priority = 0});
    pthread_create(&writer, NULL, tty_writer, NULL);
    // pthread_setschedparam(writer, SCHED_OTHER,
    //                       &(struct sched_param){.sched_priority = 0});

    struct sigaction act = {.sa_handler = reader_signal,
                            .sa_flags = SA_RESTART};
    ret = sigaction(SIGUSR1, &act, NULL);
    if (ret < 0){
        report_errno("sigaction", ret);
        return -1;
    }

    return 0;
}


/****************************************************************
 * Console handling
 ****************************************************************/

// Process any incoming commands
void
console_task(void)
{
    int tail = atomic_load(&read_queue.tail);
    int head = atomic_load(&read_queue.head);
    while (tail != head) {
        const struct command_parser *cp = command_lookup_parser(read_queue.cq[tail].cmdid);
        void (*func)(uint32_t *) = READP(cp->func);
        if (atomic_load(&force_shutdown))
            shutdown("Force shutdown command");
        func(read_queue.cq[tail].args);
        // printf("Func id: %li\n", read_queue.cq[tail].cmdid);
        if (read_queue.cq[tail].ack) {
            read_queue.cq[tail].ack = 0;
        }
        tail = (tail + 1) % QSIZE;
        atomic_store(&read_queue.tail, tail);
        head = atomic_load(&read_queue.head);
    }
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Generate message
    uint8_t buf[MESSAGE_MAX];
    uint_fast8_t msglen = command_encode_and_frame(buf, ce, args);

    pthread_mutex_lock(&outputq_mutex);
    while (ring_buffer_available_to_write(&outputq) < msglen) {
        pthread_mutex_unlock(&outputq_mutex);
        nanosleep(&(struct timespec){.tv_nsec = 5000}, NULL);
        pthread_mutex_lock(&outputq_mutex);
    }

    // Transmit message
    ring_buffer_write(&outputq, buf, msglen);
    pthread_cond_signal(&outputq_cond);
    pthread_mutex_unlock(&outputq_mutex);
}


// Sleep for the specified time or until a signal interrupts
void
console_sleep(void)
{
    atomic_store(&main_is_sleeping, 1);
    nanosleep(&(struct timespec){.tv_nsec = 1000000}, NULL);
    atomic_store(&main_is_sleeping, 0);
}
